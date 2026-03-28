#!/usr/bin/env python3
"""Automatic PID closed-loop tuner for ZDTCar firmware.

This script runs a repeatable command sequence on the board, reads telemetry,
scores stability/tracking, and iteratively adjusts PID parameters.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import statistics
import sys
import time
from dataclasses import dataclass
from typing import Any

try:
    import serial  # type: ignore
    import serial.tools.list_ports  # type: ignore
except Exception:
    serial = None


def list_candidate_ports() -> list[str]:
    if serial is not None:
        ports = [port.device for port in serial.tools.list_ports.comports()]
        return sorted(ports)

    candidates: list[str] = []
    for pattern in ("/dev/tty.usb*", "/dev/cu.usb*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        candidates.extend(glob.glob(pattern))
    return sorted(set(candidates))


@dataclass
class ParamSpec:
    name: str
    group: str
    key: str
    minimum: float
    maximum: float
    step: float
    min_step: float


@dataclass
class TrialResult:
    score: float
    valid: bool
    fault: str
    samples: int
    pitch_peak: float
    pitch_mean: float
    rpm_error: float
    sat_ratio: float
    fall_delta: int


class AutoTuner:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        if serial is None:
            raise RuntimeError("Missing dependency pyserial. Install: pip install pyserial")

        self.ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=0.08,
            dsrdtr=False,
            rtscts=False,
        )
        self.ser.dtr = False
        self.ser.rts = False
        self.specs = self._build_specs(args)

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass

    @staticmethod
    def _build_specs(args: argparse.Namespace) -> list[ParamSpec]:
        all_specs = {
            "angle.kp": ParamSpec("angle.kp", "angle", "kp", 10.0, 90.0, 2.0, 0.25),
            "angle.kd": ParamSpec("angle.kd", "angle", "kd", 0.2, 3.5, 0.08, 0.01),
            "speed.kp": ParamSpec("speed.kp", "speed", "kp", 0.05, 3.0, 0.05, 0.005),
            "speed.ki": ParamSpec("speed.ki", "speed", "ki", 0.0, 1.2, 0.01, 0.002),
        }

        requested = [part.strip() for part in args.params.split(",") if part.strip()]
        specs: list[ParamSpec] = []
        for name in requested:
            if name not in all_specs:
                raise ValueError(f"Unsupported param in --params: {name}")
            specs.append(all_specs[name])

        if not specs:
            raise ValueError("No tunable params selected")
        return specs

    def send(self, payload: dict[str, Any]) -> None:
        line = json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n"
        self.ser.write(line.encode("utf-8"))
        self.ser.flush()

    def _read_messages(self, duration: float) -> list[dict[str, Any]]:
        end = time.monotonic() + duration
        out: list[dict[str, Any]] = []

        while time.monotonic() < end:
            raw = self.ser.readline()
            if not raw:
                continue

            try:
                line = raw.decode("utf-8", errors="replace").strip()
            except Exception:
                continue

            if not line:
                continue
            if self.args.raw:
                print(f"[raw] {line}")

            try:
                obj = json.loads(line)
            except Exception:
                continue
            if isinstance(obj, dict):
                out.append(obj)

        return out

    def _drain(self, duration: float = 0.3) -> None:
        self._read_messages(duration)

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

    def _arm(self, enabled: bool) -> None:
        self.send({"cmd": "set_drive", "arm": enabled})

    def _stop(self) -> None:
        self.send({"cmd": "set_drive", "arm": False, "speed_rpm": 0.0, "turn_rpm": 0.0})

    def _clear_fault(self) -> None:
        self.send({"cmd": "clear_fault"})

    def _request_pid(self) -> dict[str, Any] | None:
        for _ in range(3):
            self._drain(0.1)
            self.send({"cmd": "get_pid"})
            messages = self._read_messages(1.2)
            for msg in reversed(messages):
                if msg.get("type") == "pid":
                    return msg
        return None

    def _request_status(self) -> dict[str, Any] | None:
        for _ in range(3):
            self._drain(0.1)
            self.send({"cmd": "get_status"})
            messages = self._read_messages(1.0)
            for msg in reversed(messages):
                if msg.get("type") == "status":
                    return msg
        return None

    @staticmethod
    def _extract_params_from_pid(pid: dict[str, Any]) -> dict[str, float]:
        angle = pid.get("angle", {})
        speed = pid.get("speed", {})
        return {
            "angle.kp": float(angle.get("kp", 0.0)),
            "angle.kd": float(angle.get("kd", 0.0)),
            "speed.kp": float(speed.get("kp", 0.0)),
            "speed.ki": float(speed.get("ki", 0.0)),
        }

    def _set_mode_and_limits(self) -> None:
        self.send({"cmd": "set_mode", "mode": "manual_tune"})
        self._read_messages(0.25)
        self.send({"cmd": "set_limit", "pitch_target": self.args.pitch_target})
        self._read_messages(0.25)
        self.send({"cmd": "set_limit", "rpm_limit": self.args.rpm_limit})
        self._read_messages(0.25)
        self.send({"cmd": "set_limit", "turn_limit": self.args.turn_limit})
        self._read_messages(0.25)

    def _apply_params(self, params: dict[str, float]) -> None:
        payload_angle: dict[str, Any] = {"cmd": "set_pid", "group": "angle"}
        payload_speed: dict[str, Any] = {"cmd": "set_pid", "group": "speed"}

        payload_angle["kp"] = params["angle.kp"]
        payload_angle["kd"] = params["angle.kd"]
        payload_speed["kp"] = params["speed.kp"]
        payload_speed["ki"] = params["speed.ki"]

        self.send(payload_angle)
        self._read_messages(0.35)
        self.send(payload_speed)
        self._read_messages(0.35)
        self.send({"cmd": "get_pid"})
        self._read_messages(0.45)

    def _segment(self, duration: float, speed_rpm: float | None = None, turn_rpm: float | None = None) -> list[dict[str, Any]]:
        drive_payload: dict[str, Any] = {"cmd": "set_drive"}
        if speed_rpm is not None:
            drive_payload["speed_rpm"] = speed_rpm
        if turn_rpm is not None:
            drive_payload["turn_rpm"] = turn_rpm
        self.send(drive_payload)
        return self._read_messages(duration)

    def _telemetry_samples(self, messages: list[dict[str, Any]]) -> list[dict[str, Any]]:
        return [msg for msg in messages if msg.get("type") == "telemetry"]

    def _evaluate_samples(self, samples: list[dict[str, Any]]) -> TrialResult:
        if not samples:
            return TrialResult(
                score=9e9,
                valid=False,
                fault="no_telemetry",
                samples=0,
                pitch_peak=999.0,
                pitch_mean=999.0,
                rpm_error=999.0,
                sat_ratio=1.0,
                fall_delta=999,
            )

        pitch_abs = [abs(float(s.get("pitch", 0.0))) for s in samples]
        rpm_error = [
            0.5
            * (
                abs(float(s.get("l_rpm_t", 0.0)) - float(s.get("l_rpm", 0.0)))
                + abs(float(s.get("r_rpm_t", 0.0)) - float(s.get("r_rpm", 0.0)))
            )
            for s in samples
        ]
        sat_ratio = sum(1.0 for s in samples if bool(s.get("sat", False))) / float(len(samples))

        fall_start = int(samples[0].get("fall_count", 0))
        fall_end = int(samples[-1].get("fall_count", 0))
        fall_delta = max(0, fall_end - fall_start)

        fault = "none"
        for s in reversed(samples):
            fault_value = str(s.get("fault", "none"))
            if fault_value != "none":
                fault = fault_value
                break

        pitch_peak = max(pitch_abs)
        pitch_mean = statistics.fmean(pitch_abs)
        rpm_error_mean = statistics.fmean(rpm_error)

        score = (
            pitch_peak * 8.0
            + pitch_mean * 4.5
            + rpm_error_mean * 0.32
            + sat_ratio * 120.0
            + fall_delta * 800.0
        )

        valid = True
        if fault != "none":
            score += 3000.0
            valid = False
        if pitch_peak >= self.args.abort_pitch_deg:
            score += 2000.0
            valid = False

        return TrialResult(
            score=score,
            valid=valid,
            fault=fault,
            samples=len(samples),
            pitch_peak=pitch_peak,
            pitch_mean=pitch_mean,
            rpm_error=rpm_error_mean,
            sat_ratio=sat_ratio,
            fall_delta=fall_delta,
        )

    def run_trial(self, params: dict[str, float]) -> TrialResult:
        self._drain(0.15)
        self._clear_fault()
        self._read_messages(0.25)
        self._set_mode_and_limits()
        self._apply_params(params)

        all_messages: list[dict[str, Any]] = []
        self._arm(True)
        all_messages.extend(self._read_messages(0.35))

        segments = [
            (self.args.settle_seconds, 0.0, 0.0),
            (self.args.segment_seconds, self.args.speed_rpm, 0.0),
            (self.args.segment_seconds, 0.0, 0.0),
            (self.args.segment_seconds, 0.0, self.args.turn_rpm),
            (self.args.segment_seconds, 0.0, -self.args.turn_rpm),
            (self.args.settle_seconds, 0.0, 0.0),
        ]

        aborted = False
        for duration, speed, turn in segments:
            messages = self._segment(duration, speed_rpm=speed, turn_rpm=turn)
            all_messages.extend(messages)
            samples = self._telemetry_samples(messages)
            if not samples:
                continue

            latest = samples[-1]
            if str(latest.get("fault", "none")) != "none":
                aborted = True
                break
            if abs(float(latest.get("pitch", 0.0))) >= self.args.abort_pitch_deg:
                aborted = True
                break

        self._stop()
        all_messages.extend(self._read_messages(0.4))
        if aborted:
            self._clear_fault()
            all_messages.extend(self._read_messages(0.3))
            self.send({"cmd": "set_mode", "mode": "manual_tune"})
            all_messages.extend(self._read_messages(0.25))

        samples = self._telemetry_samples(all_messages)
        return self._evaluate_samples(samples)

    def run(self) -> int:
        print(f"[auto_tune] connected {self.args.port} @ {self.args.baud}")
        self._drain(0.5)

        self._clear_fault()
        self._read_messages(0.4)
        self._set_mode_and_limits()

        pid = self._request_pid()
        if pid is None:
            print("[auto_tune] failed to fetch initial PID")
            return 2

        best_params = self._extract_params_from_pid(pid)
        initial_params = dict(best_params)
        print(f"[auto_tune] initial params: {best_params}")

        steps = {spec.name: spec.step for spec in self.specs}
        best_result = self.run_trial(best_params)
        print(
            "[auto_tune] baseline "
            f"score={best_result.score:.2f} valid={best_result.valid} fault={best_result.fault} "
            f"samples={best_result.samples} pitch_peak={best_result.pitch_peak:.2f} "
            f"pitch_mean={best_result.pitch_mean:.2f} rpm_err={best_result.rpm_error:.2f} "
            f"sat={best_result.sat_ratio:.2f} fall_delta={best_result.fall_delta}"
        )
        if not best_result.valid:
            print(
                "[auto_tune] baseline is invalid. "
                "Check wiring/power/motor feedback before expecting useful tuning."
            )

        for iteration in range(1, self.args.max_iterations + 1):
            improved = False
            print(f"\n[auto_tune] iteration {iteration}")

            for spec in self.specs:
                current_value = best_params[spec.name]
                step = steps[spec.name]
                if step < spec.min_step:
                    continue

                candidates = []
                for direction in (-1.0, 1.0):
                    candidate_value = self._clamp(current_value + direction * step, spec.minimum, spec.maximum)
                    if math.isclose(candidate_value, current_value, rel_tol=0.0, abs_tol=1e-9):
                        continue
                    candidate_params = dict(best_params)
                    candidate_params[spec.name] = candidate_value
                    candidates.append(candidate_params)

                for idx, candidate in enumerate(candidates, start=1):
                    print(
                        f"[auto_tune] trial {spec.name} ({idx}/{len(candidates)}) -> "
                        f"{candidate[spec.name]:.6g}"
                    )
                    result = self.run_trial(candidate)
                    print(
                        "[auto_tune] result "
                        f"score={result.score:.2f} valid={result.valid} fault={result.fault} "
                        f"samples={result.samples} pitch_peak={result.pitch_peak:.2f} "
                        f"pitch_mean={result.pitch_mean:.2f} rpm_err={result.rpm_error:.2f} "
                        f"sat={result.sat_ratio:.2f} fall_delta={result.fall_delta}"
                    )

                    should_accept = False
                    if result.valid and not best_result.valid:
                        should_accept = True
                    elif result.valid == best_result.valid and (
                        result.score + self.args.improve_margin < best_result.score
                    ):
                        should_accept = True

                    if should_accept:
                        best_result = result
                        best_params = candidate
                        improved = True
                        print(f"[auto_tune] accepted new best: {best_params}")

            if not improved:
                all_small = True
                for spec in self.specs:
                    steps[spec.name] *= self.args.step_decay
                    if steps[spec.name] >= spec.min_step:
                        all_small = False
                print(f"[auto_tune] no improvement, decayed steps: {steps}")
                if all_small:
                    print("[auto_tune] all step sizes reached min threshold, stopping")
                    break

        print("\n[auto_tune] applying best params")
        if not best_result.valid:
            best_params = initial_params
            print("[auto_tune] no valid trial found, restored initial params")
        self._clear_fault()
        self._read_messages(0.25)
        self._set_mode_and_limits()
        self._apply_params(best_params)

        status = self._request_status()
        if status is not None:
            print(f"[auto_tune] final status: {json.dumps(status, ensure_ascii=False)}")

        print(
            "[auto_tune] best summary "
            f"score={best_result.score:.2f} fault={best_result.fault} "
            f"pitch_peak={best_result.pitch_peak:.2f} pitch_mean={best_result.pitch_mean:.2f} "
            f"rpm_err={best_result.rpm_error:.2f} sat={best_result.sat_ratio:.2f} "
            f"fall_delta={best_result.fall_delta}"
        )
        print(f"[auto_tune] best params: {best_params}")

        if self.args.save_best:
            self.send({"cmd": "save_pid"})
            msgs = self._read_messages(0.8)
            ack = None
            for msg in reversed(msgs):
                if msg.get("type") == "ack" and msg.get("cmd") == "save_pid":
                    ack = msg
                    break
            print(f"[auto_tune] save_pid ack: {json.dumps(ack or {'ok': False}, ensure_ascii=False)}")

        self._stop()
        self._read_messages(0.25)
        return 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Automatic PID tuner for ZDTCar")
    parser.add_argument("--port", help="Serial port, e.g. /dev/cu.usbmodemXXXX")
    parser.add_argument("--baud", type=int, default=115200, help="Debug serial baud")
    parser.add_argument("--list-ports", action="store_true", help="List serial ports and exit")
    parser.add_argument(
        "--params",
        default="angle.kp,angle.kd,speed.kp,speed.ki",
        help="Comma separated params to tune",
    )
    parser.add_argument("--max-iterations", type=int, default=4, help="Max coordinate-descent iterations")
    parser.add_argument("--segment-seconds", type=float, default=1.6, help="Duration of each active segment")
    parser.add_argument("--settle-seconds", type=float, default=0.8, help="Duration of settle segments")
    parser.add_argument("--speed-rpm", type=float, default=20.0, help="Speed command during test")
    parser.add_argument("--turn-rpm", type=float, default=20.0, help="Turn command during test")
    parser.add_argument("--rpm-limit", type=float, default=280.0, help="Safety rpm limit")
    parser.add_argument("--turn-limit", type=float, default=80.0, help="Safety turn limit")
    parser.add_argument("--pitch-target", type=float, default=0.9, help="Pitch target in degree")
    parser.add_argument(
        "--abort-pitch-deg",
        type=float,
        default=10.0,
        help="Abort current trial if abs(pitch) exceeds this",
    )
    parser.add_argument("--improve-margin", type=float, default=0.1, help="Minimum score improvement to accept")
    parser.add_argument("--step-decay", type=float, default=0.6, help="Step decay factor when no improvement")
    parser.add_argument("--save-best", action="store_true", help="Save tuned PID to NVS at the end")
    parser.add_argument("--raw", action="store_true", help="Print raw serial JSON lines")
    return parser


def resolve_port(args: argparse.Namespace) -> str:
    if args.port:
        return args.port

    candidates = list_candidate_ports()
    if len(candidates) == 1:
        print(f"[auto_tune] auto-selected port: {candidates[0]}")
        return candidates[0]

    if not candidates:
        raise RuntimeError("No serial ports found. Specify --port")

    raise RuntimeError(
        "Multiple serial ports found. Specify --port explicitly: " + ", ".join(candidates)
    )


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    if args.list_ports:
        for p in list_candidate_ports():
            print(p)
        return 0

    if serial is None:
        print("Missing dependency: pyserial")
        print("Install with: python3 -m pip install pyserial")
        return 1

    try:
        args.port = resolve_port(args)
    except Exception as exc:
        print(exc)
        return 1

    tuner = AutoTuner(args)
    try:
        return tuner.run()
    except KeyboardInterrupt:
        print("\n[auto_tune] interrupted")
        return 130
    finally:
        tuner.close()


if __name__ == "__main__":
    sys.exit(main())
