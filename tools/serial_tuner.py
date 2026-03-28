#!/usr/bin/env python3
"""Interactive serial tuner for the ESP32 balance car firmware.

This tool reads JSON telemetry from the USB debug serial port and sends JSON
commands to the firmware at runtime. It is intended to replace a traditional
GUI "upper computer" with a terminal-based workflow.
"""

from __future__ import annotations

import argparse
import glob
import json
import queue
import shlex
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any

try:
    import serial  # type: ignore
    import serial.tools.list_ports  # type: ignore
except Exception:  # pragma: no cover - dependency is optional at import time
    serial = None


def list_candidate_ports() -> list[str]:
    if serial is not None:
        return [port.device for port in serial.tools.list_ports.comports()]

    candidates: list[str] = []
    for pattern in ("/dev/tty.usb*", "/dev/cu.usb*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        candidates.extend(glob.glob(pattern))
    return sorted(set(candidates))


def print_json(data: dict[str, Any]) -> None:
    print(json.dumps(data, ensure_ascii=False))


def format_float(value: Any, digits: int = 2) -> str:
    try:
        return f"{float(value):.{digits}f}"
    except Exception:
        return "nan"


@dataclass
class RuntimeState:
    telemetry: dict[str, Any] = field(default_factory=dict)
    pid: dict[str, Any] = field(default_factory=dict)
    status: dict[str, Any] = field(default_factory=dict)
    last_assist: str | None = None
    telemetry_count: int = 0
    compact_mode: bool = True
    show_ack: bool = True
    show_raw: bool = False


class SerialTuner:
    def __init__(self, port: str, baudrate: int, timeout: float = 0.1) -> None:
        if serial is None:
            raise RuntimeError(
                "Missing dependency: pyserial. Install with `python3 -m pip install pyserial`."
            )

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.state = RuntimeState()
        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            dsrdtr=False,
            rtscts=False,
        )
        self._serial.dtr = False
        self._serial.rts = False
        self._running = True
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._message_queue: queue.Queue[dict[str, Any]] = queue.Queue()
        self._last_compact_print = 0.0

    def start(self) -> None:
        self._reader_thread.start()

    def stop(self) -> None:
        self._running = False
        try:
            self._serial.close()
        except Exception:
            pass

    def send(self, payload: dict[str, Any]) -> None:
        line = json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n"
        self._serial.write(line.encode("utf-8"))

    def drain_messages(self) -> None:
        while True:
            try:
                message = self._message_queue.get_nowait()
            except queue.Empty:
                break
            self._handle_message(message)

    def _reader_loop(self) -> None:
        while self._running:
            try:
                raw = self._serial.readline()
            except Exception as exc:
                print(f"[serial] read error: {exc}")
                self._running = False
                return

            if not raw:
                continue

            try:
                line = raw.decode("utf-8", errors="replace").strip()
            except Exception:
                continue

            if not line:
                continue

            if self.state.show_raw:
                print(f"[raw] {line}")

            try:
                message = json.loads(line)
            except json.JSONDecodeError:
                print(f"[non-json] {line}")
                continue

            if isinstance(message, dict):
                self._message_queue.put(message)

    def _handle_message(self, message: dict[str, Any]) -> None:
        message_type = message.get("type", "")

        if message_type == "telemetry":
            self.state.telemetry = message
            self.state.telemetry_count += 1
            self._print_telemetry(message)
            return

        if message_type == "pid":
            self.state.pid = message
            print("[pid]")
            print_json(message)
            return

        if message_type == "status":
            self.state.status = message
            print("[status]")
            print_json(message)
            return

        if message_type == "assist":
            self.state.last_assist = str(message.get("suggestion", ""))
            print(f"[assist] {self.state.last_assist}")
            return

        if message_type == "ack":
            if self.state.show_ack:
                print("[ack]")
                print_json(message)
            return

        if message_type == "error":
            print("[error]")
            print_json(message)
            return

        print("[message]")
        print_json(message)

    def _print_telemetry(self, message: dict[str, Any]) -> None:
        if not self.state.compact_mode:
            print("[telemetry]")
            print_json(message)
            return

        now = time.monotonic()
        if now - self._last_compact_print < 0.08:
            return
        self._last_compact_print = now

        fault = message.get("fault", "none")
        mode = message.get("mode", "unknown")
        armed = "ARM" if message.get("armed", False) else "SAFE"
        pitch = format_float(message.get("pitch"))
        gyro = format_float(message.get("gyro"))
        l_target = format_float(message.get("l_rpm_t"))
        r_target = format_float(message.get("r_rpm_t"))
        l_rpm = format_float(message.get("l_rpm"))
        r_rpm = format_float(message.get("r_rpm"))
        balance = format_float(message.get("balance_out"))
        speed = format_float(message.get("speed_out"))
        turn = format_float(message.get("turn_out"))
        sat = int(bool(message.get("sat", 0)))

        print(
            f"[tel] {armed} mode={mode} fault={fault} "
            f"pitch={pitch} gyro={gyro} "
            f"L={l_target}/{l_rpm} R={r_target}/{r_rpm} "
            f"out(b/s/t)={balance}/{speed}/{turn} sat={sat}"
        )


def command_help() -> None:
    print(
        """
Commands:
  help
  pid                             Request current PID config
  status                          Request current status
  save                            Save active PID to NVS and factory slot
  rollback                        Restore factory PID
  clear_fault                     Clear latched fault when pitch is safe
  mode <idle|manual_tune|assist_tune|auto_tune_prepare|auto_tune_running|auto_tune_evaluate|auto_tune_apply|auto_tune_rollback|fault_lock>
  arm <on|off>
  speed <rpm>
  turn <rpm>
  motor_test <left|right|both> <rpm>
  stop                            Shortcut for arm off + speed 0 + turn 0
  set <angle|speed|turn> <kp|ki|kd|integral_limit|output_limit> <value>
  gains <angle|speed|turn> <kp> <ki> [kd] [integral_limit] [output_limit]
  limit rpm <value>
  limit turn <value>
  limit pitch <value>
  limit accel <0-255>
  limit telemetry <ms>
  compact <on|off>                Compact telemetry output
  ack <on|off>                    Show ack packets
  raw <on|off>                    Show raw serial lines
  json <raw json>                 Send a raw JSON command line
  show telemetry                  Print last telemetry snapshot
  show pid                        Print last PID snapshot
  ports                           List local serial candidates
  quit
""".strip()
    )


def bool_arg(text: str) -> bool:
    lowered = text.strip().lower()
    if lowered in {"1", "on", "true", "yes"}:
        return True
    if lowered in {"0", "off", "false", "no"}:
        return False
    raise ValueError(f"invalid bool value: {text}")


def handle_command(tuner: SerialTuner, line: str) -> bool:
    try:
        args = shlex.split(line)
    except ValueError as exc:
        print(f"[cli] parse error: {exc}")
        return True

    if not args:
        return True

    cmd = args[0].lower()

    if cmd in {"quit", "exit"}:
        return False

    if cmd == "help":
        command_help()
        return True

    if cmd == "pid":
        tuner.send({"cmd": "get_pid"})
        return True

    if cmd == "status":
        tuner.send({"cmd": "get_status"})
        return True

    if cmd == "save":
        tuner.send({"cmd": "save_pid"})
        return True

    if cmd == "rollback":
        tuner.send({"cmd": "rollback_pid"})
        return True

    if cmd == "clear_fault":
        tuner.send({"cmd": "clear_fault"})
        return True

    if cmd == "mode" and len(args) == 2:
        tuner.send({"cmd": "set_mode", "mode": args[1]})
        return True

    if cmd == "arm" and len(args) == 2:
        tuner.send({"cmd": "set_drive", "arm": bool_arg(args[1])})
        return True

    if cmd == "speed" and len(args) == 2:
        tuner.send({"cmd": "set_drive", "speed_rpm": float(args[1])})
        return True

    if cmd == "turn" and len(args) == 2:
        tuner.send({"cmd": "set_drive", "turn_rpm": float(args[1])})
        return True

    if cmd == "stop":
        tuner.send({"cmd": "set_drive", "arm": False, "speed_rpm": 0.0, "turn_rpm": 0.0})
        return True

    if cmd == "motor_test" and len(args) == 3:
        target = args[1].lower()
        if target not in {"left", "right", "both"}:
            print("[cli] motor_test target must be left/right/both")
            return True
        tuner.send({"cmd": "motor_test", "target": target, "rpm": float(args[2])})
        return True

    if cmd == "set" and len(args) == 4:
        group, key, value = args[1], args[2], float(args[3])
        tuner.send({"cmd": "set_pid", "group": group, key: value})
        return True

    if cmd == "gains" and 4 <= len(args) <= 7:
        payload: dict[str, Any] = {
            "cmd": "set_pid",
            "group": args[1],
            "kp": float(args[2]),
            "ki": float(args[3]),
        }
        if len(args) >= 5:
            payload["kd"] = float(args[4])
        if len(args) >= 6:
            payload["integral_limit"] = float(args[5])
        if len(args) >= 7:
            payload["output_limit"] = float(args[6])
        tuner.send(payload)
        return True

    if cmd == "limit" and len(args) == 3:
        key_map = {
            "rpm": "rpm_limit",
            "turn": "turn_limit",
            "pitch": "pitch_target",
            "accel": "motor_accel",
            "telemetry": "telemetry_ms",
        }
        key = key_map.get(args[1].lower())
        if key is None:
            print("[cli] unsupported limit key")
            return True
        value: Any = int(args[2]) if args[1].lower() in {"accel", "telemetry"} else float(args[2])
        tuner.send({"cmd": "set_limit", key: value})
        return True

    if cmd == "compact" and len(args) == 2:
        tuner.state.compact_mode = bool_arg(args[1])
        print(f"[cli] compact={tuner.state.compact_mode}")
        return True

    if cmd == "ack" and len(args) == 2:
        tuner.state.show_ack = bool_arg(args[1])
        print(f"[cli] ack={tuner.state.show_ack}")
        return True

    if cmd == "raw" and len(args) == 2:
        tuner.state.show_raw = bool_arg(args[1])
        print(f"[cli] raw={tuner.state.show_raw}")
        return True

    if cmd == "json" and len(args) >= 2:
        raw_json = line.partition(" ")[2]
        tuner.send(json.loads(raw_json))
        return True

    if cmd == "show" and len(args) == 2:
        if args[1] == "telemetry":
            print_json(tuner.state.telemetry)
            return True
        if args[1] == "pid":
            print_json(tuner.state.pid)
            return True
        if args[1] == "status":
            print_json(tuner.state.status)
            return True
        print("[cli] unknown show target")
        return True

    if cmd == "ports":
        for port in list_candidate_ports():
            print(port)
        return True

    print("[cli] unknown command, type `help`")
    return True


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Serial tuner for the balance car firmware")
    parser.add_argument("--port", help="Serial device path, such as /dev/cu.usbmodemXXXX")
    parser.add_argument("--baud", type=int, default=115200, help="Debug serial baud rate")
    parser.add_argument(
        "--raw", action="store_true", help="Print raw serial lines in addition to parsed output"
    )
    parser.add_argument(
        "--no-compact", action="store_true", help="Disable compact telemetry formatting"
    )
    parser.add_argument("--list-ports", action="store_true", help="List local serial candidates")
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    if args.list_ports:
        for port in list_candidate_ports():
            print(port)
        return 0

    if serial is None:
        print("Missing dependency: pyserial")
        print("Install with: python3 -m pip install pyserial")
        return 1

    port = args.port
    if not port:
        candidates = list_candidate_ports()
        if len(candidates) == 1:
            port = candidates[0]
            print(f"[cli] auto-selected port: {port}")
        else:
            print("Please specify --port. Candidates:")
            for candidate in candidates:
                print(f"  {candidate}")
            return 1

    tuner = SerialTuner(port=port, baudrate=args.baud)
    tuner.state.compact_mode = not args.no_compact
    tuner.state.show_raw = args.raw
    tuner.start()

    print(f"[cli] connected to {port} @ {args.baud}")
    command_help()

    try:
        while True:
            tuner.drain_messages()
            try:
                line = input("> ")
            except EOFError:
                break
            tuner.drain_messages()
            if not handle_command(tuner, line):
                break
    except KeyboardInterrupt:
        print()
    finally:
        tuner.stop()

    return 0


if __name__ == "__main__":
    sys.exit(main())
