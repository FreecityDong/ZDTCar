# ZDTCar

基于 `ESP32-S3 + Emm_V5.0 双闭环步进 + MPU6050` 的两轮自平衡小车工程。

当前工程已实现：

- PlatformIO + Arduino 工程骨架
- MPU6050 读取与陀螺零偏校准
- Emm_V5.0 串口速度控制与状态读取
- 平衡环 / 速度环基础控制结构
- USB 串口在线调参 JSON 协议
- 本地终端串口调参工具 `tools/serial_tuner.py`

详细技术方案见：

- [docs/平衡小车技术方案.md](docs/平衡小车技术方案.md)

## 1. 硬件组成

本项目默认硬件：

- 主控：`ESP32-S3-DevKitC-1`
- 姿态传感器：`MPU6050`
- 执行机构：`2 x Emm_V5.0 闭环步进`
- 调试链路：`USB CDC / 串口`
- 电机控制链路：`UART1`

默认软件配置见：

- [platformio.ini](platformio.ini)
- [src/bsp/board_pins.h](src/bsp/board_pins.h)

## 2. 工程结构

```text
src/
  app/          主循环与任务调度
  bsp/          板级驱动、IMU、电机串口协议
  common/       公共类型与工具
  control/      互补滤波、平衡环、速度环
  safety/       故障检测与锁定
  tuning/       PID 存储与串口 JSON 协议
tools/
  serial_tuner.py    串口在线调参工具
docs/
  平衡小车技术方案.md
```

## 3. 上电前检查

上电前先确认：

1. `MPU6050` 的 `SDA/SCL` 与 [src/bsp/board_pins.h](src/bsp/board_pins.h) 一致
2. 电机串口 `RX/TX` 与 [src/bsp/board_pins.h](src/bsp/board_pins.h) 一致
3. 两个电机地址分别为 `1` 和 `2`
4. 电机驱动单独供电，`GND` 与 ESP32 共地
5. 电机电源不要只靠 USB
6. 初次测试时让车轮悬空，避免直接冲车

## 4. 构建与烧录

编译：

```bash
~/.platformio/penv/bin/pio run
```

烧录：

```bash
~/.platformio/penv/bin/pio run -t upload
```

如果需要手动指定串口：

```bash
~/.platformio/penv/bin/pio run -t upload --upload-port /dev/cu.usbmodemXXXX
```

## 5. 串口调参工具

本项目不依赖传统 GUI 上位机，默认直接使用本地终端调参。

依赖安装：

```bash
python3 -m pip install --user -r tools/requirements-serial-tuner.txt
```

列出串口：

```bash
python3 tools/serial_tuner.py --list-ports
```

连接调参工具：

```bash
python3 tools/serial_tuner.py --port /dev/cu.usbmodemXXXX --baud 115200
```

常用命令：

```text
pid
status
arm on
arm off
speed 20
turn 10
set angle kp 40
set angle kd 1.0
set speed kp 0.45
set speed ki 0.02
save
rollback
clear_fault
stop
```

## 6. 首次测试流程

首次测试请严格按下面顺序，不要跳步。

### 6.1 步骤 1：烧录成功后先连串口

```bash
python3 tools/serial_tuner.py --list-ports
python3 tools/serial_tuner.py --port /dev/cu.usbmodemXXXX --baud 115200
```

进入工具后先发：

```text
status
pid
show telemetry
```

期望结果：

- 能持续收到 telemetry
- `fault` 为 `none`
- `mode` 正常显示
- `pitch`、`gyro` 有值

### 6.2 步骤 2：验证 IMU 方向

先不要使能电机，手扶车体前后倾斜。

观察 telemetry 中：

- `pitch`
- `gyro`

要求：

- 前倾和后仰时，`pitch` 会明显变化
- `gyro` 方向也应跟着变化
- 静止时数值不能无限漂

如果方向明显反了，修改：

- [src/bsp/board_pins.h](src/bsp/board_pins.h) 中的 `kPitchAccelSign`
- [src/bsp/board_pins.h](src/bsp/board_pins.h) 中的 `kPitchGyroSign`

### 6.3 步骤 3：验证单纯速度命令

让车轮悬空，避免落地测试。

命令顺序：

```text
arm on
speed 20
stop
```

要求：

- 两个轮子都转
- 两个轮子物理方向应一致地对应“前进”
- telemetry 中 `l_rpm_t / r_rpm_t` 与 `l_rpm / r_rpm` 基本同向

如果左右某一边方向反了，修改：

- [src/bsp/board_pins.h](src/bsp/board_pins.h) 中的 `kLeftMotorSign`
- [src/bsp/board_pins.h](src/bsp/board_pins.h) 中的 `kRightMotorSign`

### 6.4 步骤 4：验证转向命令

仍然保持车轮悬空。

```text
arm on
turn 20
turn -20
stop
```

要求：

- 左右轮应产生相反方向或不同目标转速
- telemetry 中 `turn_out` 有变化

### 6.5 步骤 5：检查故障保护

执行：

```text
status
clear_fault
```

检查：

- 倾角过大时系统是否会锁定
- `fault` 是否能在安全姿态下清除

## 7. 平衡调试流程

### 7.1 第一阶段：只验证基础闭环链路

目标：

- IMU 正常
- 电机方向正常
- telemetry 正常
- 串口调参正常

这一步不要急着追求“站住”。

### 7.2 第二阶段：低风险试调

建议先用默认参数，不要大幅修改：

```text
mode manual_tune
set angle kp 38
set angle kd 0.92
set speed kp 0.45
set speed ki 0.02
```

原则：

1. 先调 `angle.kp`
2. 再调 `angle.kd`
3. 能基本站住后再调 `speed.kp / speed.ki`

### 7.3 第三阶段：扶持落地

操作建议：

1. 落地但手扶车体
2. `arm on`
3. 观察是否出现瞬间猛冲、强振荡、轮子方向反
4. 一旦异常，立即 `stop`

### 7.4 第四阶段：短时站立

当扶持状态下已比较正常，再尝试短时放手。

观察重点：

- `pitch` 是否快速回零
- `l_rpm / r_rpm` 是否剧烈抖动
- `balance_out` 是否经常打满
- 是否频繁进入 `fault`

## 8. 常见测试命令

查看 PID：

```text
pid
```

查看状态：

```text
status
```

查看最近一帧遥测：

```text
show telemetry
```

开启电机：

```text
arm on
```

关闭并停止：

```text
stop
```

修改角度环参数：

```text
set angle kp 40
set angle kd 1.0
```

修改速度环参数：

```text
set speed kp 0.5
set speed ki 0.03
```

保存当前参数：

```text
save
```

回滚到工厂参数：

```text
rollback
```

## 9. 测试时的安全要求

调试时请注意：

1. 初次测试必须让轮子悬空
2. 人要能第一时间触碰到断电或 `stop`
3. 不要在桌边、台边直接调
4. PID 参数不要大步跳变
5. 修改方向符号后一定重新验证电机方向

## 10. 已知限制

当前工程是“可运行的基础版本”，重点在：

- 打通硬件链路
- 建立在线调参能力
- 建立安全保护

还没有做的内容包括：

- 真正自动搜索 PID 参数
- 专门的图形化曲线界面
- 更复杂的多任务实时优化
- 遥控输入和路径控制

## 11. 推荐的下一步

当你完成上述测试流程后，下一步建议是：

1. 把一组稳定的 `angle/speed` 参数保存下来
2. 记录 `show telemetry` 的典型数据
3. 再继续做更激进的调参或自动整定
