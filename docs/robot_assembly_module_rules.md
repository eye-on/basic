# 机器人装配与模块化规则（RULES）

> 本文档固化本仓库的装配/模块化约定。任何新增机器人、模块或接线改动都必须遵守。
> 关联：`CODE_STYLE.md`、`src/README.md`、`include/mechanism/arm/BUILD_GUIDE.md`。

## 1. 目录分层规则

```text
include/   ← 公共接口（纯头文件区）
  app/robot.h                   机器人抽象基类（initialize/bind_background_tasks/bind_competition）
  hardware/robot_selector.h     机器人选择：枚举 RobotIdentity + kSelectedRobot 常量
  device_config.h               设备描述：MotorConfig{port, gear_ratio, reversed} / DigitalOutConfig{port&}
  chassis/  mechanism/  vision/  模块公共头
src/       ← 实现 + 机器人专属 + 入口
  executer/main.cpp             入口（禁止放业务）
  hardware/shared/state_types.h 共享状态（ControllerInputState / SensorState / AutonomousState）
  hardware/<robot>_robot/       一台机器人一个文件夹
  input/  mechanism/  control/  vision/   共享实现（makefile wildcard 自动收集 src/**/*.cpp，加文件零配置）
```

- 构建系统 include 路径 = `include` + `src`（故 `input/controller.h` 等头文件可放 src 下被 #include "input/controller.h" 引用）
- 命名空间即路径：`basic::hardware::new_robot` ↔ `src/hardware/new_robot/`

## 2. 启动装配链规则

```cpp
main() → basic::hardware::get_current_robot()   // 单例
       → robot.initialize()                     // 惯量校准 + 屏幕提示
       → robot.bind_background_tasks()           // 后台传感器线程（50ms 节拍）
       → robot.bind_competition(competition)     // 绑定 autonomous / drivercontrol
```

**切换机器人只改一处**：`include/hardware/robot_selector.h` 的 `kSelectedRobot`；
`robot_selector.cpp` 的 switch 必须给每台机器人加 case（`xxx_robot::get_robot()`）。

## 3. 机器人文件夹模板规则（6 文件）

```text
src/hardware/<name>/robot_hardware.h   物理装配：RobotHardware 结构体 + 构造函数初始化列表用 xxx_init({Config}) 装配
src/hardware/<name>/robot_state.h      状态：共享 ControllerInputState + 用到的模块 State
src/hardware/<name>/sensors.h/.cpp     后台传感器循环 sensor_update(hw, state)
src/hardware/<name>/autonomous.h/.cpp  自走 run_routine(hw, state, competition)
src/hardware/<name>/<name>.cpp         class <Name>Robot final : public basic::app::Robot + 文件内匿名命名空间单例
```

Robot 子类统一骨架：三个 static 入口（start_background_tasks / start_driver_control_entry /
start_autonomous_entry）→ 成员方法；`should_run_driver_control()` 判断 `competition_->isEnabled() && isDriverControl()`。

**Driver 控制循环统一流水线（10ms 节拍）**：

```cpp
while (should_run_driver_control()) {
  controller_update(hw.brain, hw.controller, state.controller);          // 1. 唯一读手柄入口
  chassis_update(hw.chassis, chassis_command_from_controller(state.controller));   // 2..n 每个模块一行
  mechanism_update(hw.lift, lift_command_from_controller(state.controller));
  sleep(kRefreshTime);  // 10
}
stop_all_outputs(brake_type);  // 退出统一停机：底盘 + 所有机构全部 stop
```

## 4. 模块接口四件套规则（所有 chassis / mechanism 模块必须遵守）

```cpp
xxx_init(Config)                                       // 装配；Config 只用 device_config 的设备类型描述端口
xxx_command_from_controller(const ControllerInputState&) // 手柄→指令（键位映射归属模块自己）
xxx_update(Hw&, const Command&)                        // 周期执行（无阻塞，由调用方定节拍）
xxx_stop(Hw&, brake_type = ...)                        // 停机
xxx_state(Hw&)                                         // 状态读写（const 重载成对）
```

- 头文件：Config / Command / State / class + 上述自由函数声明（参照 `include/mechanism/roller_shooter.h`）
- 实现放 `src/mechanism/xxx.cpp` / `include/chassis/` 纯内联 + `src/`（参照 roller_shooter.cpp / x_chassis.h 包装内核模式）
- 底盘模块常分「内核（如 XDrive/arcade_drive）+ 命名包装层（x_chassis.h）」两层，包装层只做类型别名与转发

## 5. 手柄输入规则

- **唯一入口**：`basic::input::controller_update(brain, controller, state)`，把所有摇杆（axis1-4，含 last_*）与按键（含 last_*/press_* 边沿）写入 `ControllerInputState`
- 模块指令生成一律消费该共享状态，禁止在机器人循环里直接读 controller
- toggle/单拍动作只用 `press_*` 边沿标志（按下次沿触发一次），长按不重复
- 按键冲突管理：每个模块的 command_from_controller 内注释声明占用键位

## 6. 给现有机器人加入机构模块（3 文件）

1. `robot_hardware.h`：#include 模块头 → 结构体加成员 → 构造函数初始化列表 `xxx_init({端口配置})`
2. `robot_state.h`：#include 模块头 → RobotState 加 `XxxState` 字段
3. `<name>.cpp`：#include 模块头 → driver 循环加 `xxx_update(hw.xxx, xxx_command_from_controller(state.controller))` → `stop_all_outputs` 加 `xxx_stop(hw.xxx)`

示例：looklook（x_chassis + heading_hold + linear_lift + gripper）、second_robot（second_chassis + roller_shooter）。

## 7. 编写新模块（5 步）

1. `include/mechanism/<name>.h`（或 chassis 内核+包装）：Config/Command/State/class + 四件套声明
2. `src/mechanism/<name>.cpp` 实现 —— 无需改 makefile
3. 需要手柄控制 → 实现 `command_from_controller`（轴 / press_* 边沿）
4. 需要新控制算法 → 放 `include/control`（pid/adrc/kalman/gearbox/software_gearbox），模块内复用
5. 按第 6 节把模块接到目标机器人

## 8. 新增机器人（6 步）

1. 复制 `src/hardware/<name>/` 六文件骨架（模板见第 3 节）
2. `robot_hardware.h`：按**物理实际接线**填 MotorConfig 端口/齿比/正反、ADI 数字口（`DigitalOutConfig{brain.ThreeWirePort.X}`）、编码器标零值
3. `robot_selector.h` 枚举加一项 → `robot_selector.cpp` 前置声明 + switch case
4. 编译 → 烧录测试
5. 物理上电验证：编码器方向反 → `reversed`/`analog_reversed`；动作反 → 检查 MotorConfig.reversed
6. PROS 版若需同步 → 移植到 `../my_robot`（见第 10 节）

## 9. 装配参数约定（接线/标零/调试）

- 电机默认 `ratio6_1` 蓝盒；配置里注明 `motor_max_rpm = 600`
- 三线模拟编码器：0-4095 绝对位置；滞环死区 raw 域（±8 ≈ ±0.70°）；`analog_zero_raw = 0` 表示上电自动捕获当前位置
- 物理回正目标为持久化标零值：误差只用 wrap_180 最短路径（0° 与 180° 是不同位置）；回正容差 1.0°、超时 2.5s
- 力控（电压直驱）约定：PID 输出 ±200 pct 预算 → ×120 mV/pct 下发 move_voltage；停止 = 零电压自然滑行；静摩擦 kick（转向 3%、驱动 2%）
- 摇杆死区 ±3 pct；手柄映射（VEX 轴号）：轴3(左Y)=前后、轴4(左X)=平移、轴1(右X)=旋转，量程 ×127/100 对齐 PROS
- 调试打印：模块级 10Hz 节流，一行 <60B/轮×4，115200 波特安全；全部设备打印统一格式 `FR|a:..rpm b:..rpm steer_v:.. wheel_v:.. tgt:..`
- 接线/标零数值改动必须**同步 basic 与 my_robot（PROS）两工程**，并更新注释中的实测日期与值

## 10. PROS（my_robot）同步规则

- `../my_robot` 是 basic new_robot 的 PROS 移植：接线、标零值、PID、手柄映射保持一致
- 构建/烧录环境：`pros.exe` 在 `C:\Users\a\AppData\Local\Python\pythoncore-3.14-64\Scripts\`；
  工具链 `C:\Users\a\Documents\local\pros-toolchain\extracted\usr`（需 `$env:PROS_TOOLCHAIN` 指到它）
- 烧录：主控 USB 连接后 `pros upload --after run`（槽位 1）；`pros lsusb` 查设备；`pros terminal` 看串口
- 手柄 A 键（按下沿）切换循环打印四轮编码器原始值（10Hz）——标零实测用

## 11. Git 约定

- 分支 `VCS`；直连 GitHub 443 不通，推送需走本机代理：
  `git -c http.proxy=http://127.0.0.1:7897 -c https.proxy=http://127.0.0.1:7897 push origin VCS`
- 提交消息用中文、一句话概括动机（如「new_robot 更新实测标零值：E=385」）
- 只改动目标文件；`.vscode/settings.json` 索引 stat 过期时不要提交

## 附录 A：当前机器人清单与接线（2026-08-29 实测）

| 轮位 | 电机端口 | 编码器 ADI | 标零值 | 运动学位置 |
|---|---|---|---|---|
| 右前 FR | 1, 2 | B | 3567 | 前右 |
| 左前 FL | 11, 13 | A | 3465 | 前左 |
| 右后 BR | 9, 10 | C | 1383 | 后右 |
| 左后 BL | 19, 20 | E | 385 | 后左 |

选择器：`kSelectedRobot = kNewRobot`（舵轮底盘，移植同步对象 my_robot）。

## 附录 B：机器人清单（2026-08-29）

| 枚举 | 目录 | 底盘 | 备注 |
|---|---|---|---|
| kBasicRobot | basic_robot | old_chassis | indexed_intake |
| kSecondRobot | second_robot | second_chassis | roller_shooter |
| kNewRobot | new_robot | new_chassis（舵轮 steering） | 同步对象 my_robot（PROS） |
| kFootballRobot | football_robot | h_chassis | pneumatic_motor_actuator + vision |
| kLooklook | looklook | x_chassis + heading_hold | linear_lift + gripper |
| kBed | bed | bed_chassis = XDrive\<2,2,2,2\>（每轮 2 电机共 8） | intake（L1）+ pneumatic_gripper（R1）+ arm_2dof（上/下、X/B；暂开环速度）；未接线，端口占位 + TODO |
