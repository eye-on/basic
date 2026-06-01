# basic 项目说明

## 1. 项目概览

这是一个基于 VEX V5 的机器人控制项目，使用 `makefile + VEXcode` 工具链构建，代码按“入口层 / 机器人装配层 / 输入层 / 控制层 / 硬件层 / 自动程序层”划分。

当前主流程的特点：

- 入口非常薄，只负责拿到当前机器人并完成初始化与比赛回调绑定。
- `BasicRobot` 负责装配整机行为，统一持有硬件对象和运行状态。
- 手动控制由手柄输入驱动，分别更新底盘和机构。
- 自动程序集中在 `src/control/autonomous/routine.cpp`。
- 背景线程持续处理颜色识别、球数统计和控制器屏幕显示。

## 2. 目录结构

```text
basic/
├── CODE_STYLE.md
├── makefile
├── include/
│   ├── app/
│   │   └── robot.h
│   ├── hardware/
│   │   └── robot_selector.h
│   └── vex.h
├── src/
│   ├── control/
│   │   ├── adrc/
│   │   ├── autonomous/
│   │   ├── kalman/
│   │   ├── chassis.cpp
│   │   ├── chassis.h
│   │   ├── mechanisms.cpp
│   │   ├── mechanisms.h
│   │   ├── motor_control.cpp
│   │   └── motor_control.h
│   ├── executer/
│   │   └── main.cpp
│   ├── hardware/
│   │   ├── robot_hardware.h
│   │   ├── sensors.cpp
│   │   ├── sensors.h
│   │   └── robots/
│   │       ├── basic_robot.cpp
│   │       └── robot_state.h
│   └── input/
│       ├── controller.cpp
│       └── controller.h
├── old/
├── build/
└── vex/
```

各目录职责：

- `include/`：放对外程序契约，主要是机器人接口和机器人选择入口。
- `src/executer/`：程序入口。
- `src/hardware/robots/`：具体机器人实现及共享状态定义。
- `src/input/`：手柄输入采集。
- `src/control/`：底盘、机构、电机控制与自动程序。
- `src/hardware/`：硬件定义、传感器更新逻辑。
- `old/`：历史版本代码，不参与当前主流程。

## 3. 主执行流程

### 3.1 程序入口

入口文件是 `src/executer/main.cpp`。

启动流程如下：

1. 调用 `basic::hardware::get_current_robot()` 获取当前机器人对象。
2. 执行 `robot.initialize()` 完成惯导校准等初始化。
3. 执行 `robot.bind_background_tasks()` 启动后台线程。
4. 如果定义了 `COMPETITION`，则绑定 VEX 比赛回调：
   - `autonomous`
   - `drivercontrol`

### 3.2 机器人装配

当前机器人实现位于 `src/hardware/robots/basic_robot.cpp`，核心类是 `BasicRobot`。

这个类内部持有：

- `robots::RobotHardware hardware_`
- `robots::RobotState state_`
- `vex::competition* competition_`

它负责三类事情：

- 初始化硬件
- 启动后台任务
- 在比赛模式下分发自动 / 手动逻辑

### 3.3 初始化流程

`initialize()` 当前主要做两件事：

- 校准惯性传感器 `inertial`
- 在手柄屏幕打印校准完成提示

### 3.4 后台线程

`bind_background_tasks()` 会启动后台线程，循环执行：

- 配置颜色传感器积分时间和补光灯
- 调用 `sensor_update(...)` 做颜色识别联动
- 调用 `count_balls_number(...)` 统计红球和蓝球数量
- 在控制器屏幕显示当前球数

### 3.5 手动控制循环

驱动阶段由 `run_driver_control_loop()` 执行。只要当前仍处于启用状态且比赛处于遥控阶段，就会持续循环：

1. `controller_update(hardware_, state_)`
2. `chassis_update(hardware_, state_)`
3. `mechanism_update(hardware_, state_)`
4. 休眠 `kRefreshTime` 毫秒

退出时会调用 `stop_all_outputs(...)` 停止所有电机输出。

### 3.6 自动程序入口

自动阶段由 `run_autonomous_routine()` 调用：

```cpp
robots::autonomous::run_routine(hardware_, state_, *competition_);
```

自动程序结束后会以 `hold` 方式停车。

## 4. 硬件配置

硬件定义在 `src/hardware/robot_hardware.h`。

### 4.1 底盘电机

8 电机差速底盘：

- 右前：`motor_fr1` `PORT14`，`motor_fr2` `PORT9`
- 右后：`motor_br1` `PORT8`，`motor_br2` `PORT7`
- 左前：`motor_fl1` `PORT6`，`motor_fl2` `PORT5`
- 左后：`motor_bl1` `PORT4`，`motor_bl2` `PORT3`

### 4.2 机构电机

- 中层机构：`middle_motor1` `PORT16`
- 下层机构：`under_motor1` `PORT11`
- 上层机构：`upper_motor1` `PORT19`
- 传送机构：`trans_motor1` `PORT12`
- 传送机构：`trans_motor2` `PORT1`
- 传送机构：`trans_motor3` `PORT15`
- 传送机构：`trans_motor4` `PORT15`

说明：

- `trans_motor3` 和 `trans_motor4` 当前都绑定在 `PORT15`，从代码上看这是重复端口定义，实机使用前应再次确认是否为预期配置。

### 4.3 展开机构

- 下翻机构：`under_overhang_motor` `PORT18`
- 上翻机构：`upper_overhang_motor` `PORT2`
- 中翻机构：`middle_overhang_motor` `PORT17`

### 4.4 传感器与控制设备

- 激光测距：`laser_rangefinder` `PORT13`
- 颜色传感器：`color_sensor` `PORT20`
- 惯性传感器：`inertial` `PORT10`
- 主控：`brain`
- 主手柄：`controller`

## 5. 状态管理

共享状态定义在 `src/hardware/robots/robot_state.h`。

`RobotState` 主要包含：

- `controller`：手柄输入状态
- `sensors`：传感器状态
- `chassis`：底盘目标输出和刹车模式
- `mechanism`：当前机构模式
- `overhang`：各翻折机构状态
- `autonomous`：自动阶段的位置、航向估计信息
- `blue_balls` / `red_balls`：球数统计

这种组织方式的好处是：

- 输入、控制、自动程序通过同一份状态协作
- 模块边界比较清楚
- 后续扩展时不需要在模块之间大量互相调用

## 6. 输入模块

输入模块位于：

- `src/input/controller.h`
- `src/input/controller.cpp`

`controller_update(...)` 每次循环会读取：

- 四个摇杆轴 `Axis1 ~ Axis4`
- `L1 / L2 / R1 / R2`
- `X / Y / A / B`
- `Up / Down / Left / Right`

同时维护：

- 当前值
- 上一周期值
- 单次按下事件 `press_x`、`press_a` 等
- 摇杆变化速率评分 `rating[]`

其中 `rating[]` 会在底盘平滑控制中使用，用来决定当前输入变化时的滤波强度。

## 7. 底盘控制

底盘控制位于：

- `src/control/chassis.h`
- `src/control/chassis.cpp`

### 7.1 控制方式

当前是双摇杆差速控制的简化形式：

- `axis2`：前后
- `axis4`：转向

四个角的目标速度按以下方式组合：

- 左侧：`axis2 + axis4`
- 右侧：`axis2 - axis4`

### 7.2 控制特点

底盘模块做了几层处理：

- `dynamic_smooth(...)`
  - 根据当前输入变化速度做动态平滑，减少突变。
- `shape_input(...)`
  - 对输入做 S 曲线整形，让小输入更细腻，大输入保留足够输出。
- 输出归一化
  - 如果组合后超过 `100%`，按比例整体缩放。

最终通过 `apply_motor_power(...)` 下发到底盘 8 个电机。

## 8. 机构控制

机构控制位于：

- `src/control/mechanisms.h`
- `src/control/mechanisms.cpp`

### 8.1 机构模式

当前机构模式枚举 `IndexedMechanismMode` 包括：

- `kOff`
- `kPreLoad`
- `kLegacyIntake`
- `kUnderTrow`
- `kMiddleThrow`
- `kUpperThrow`
- `kSortIntake`

不同模式会给 7 个机构电机分配一组固定转速表。

### 8.2 当前手柄映射

在 `mechanism_update(...)` 中，当前按键与模式的关系大致如下：

- `A`：切换 `kLegacyIntake`
- `L2`：切换 `kUnderTrow`
- `L1`：切换 `kMiddleThrow`
- `R1`：切换 `kUpperThrow`
- `R2`：切换 `kPreLoad`

翻折相关：

- `X / B`：当前直接控制上翻机构电机正反转
- `Down`：触发下翻机构模式切换
- `Left`：触发中翻机构部分收回

说明：

- `R2` 当前用的是持续按下 `input.r2`，不是单次按下 `input.press_r2`。这意味着按住 `R2` 时会在每个周期反复切换模式，实际行为可能不稳定，后续维护时应重点注意。

## 9. 传感器与球色识别

传感器逻辑位于：

- `src/hardware/sensors.h`
- `src/hardware/sensors.cpp`

### 9.1 颜色识别

当前通过颜色传感器 `hue()` 粗略识别：

- 红色：`12`
- 蓝色：`213`

识别容差当前写死为 `5`。

`sensor_update(...)` 的核心用途是：

- 当机构处于 `kLegacyIntake` 模式时
- 如果检测到目标颜色球
- 自动切换到 `kSortIntake`
- 保持一段时间后恢复为 `kLegacyIntake`

当前后台线程里调用的是：

```cpp
robots::sensor_update(hardware_, state_, vex::color::red, 10);
```

也就是说目前逻辑是围绕“识别红球”来做联动。

### 9.2 球数统计

`count_balls_number(...)` 会根据颜色传感器的颜色跳变统计：

- 吸入模式时球数加一
- 非吸入 / 出球状态时球数减一

并分别维护：

- `state.red_balls`
- `state.blue_balls`

## 10. 自动程序

自动程序位于：

- `src/control/autonomous/routine.h`
- `src/control/autonomous/routine.cpp`

### 10.1 自动能力

当前自动模块提供的能力包括：

- 展开 / 收回翻折机构
- 直线移动
- 原地转向
- 按目标位姿移动 `go_to_pose(...)`
- 相对位姿移动 `go_to_relative_pose(...)`
- 相对圆弧跟随 `follow_relative_arc(...)`
- 激光测距定距 `drive_to_laser_distance_mm(...)`

### 10.2 状态估计方法

自动程序使用：

- 左右侧驱动电机平均圈数
- 惯性传感器航向

来估计：

- `estimated_x_mm`
- `estimated_y_mm`
- `estimated_heading_deg`

这是一套简化的里程计实现，足够支撑当前的相对位姿自动控制。

### 10.3 当前自动流程概述

`run_routine(...)` 当前大致流程如下：

1. 设置底盘停车模式为 `hold`
2. 重置自动坐标系
3. 初始化中翻、上翻、下翻机构状态
4. 利用激光测距前进到约 `515 mm`
5. 左右转向并切换到 intake
6. 继续接近目标，维持 intake 一段时间
7. 后退、转身、切换投球模式
8. 利用一系列直线 / 转向 / 定距动作完成后续路径
9. 最后执行中层投射并停车

这说明当前自动程序已经不是简单的“固定距离前进”，而是混合了：

- 激光定距
- 位姿规划
- 机构联动
- 延时和动作等待

### 10.4 可调参数

`routine.cpp` 顶部集中放置了大量自动参数，例如：

- 轮周长换算参数
- 转向比例系数
- 最小 / 最大速度
- 加减速窗口
- 定距容差
- 位姿容差
- 超时参数

后续调自动时，优先修改这些常量，不要直接在流程里写魔法数字逻辑。

## 11. 电机控制封装

电机基础封装位于：

- `src/control/motor_control.h`
- `src/control/motor_control.cpp`

封装内容包括：

- 百分比速度控制
- 速度单位控制
- 扭矩近似控制
- 停车控制
- 电机位置 / 转速 / 电流 / 电压 / 功率 / 扭矩 / 效率 / 温度读取

这层封装的价值在于：

- 统一所有模块对电机的调用方式
- 减少上层业务代码直接散落 SDK 调用

## 12. 预研与未接入主流程的模块

当前仓库里还存在两类模块：

- `src/control/adrc/`
- `src/control/kalman/`

它们已经有独立实现，但从当前主流程调用关系看，尚未接入 `BasicRobot` 的运行路径，暂时更像是算法储备或实验代码。

此外还有：

- `old/VEXU_pushback/`

这是旧版本工程，和当前 `basic` 主工程是分开的，阅读当前代码时可以作为历史参考，但不属于当前运行版本。

## 13. 当前代码的几个注意点

阅读现有代码时，建议优先注意下面这些地方：

- `trans_motor3` 与 `trans_motor4` 使用了同一个端口 `PORT15`，需要确认是否为笔误。
- `mechanism_update()` 中 `R2` 使用持续按下而不是单次边沿触发，可能导致模式反复切换。
- 后台线程会持续修改机构模式相关状态，后续如果扩展多线程逻辑，需要注意状态竞争问题。
- `sensor_update(...)` 目前目标颜色写死为红色，如果比赛策略改变，需要同步调整。
- `print_laser_data_csv()` 和 `print_chassic_data_csv()` 目前存在但未接入主流程，可作为调试辅助函数看待。

## 14. 构建方式

项目根目录下的 `makefile` 使用 VEX 提供的 `vex/mkenv.mk` 与 `vex/mkrules.mk`。

当前构建特点：

- C++ 标准被显式切到 `gnu++17`
- 会自动收集 `src/` 下多层目录中的 `.cpp` / `.c` 文件
- 头文件搜索路径包含 `include` 和 `src`

如果本地工具链配置正确，通常可在项目根目录直接构建。

## 15. 维护建议

如果后续继续演进这个项目，建议按下面思路维护：

- 保持 `main.cpp` 继续只做入口绑定，不在里面加业务逻辑。
- 新硬件优先放进 `RobotHardware`，保持端口定义集中。
- 新运行状态优先补到 `RobotState`，保持模块协作方式一致。
- 新自动动作优先加到 `routine.cpp` 的能力函数层，不要全部直接堆进 `run_routine()`。
- 如果 `adrc` 或 `kalman` 要正式接入，先明确它们接入的是底盘控制、姿态控制还是传感器滤波，不要在多个层次同时试接。

## 16. 快速总结

这个仓库当前已经形成了一个比较明确的机器人控制骨架：

- `main.cpp` 负责启动
- `BasicRobot` 负责装配
- `RobotHardware` 负责硬件注册
- `RobotState` 负责状态共享
- `controller / chassis / mechanisms / sensors` 负责手动控制链路
- `autonomous::run_routine()` 负责自动路径执行

如果要继续扩展功能，优先沿着这条结构演进，会比把新逻辑直接散落到各处更容易维护。
