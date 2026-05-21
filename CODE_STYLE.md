# Basic 项目当前代码结构

本文档描述仓库当前的代码边界。现在的原则不是“所有模块都各有一套公共头文件”，而是“只公开真正的程序契约，其余实现尽量与定义同址并留在 `.cpp` 内部”。

## 根目录

```text
basic/
├── CODE_STYLE.md
├── makefile
├── include/
├── src/
├── build/
└── vex/
```

- `makefile`：VEX V5 工程构建入口。
- `include/`：只放真实的外部契约。
- `src/`：实现文件，优先在此处收纳内部实现。
- `build/`：构建产物目录。
- `vex/`：VEX 工具链相关 make 规则。

## 当前头文件结构

```text
include/
├── vex.h
├── app/
│   └── robot.h
└── hardware/
    └── robot_selector.h
```

- `include/vex.h`
  - 引入 VEX SDK 头文件。
  - 定义 `COMPETITION` 宏。

- `include/app/robot.h`
  - 声明程序级抽象接口 `basic::app::Robot`。
  - 这是主程序与具体机器人实现之间的契约。

- `include/hardware/robot_selector.h`
  - 声明 `basic::hardware::get_current_robot()`。
  - 这是主程序获取当前机器人实现的唯一入口。

## 当前源文件结构

```text
src/
├── control/
│   ├── chassis.h
│   ├── chassis.cpp
│   ├── mechanisms.h
│   ├── mechanisms.cpp
│   └── adrc/
│       └── controller.cpp
├── executer/
│   └── main.cpp
├── hardware/
│   ├── robot_hardware.h
│   ├── sensors.h
│   ├── sensors.cpp
│   └── robots/
│       ├── basic_robot.cpp
│       └── robot_state.h
└── input/
    ├── controller.h
    └── controller.cpp
```

- `src/executer/main.cpp`
  - 程序入口。
  - 负责获取机器人、初始化、启动后台线程并注册 `competition` 回调。

- `src/hardware/robots/basic_robot.cpp`
  - 只负责装配 `BasicRobot`。
  - 持有 `RobotHardware` 和 `RobotState`，并决定功能模块的调用顺序。

- `src/hardware/robot_hardware.h`
  - 定义当前机器人使用的全部外部硬件对象。
  - 这是硬件注册点，不对 `include/` 暴露。

- `src/hardware/robots/robot_state.h`
  - 定义模块之间共享的运行时状态。
  - 模块之间通过 `RobotState` 传值，不直接互相调用。

- `src/input/controller.h`
  - 声明输入模块的私有入口。

- `src/input/controller.cpp`
  - 更新手柄输入状态并写入 `RobotState`。

- `src/hardware/sensors.h`
  - 声明传感器模块的私有入口。

- `src/hardware/sensors.cpp`
  - 更新传感器状态并写入 `RobotState`。

- `src/control/chassis.h`
  - 声明底盘控制模块的私有入口。

- `src/control/chassis.cpp`
  - 读取 `RobotState` 中的输入与传感器值，计算并下发底盘输出。

- `src/control/mechanisms.h`
  - 声明机构控制模块的私有入口，对外封装4种机构动作的控制接口

- `src/control/mechanisms.cpp`
  - 读取 `RobotState` 中的输入值，更新机构动作。

- `src/control/adrc/controller.cpp`
  - ADRC 控制相关实现。
  - 目前仍独立存在，但不参与主机器人装配边界。

## 当前边界规则

- `include/` 只保留真正跨编译单元且具备程序级意义的契约。
- 只在一个机器人实现内部使用的共享类型，优先放在 `src/` 下的私有头中。
- 只在一个 `.cpp` 内部使用的辅助类型、函数和常量，必须留在该 `.cpp` 中。
- 不再新增跨模块裸 `free function` 入口。
- 如果某个行为只服务一个拥有者，就让定义和实现与拥有者同址。
- 功能模块之间通过共享状态协作，而不是通过散落在不同命名空间中的函数互调。

## 实际执行原则

创建新文件前先问自己：

“这是一个新的程序边界，还是某个已有实现对象的内部细节？”

如果它不是新的边界，就优先放回已有实现文件内部，或者放进 `src/` 下的私有实现，而不是扩张 `include/`。
