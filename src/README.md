# `src` 目录当前框架说明

本文档描述当前 `basic/src` 的代码结构。

## 当前目标

现在的框架围绕三条规则组织：

1. 机器人专属内容只留在 `src/hardware/*_robot/`
2. 底盘代码归底盘模块
3. 执行机构代码归执行机构模块

也就是说，机器人身份只应该影响：

- 选哪个底盘模块
- 选哪个执行机构模块
- autonomous 怎么写
- 端口和方向怎么装配

## 当前目录结构

```text
src/
├─ executer/
│  └─ main.cpp
├─ hardware/
│  ├─ robot_selector.cpp
│  ├─ shared/
│  │  └─ state_types.h
│  ├─ basic_robot/
│  │  ├─ autonomous.h
│  │  ├─ autonomous.cpp
│  │  ├─ basic_robot.cpp
│  │  ├─ robot_hardware.h
│  │  ├─ robot_state.h
│  │  ├─ sensors.h
│  │  └─ sensors.cpp
│  └─ second_robot/
│     ├─ autonomous.h
│     ├─ autonomous.cpp
│     ├─ second_robot.cpp
│     ├─ robot_hardware.h
│     ├─ robot_state.h
│     ├─ sensors.h
│     └─ sensors.cpp
├─ input/
│  ├─ controller.h
│  └─ controller.cpp
├─ mechanism/
│  ├─ indexed_intake.cpp
│  └─ roller_shooter.cpp
└─ control/
   ├─ motor_control.h
   ├─ motor_control.cpp
   ├─ autonomous/
   │  └─ README.md
   ├─ adrc/
   └─ kalman/
```

## `include/` 中的公共接口归类

当前 `include/` 也按职责分组：

```text
include/
├─ vex.h
├─ device_config.h
├─ app/
│  └─ robot.h
├─ hardware/
│  └─ robot_selector.h
├─ chassis/
│  ├─ arcade_drive.h
│  ├─ old_chassis.h
│  └─ second_chassis.h
└─ mechanism/
   ├─ indexed_intake.h
   └─ roller_shooter.h
```

含义是：

- `include/chassis/`：底盘驱动接口
- `include/mechanism/`：执行机构接口
- `include/app/`：程序级机器人抽象
- `include/hardware/`：机器人选择入口

## 当前底盘命名

当前显式底盘名是：

- `old_chassis`
- `second_chassis`

其中：

- `include/chassis/old_chassis.h`
  - 对应原来的底盘
- `include/chassis/second_chassis.h`
  - 对应 `second_robot` 的底盘

`include/chassis/arcade_drive.h` 只是它们共享的底层内核。

## 当前执行机构命名

当前执行机构模块是：

- `indexed_intake`
- `roller_shooter`

它们的公共接口分别在：

- `include/mechanism/indexed_intake.h`
- `include/mechanism/roller_shooter.h`

实现分别在：

- `src/mechanism/indexed_intake.cpp`
- `src/mechanism/roller_shooter.cpp`

## 每台机器人当前装配了什么

### `basic_robot`

定义见：

- `src/hardware/basic_robot/robot_hardware.h`

它当前装配：

- `old_chassis`
- `indexed_intake`
- brain / controller / inertial / sensors

### `second_robot`

定义见：

- `src/hardware/second_robot/robot_hardware.h`

它当前装配：

- `second_chassis`
- `roller_shooter`
- brain / controller / inertial

## 程序运行流程

程序入口在：

- `src/executer/main.cpp`

流程是：

1. `get_current_robot()`
2. `robot.initialize()`
3. `robot.bind_background_tasks()`
4. `robot.bind_competition(...)`

当前机器人选择由：

- `include/hardware/robot_selector.h`

控制。

## 现在手动控制放在哪里

现在已经没有：

- `src/control/basic_robot/`
- `src/control/second_robot/`

这种机器人专属控制目录了。

手动控制现在直接在机器人装配入口中接线：

- `src/hardware/basic_robot/basic_robot.cpp`
- `src/hardware/second_robot/second_robot.cpp`

这些文件负责：

- 读取控制器状态
- 生成底盘命令
- 调用底盘更新接口
- 调用执行机构更新接口

## 现在 autonomous 放在哪里

autonomous 属于“装配后的机器人行为”，所以现在放在 `hardware` 下：

- `src/hardware/basic_robot/autonomous.h`
- `src/hardware/basic_robot/autonomous.cpp`
- `src/hardware/second_robot/autonomous.h`
- `src/hardware/second_robot/autonomous.cpp`

## 常见修改入口

### 改 `basic_robot` 底盘

- `include/chassis/old_chassis.h`
- `src/hardware/basic_robot/basic_robot.cpp`

### 改 `second_robot` 底盘

- `include/chassis/second_chassis.h`
- `src/hardware/second_robot/second_robot.cpp`

### 改 `basic_robot` 执行机构

- `include/mechanism/indexed_intake.h`
- `src/mechanism/indexed_intake.cpp`

### 改 `second_robot` 执行机构

- `include/mechanism/roller_shooter.h`
- `src/mechanism/roller_shooter.cpp`

### 改 autonomous

- `src/hardware/basic_robot/autonomous.cpp`
- `src/hardware/second_robot/autonomous.cpp`

### 改端口和装配

- `src/hardware/basic_robot/robot_hardware.h`
- `src/hardware/second_robot/robot_hardware.h`

## 总结

当前结构可以概括成：

- `hardware/*_robot` 负责装配、sensors、autonomous
- `include/chassis/` 负责底盘公共接口
- `include/mechanism/` 负责执行机构公共接口
- `src/mechanism/` 负责共享执行机构实现
- `src/control/` 只负责共享控制工具和算法
