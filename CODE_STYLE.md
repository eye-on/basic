# Basic 当前代码规范与结构说明

本文档描述当前仓库的代码边界与放置规则。

当前结构的核心原则是：

1. 底盘归底盘模块。
2. 执行机构归执行机构模块。
3. 机器人专属装配只放在 `src/hardware/*_robot/`。
4. `src/control/` 只保留共享控制工具与算法。
5. `include/` 只放公共接口与程序级契约。

## 根目录结构

```text
basic/
├─ CODE_STYLE.md
├─ makefile
├─ include/
├─ src/
├─ build/
└─ vex/
```

- `makefile`：VEX 工程构建入口
- `include/`：公共接口头文件
- `src/`：实现文件与项目内部实现
- `build/`：构建产物
- `vex/`：VEX 工具链规则

## `include/` 当前归类

现在 `include/` 下的头文件按职责分组。

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

### 各类头文件职责

#### `include/vex.h`

- 引入 VEX SDK
- 放项目级宏

#### `include/device_config.h`

- 放最底层的设备配置结构
- 例如：
  - `MotorConfig`
  - `DigitalOutConfig`

#### `include/app/robot.h`

- 定义程序级机器人抽象接口 `basic::app::Robot`
- 主程序只依赖这个接口，不依赖具体机器人实现

#### `include/hardware/robot_selector.h`

- 定义当前机器人选择逻辑
- 声明 `get_current_robot()`

#### `include/chassis/`

这一组头文件都属于底盘驱动接口。

- `arcade_drive.h`
  - 共享底盘内核
  - 是底层通用模板，不是直接面向具体机器人命名的最终接口

- `old_chassis.h`
  - 原底盘模块的对外接口
  - 当前 `basic_robot` 装配它

- `second_chassis.h`
  - `second_robot` 底盘模块的对外接口

#### `include/mechanism/`

这一组头文件都属于执行机构接口。

- `indexed_intake.h`
  - 原车执行机构模块接口

- `roller_shooter.h`
  - `second_robot` 执行机构模块接口

## `src/` 当前结构

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

## 各层职责

### `src/executer/`

- 只放程序入口
- 不放机器人专属控制逻辑

### `src/hardware/shared/`

- 放共享状态定义
- 比如控制器输入状态、传感器状态、自动状态

### `src/hardware/basic_robot/` 与 `src/hardware/second_robot/`

这是机器人身份唯一应该出现的地方。

这里只负责：

- 装配底盘模块
- 装配执行机构模块
- 绑定端口与方向
- 定义机器人专属 autonomous
- 定义机器人专属 sensors
- 维护机器人生命周期入口

不负责保存可复用的底盘实现或执行机构实现。

### `src/mechanism/`

- 放共享执行机构实现
- 当前包括：
  - `indexed_intake.cpp`
  - `roller_shooter.cpp`

如果某个执行机构以后可以被不同机器人复用，它的实现就应该放这里。

### `src/control/`

- 只放共享控制工具与算法
- 当前包括：
  - `motor_control`
  - `adrc`
  - `kalman`

不要再在这里新增机器人专属 `chassis.cpp` 或 `mechanisms.cpp`。

## 命名规则

### 底盘命名

不要再使用“某机器人目录下一个泛化的 `chassis.cpp`”这种方式。

当前显式底盘名是：

- `old_chassis`
- `second_chassis`

新底盘模块也应采用“模块自身名字”，而不是“挂在机器人名下的通用 chassis 文件名”。

### 执行机构命名

执行机构也应以模块本身命名：

- `indexed_intake`
- `roller_shooter`

不要再新增泛化的机器人专属 `mechanisms.cpp` 适配层。

### 机器人装配命名

机器人装配层保持显式命名：

- `basic_robot.cpp`
- `second_robot.cpp`
- `robot_hardware.h`
- `robot_state.h`
- `autonomous.h`
- `autonomous.cpp`

## 文件放置规则

### 什么应该放进 `include/`

只有以下内容应该放进 `include/`：

- 可复用模块的公共接口
- 程序级抽象接口
- 跨模块共享且具有公共意义的配置结构

不要把所有内部辅助头文件都塞进 `include/`。

### 什么应该放进 `include/chassis/`

如果头文件回答的是：

- 这个底盘模块如何初始化
- 这个底盘模块如何更新
- 这个底盘模块暴露什么状态与接口

那它就属于 `include/chassis/`。

### 什么应该放进 `include/mechanism/`

如果头文件回答的是：

- 这个执行机构模块如何初始化
- 这个执行机构模块如何更新
- 这个执行机构模块暴露什么状态与接口

那它就属于 `include/mechanism/`。

### 什么应该放进 `src/hardware/*_robot/`

如果代码回答的是：

- 这台机器人装配了哪些模块
- 这些模块接了哪些端口
- 这台机器人怎样接手柄输入
- 这台机器人跑什么 autonomous

那它属于 `src/hardware/*_robot/`。

### 什么应该放进 `src/mechanism/`

如果代码回答的是：

- 一个可复用执行机构内部如何工作
- 如何解释它的命令
- 如何维护它自己的状态

那它属于 `src/mechanism/`。

### 什么应该放进 `src/control/`

如果代码回答的是：

- 一个通用控制器如何工作
- 一个通用滤波器如何工作
- 一个通用控制工具如何工作

那它属于 `src/control/`。

## 实际修改入口

### 修改 old 车底盘

- `include/chassis/old_chassis.h`
- `src/hardware/basic_robot/basic_robot.cpp`

### 修改 second_robot 底盘

- `include/chassis/second_chassis.h`
- `src/hardware/second_robot/second_robot.cpp`

### 修改 old 车执行机构

- `include/mechanism/indexed_intake.h`
- `src/mechanism/indexed_intake.cpp`

### 修改 second_robot 执行机构

- `include/mechanism/roller_shooter.h`
- `src/mechanism/roller_shooter.cpp`

### 修改机器人装配

- `src/hardware/basic_robot/robot_hardware.h`
- `src/hardware/second_robot/robot_hardware.h`

### 修改 autonomous

- `src/hardware/basic_robot/autonomous.cpp`
- `src/hardware/second_robot/autonomous.cpp`

## 总结

当前结构不是：

- 每个机器人在 `src/control/` 下各带一套底盘和执行机构控制文件

当前结构是：

- 公共底盘接口放进 `include/chassis/`
- 公共执行机构接口放进 `include/mechanism/`
- 共享执行机构实现放进 `src/mechanism/`
- 共享控制工具放进 `src/control/`
- 机器人身份只保留在 `src/hardware/*_robot/`
