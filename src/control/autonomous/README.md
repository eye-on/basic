# Autonomous 位置说明

本文档说明当前 autonomous 代码应该放在哪里。

## 当前位置

autonomous 已经不再放在机器人专属 `src/control/...` 目录下。

现在机器人专属 autonomous 放在：

- `src/hardware/basic_robot/autonomous.h`
- `src/hardware/basic_robot/autonomous.cpp`
- `src/hardware/second_robot/autonomous.h`
- `src/hardware/second_robot/autonomous.cpp`

## 为什么这样放

现在底盘和执行机构都已经拆成共享模块。

但是 autonomous 不是共享模块，它依赖：

- 这台机器人装配了哪些模块
- 这台机器人有哪些传感器
- 这台机器人自己的动作流程

所以 autonomous 更接近“机器人装配后的专属行为”，而不是通用 control 模块。

## 当前命名空间

当前 autonomous 命名空间是：

```cpp
namespace basic::hardware::basic_robot::autonomous
namespace basic::hardware::second_robot::autonomous
```

## `src/control/` 里还应该保留什么

`src/control/` 现在只应该保留共享控制工具与算法，例如：

- `motor_control`
- `adrc`
- `kalman`
- 与共享控制相关的说明文档

## 简单判断规则

如果代码回答的是：

“这台机器人在比赛里怎么跑这套动作？”

那它应该放进：

- `src/hardware/*_robot/autonomous.*`

如果代码回答的是：

“这个可复用控制器、底盘内核、执行机构模块怎么实现？”

那它应该放进共享层，例如：

- `include/chassis/`
- `include/mechanism/`
- `src/mechanism/`
- `src/control/`
