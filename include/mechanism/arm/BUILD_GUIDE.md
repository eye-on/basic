# Arm 机械臂简明使用文档

## 1. 头文件

```cpp
#include "mechanism/arm/arm.h"
```

推荐用命名空间别名：

```cpp
namespace arm_mech = basic::mechanism::arm;
```

## 2. `ArmConfig`

用于初始化机械臂：

```cpp
struct ArmConfig {
  basic::device::MotorConfig joint1_motor;
  basic::device::MotorConfig joint2_motor;
  basic::device::MotorConfig joint3_motor;
  basic::device::MotorConfig joint4_motor;
  ArmIkConfig ik_config{};
  vex::rotationUnits command_units{vex::deg};
  vex::velocityUnits move_speed_units{vex::velocityUnits::pct};
  double move_speed{30.0};
};
```

主要字段：

- `joint1_motor ~ joint4_motor`：四个关节电机配置，内容是 `port / gear_ratio / reversed`
- `ik_config.l1`：大臂长度
- `ik_config.l2e`：小臂有效长度
- `ik_config.coordinate_sign`：装配方向，正常一般填 `1`，镜像装配可试 `-1`
- `ik_config.joint_limits`：四个关节限位，单位是弧度
- `ik_config.motor_mapping`：关节角和电机角的映射
  - `q1/q2` 默认带 `1:3` 减速箱，即 `gearbox_ratio = 3.0`
- `command_units`：电机位置命令单位，通常用 `vex::deg`
- `move_speed_units`：速度单位，通常用 `vex::pct`
- `move_speed`：运动速度

## 3. `ArmCommand`

用于给机械臂下发目标：

```cpp
struct ArmCommand {
  ArmPoint target{};
  double q4_reference{0.0};
  bool hold_q4{true};
  bool enabled{false};
};
```

字段说明：

- `target`：目标点 `{x, y, z}`
- `q4_reference`：第四关节参考角，单位是弧度
- `hold_q4`
  - `true`：保持当前第四关节角度
  - `false`：使用 `q4_reference`
- `enabled`
  - `true`：执行动作
  - `false`：直接停机

## 4. 最小可用示例

```cpp
#include "mechanism/arm/arm.h"

namespace arm_mech = basic::mechanism::arm;

arm_mech::Arm arm = arm_mech::arm_init({
    {1, vex::ratio36_1, false},
    {2, vex::ratio36_1, false},
    {3, vex::ratio36_1, false},
    {4, vex::ratio36_1, false},
    {
        200.0,
        180.0,
        1e-6,
        false,
        1,
        {{
            {-arm_mech::kArmPi, arm_mech::kArmPi},
            {0.0, arm_mech::kArmPi},
            {arm_mech::arm_degrees_to_radians(29.0),
             -arm_mech::arm_degrees_to_radians(71.0)},
            {-arm_mech::kArmPi, arm_mech::kArmPi},
        }},
        {{1.0, 1.0, 1.0, 0.0}},
        {{
            {1.0, 180.0 / arm_mech::kArmPi, 0.0},
            {1.0, 180.0 / arm_mech::kArmPi, 0.0},
            {1.0, 180.0 / arm_mech::kArmPi, 0.0},
            {1.0, 180.0 / arm_mech::kArmPi, 0.0},
        }},
    },
    vex::deg,
    vex::pct,
    30.0,
});

void move_arm() {
  arm_mech::arm_update(arm, {
      {150.0, 0.0, 100.0},
      0.0,
      true,
      true,
  });
}
```

## 5. 常用接口

### `arm_update`

给机械臂发送目标点：

```cpp
arm_mech::arm_update(arm, {
    {150.0, 0.0, 100.0},
    0.0,
    true,
    true,
});
```

说明：

- 内部会先做逆解，再驱动四个电机
- 如果目标不可达或超限，会直接停机

### `arm_stop`

停止机械臂：

```cpp
arm_mech::arm_stop(arm);
```

也可以指定刹车方式：

```cpp
arm_mech::arm_stop(arm, vex::hold);
arm_mech::arm_stop(arm, vex::brake);
arm_mech::arm_stop(arm, vex::coast);
```

### `arm_state`

读取上一次命令和逆解结果：

```cpp
const auto& state = arm_mech::arm_state(arm);
```

常看这几个字段：

- `state.last_command`
- `state.last_solution.status`
- `state.last_solution.reachable`
- `state.last_solution.joint_angles`
- `state.last_solution.motor_angles`

## 6. 参数标定建议

### 连杆长度

- `l1` 和 `l2e` 必须真实
- `target.x/y/z` 必须和它们使用同一单位

### 电机方向

- 先检查 `MotorConfig.reversed`
- 再检查 `motor_mapping.direction`

如果动作方向反了，通常就是这两个地方有一个不对。

### 零位

- `motor_mapping.zero_offset` 必须对应“几何关节角为 0 时”的编码器值
- 这个值不对，机械臂就会跑偏
- 如果你手里有“当前关节角度”和“当前单圈编码器值 0-3600”，可以直接用：

```cpp
double zero_offset = arm_mech::arm_calculate_zero_offset_from_angle(
    29.0,
    1250.0);
```

这里：

- 第一个参数：手动输入的关节角度，单位是度
- 第二个参数：当前电机单圈编码器值，范围 `0-3600`

默认按：

- `direction = 1.0`
- `units_per_radian = 3600 / (2 * pi)`

计算

### 角度单位

- `joint_limits` 用弧度
- `q4_reference` 用弧度
- `command_units` 一般用 `vex::deg`

### 坐标系

- `target` 必须是肩关节坐标系下的点
- 如果你的点来自底盘坐标系或别的坐标系，要先转换

### 镜像装配

- 正常填 `coordinate_sign = 1`
- 如果整套结构是镜像安装，试 `coordinate_sign = -1`

## 7. 一句话使用流程

1. 配好 `ArmConfig`
2. 用 `arm_init(...)` 初始化
3. 用 `arm_update(...)` 给目标点
4. 用 `arm_state(...)` 看结果
5. 需要停机时用 `arm_stop(...)`
