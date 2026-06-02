# Arm 机械臂逆解与机构接口说明

本文说明当前仓库中四自由度机械臂模块的接口、逆运动学逻辑和代码行为。

对应代码文件：

- `include/mechanism/arm/arm.h`
- `src/mechanism/arm/arm.cpp`
- `include/mechanism/arm/inverse_kinematics.h`
- `src/mechanism/arm/inverse_kinematics.cpp`

## 1. 当前模块结构

当前实现分成两层：

1. `inverse_kinematics`
   - 只负责几何逆解
   - 输入目标点、连杆参数、上一拍关节角
   - 输出一组几何关节角和映射后的电机角

2. `arm`
   - 是和仓库里现有 `mechanism` 模块风格一致的机构封装
   - `arm_init(config)` 根据电机端口初始化 4 个关节电机
   - `arm_update(mechanism, command)` 接收目标命令，内部先做逆解，再把角度发给电机

也就是说，外部正常使用时优先面向 `Arm` 这一层，逆解层作为其内部计算核心。

## 2. 机构假设

这套逆解默认机械臂满足以下假设：

- `q1`：肩部方位角
- `q2`：肩部俯仰角
- `q3`：肘部相对角，`q3 = 0` 表示大臂小臂共线伸直
- `q4`：小臂绕自身轴线滚转

并且满足关键约束：

- 大臂和小臂始终在同一个平面内运动
- 这个运动平面始终垂直于肩部安装平面

因此位置逆解可以拆成：

1. 由 `(x, y)` 决定 `q1`
2. 由 `(rho, z)` 在二维平面中决定 `q2`、`q3`
3. `q4` 不影响位于小臂轴线上的点的位置，需要额外指定或保持当前值

## 3. 坐标与参数定义

目标点定义为肩关节坐标系下的点：

```text
p = [x, y, z]^T
```

代码中对应：

```cpp
struct ArmPoint {
  double x;
  double y;
  double z;
};
```

连杆参数定义在 `ArmIkConfig` 中：

```cpp
struct ArmIkConfig {
  double l1;
  double l2e;
  double rho_epsilon;
  bool clamp_unreachable_target;
  std::array<ArmJointLimit, 4> joint_limits;
  std::array<double, 4> continuity_weights;
  std::array<ArmMotorMapping, 4> motor_mapping;
};
```

其中：

- `l1`：大臂长度
- `l2e`：有效小臂长度
- `rho_epsilon`：靠近基座轴线时判断奇异的阈值
- `clamp_unreachable_target`：不可达时是否裁剪到工作空间边界
- `joint_limits`：几何关节角限位
- `continuity_weights`：分支连续性代价权重
- `motor_mapping`：几何角到电机角的方向和零位映射

## 4. 逆解核心公式

代码实现和几何说明一致。

先计算：

```text
rho = hypot(x, y)
d   = hypot(rho, z)
```

### 4.1 肩部方位角 `q1`

当目标点不在基座法向轴附近时：

```text
q1 = atan2(y, x)
```

在代码里，如果：

```text
rho < rho_epsilon
```

则不重新解 `q1`，而是沿用上一拍 `previous_joint_angles.q1`，并把状态标记为：

```cpp
ArmIkStatus::kSingularBaseAxis
```

对应实现位置：

- `src/mechanism/arm/inverse_kinematics.cpp`
- `solve_branch(...)`

### 4.2 肘部角 `q3`

由余弦定理：

```text
c3 = (d^2 - l1^2 - l2e^2) / (2 * l1 * l2e)
```

代码里先做夹紧：

```text
c3 = clamp(c3, -1, 1)
```

再按两个分支取：

```text
s3 = +/- sqrt(max(0, 1 - c3^2))
q3 = atan2(s3, c3)
```

代码中用：

```cpp
enum class ArmElbowBranch {
  kNegative = -1,
  kPositive = 1,
};
```

代表两组肘部解。

### 4.3 肩部俯仰角 `q2`

按标准二连杆公式：

```text
alpha = atan2(z, rho)
beta  = atan2(l2e * s3, l1 + l2e * c3)
q2    = alpha - beta
```

### 4.4 小臂滚转角 `q4`

当前实现中 `q4` 不是由位置逆解出来，而是外部给定：

- 如果 `ArmCommand.hold_q4 == true`
  - 使用当前电机位置反算得到的当前 `q4`
- 否则
  - 使用 `ArmCommand.q4_reference`

这与机构本身的几何性质一致：位置约束无法唯一决定 `q4`。

## 5. 分支选择策略

这套二连杆逆解天然有两组解。

代码会同时计算：

- `ArmElbowBranch::kPositive`
- `ArmElbowBranch::kNegative`

然后按以下顺序选择：

1. 先检查每组解是否满足 `joint_limits`
2. 如果两组都有效，则比较连续性代价
3. 代价更小的一组作为最终解
4. 如果两组都超限，则返回 `kNoValidSolution`

代价函数在 `joint_cost(...)` 中定义：

```text
cost =
  w1 * wrap(q1 - q1_prev)^2 +
  w2 * wrap(q2 - q2_prev)^2 +
  w3 * wrap(q3 - q3_prev)^2 +
  w4 * wrap(q4 - q4_prev)^2
```

对应配置项：

```cpp
std::array<double, 4> continuity_weights
```

默认值是：

```cpp
{{1.0, 1.0, 1.0, 0.0}}
```

即默认只用 `q1/q2/q3` 的连续性做分支选择，不用 `q4`。

## 6. 不可达点处理

工作空间判定条件为：

```text
abs(l1 - l2e) <= d <= l1 + l2e
```

实现中先算目标点到肩关节原点的距离：

```text
target_distance = sqrt(x^2 + y^2 + z^2)
```

若超出工作空间：

- `clamp_unreachable_target == false`
  - 返回 `ArmIkStatus::kUnreachable`

- `clamp_unreachable_target == true`
  - 调用 `clamp_target_to_workspace(...)`
  - 将目标点沿原点到目标点方向裁剪到最近的工作空间边界
  - 再基于裁剪后的点求逆解

返回结果里：

- `target` 是原始输入目标
- `solved_target` 是真正参与逆解的点
- `reachable` 表示原始点是否本来就在工作空间内

## 7. 几何角与电机角映射

逆解结果先得到几何角：

```cpp
struct ArmJointAngles {
  double q1;
  double q2;
  double q3;
  double q4;
};
```

再按下面公式映射到电机角：

```text
motor = direction * joint + zero_offset
```

对应配置：

```cpp
struct ArmMotorMapping {
  double direction;
  double zero_offset;
};
```

映射函数：

```cpp
ArmJointAngles arm_inverse_kinematics_map_to_motor(...)
```

`arm_update(...)` 发给电机的就是 `solution.motor_angles`。

## 8. `Arm` 机构层接口

### 8.1 初始化

头文件：

- `include/mechanism/arm/arm.h`

初始化配置：

```cpp
struct ArmConfig {
  basic::device::MotorConfig joint1_motor;
  basic::device::MotorConfig joint2_motor;
  basic::device::MotorConfig joint3_motor;
  basic::device::MotorConfig joint4_motor;
  ArmIkConfig ik_config;
  vex::rotationUnits command_units;
  vex::velocityUnits move_speed_units;
  double move_speed;
};
```

其中四个 `joint*_motor` 就是你说的“`init` 接受电机 port”的实际落地方式。  
`MotorConfig` 本身包含：

- 端口
- 减速比
- 电机反向

初始化函数：

```cpp
Arm arm_init(const ArmConfig& config);
```

内部通过 `make_motor(...)` 创建四个 `vex::motor`。

### 8.2 更新

控制命令：

```cpp
struct ArmCommand {
  ArmPoint target;
  double q4_reference;
  bool hold_q4;
  bool enabled;
};
```

更新函数：

```cpp
void arm_update(Arm& mechanism, const ArmCommand& command);
```

当前 `update` 的行为是：

1. 保存 `last_command`
2. 若 `enabled == false`，直接 `arm_stop(...)`
3. 从四个电机当前位置读取当前电机角
4. 通过 `motor_mapping` 反算当前几何关节角
5. 求逆解
6. 若解不可用，则停止电机
7. 若解可用，则对四个电机执行：

```cpp
motor.spinToPosition(target_angle, angle_units, speed, speed_units, false);
```

也就是说当前实现是位置控制接口，不是直接给速度百分比。

### 8.3 停止

```cpp
void arm_stop(Arm& mechanism, vex::brakeType brake_type = vex::hold);
```

会对四个关节电机全部执行 `stopcontrol(...)`。

### 8.4 状态

```cpp
struct ArmState {
  ArmIkSolution last_solution;
  ArmCommand last_command;
};
```

读取接口：

```cpp
ArmState& arm_state(Arm& mechanism);
const ArmState& arm_state(const Arm& mechanism);
```

可用于调试：

- 上一拍输入目标
- 逆解是否成功
- 选中的是哪一支肘部解
- 实际下发的电机角是多少

## 9. `ArmIkSolution` 返回值说明

逆解结果结构：

```cpp
struct ArmIkSolution {
  ArmIkStatus status;
  ArmElbowBranch branch;
  bool reachable;
  bool used_previous_q1;
  ArmPoint target;
  ArmPoint solved_target;
  double rho;
  double distance;
  double cost;
  ArmJointAngles joint_angles;
  ArmJointAngles motor_angles;
};
```

主要字段含义：

- `status`
  - 当前求解状态
- `branch`
  - 选中了哪一组肘部分支
- `reachable`
  - 原始目标是否天然可达
- `used_previous_q1`
  - 是否因为接近基座轴线而沿用了上一拍 `q1`
- `joint_angles`
  - 几何关节角
- `motor_angles`
  - 映射后的电机目标角
- `cost`
  - 当前分支的连续性代价

`status` 可能值：

- `kSuccess`
  - 正常求解成功
- `kSingularBaseAxis`
  - 靠近 `rho = 0`，`q1` 采用上一拍
- `kUnreachable`
  - 目标不可达且未启用裁剪
- `kJointLimitViolation`
  - 某个候选解超过关节限位
- `kNoValidSolution`
  - 两组分支都无法作为最终解

注意：`kSingularBaseAxis` 不是失败状态。当前 `arm_update(...)` 遇到这个状态仍会执行电机位置命令。

## 10. 一个最小使用示例

```cpp
#include "mechanism/arm/arm.h"

basic::mechanism::arm::Arm arm = basic::mechanism::arm::arm_init({
    {vex::PORT1, vex::ratio36_1, false},
    {vex::PORT2, vex::ratio36_1, false},
    {vex::PORT3, vex::ratio36_1, false},
    {vex::PORT4, vex::ratio36_1, false},
    {
        200.0,
        180.0,
        1e-6,
        false,
        {{
            {-3.14, 3.14},
            {-1.57, 1.57},
            {-2.62, 2.62},
            {-3.14, 3.14},
        }},
        {{1.0, 1.0, 1.0, 0.0}},
        {{
            {1.0, 0.0},
            {1.0, 0.0},
            {1.0, 0.0},
            {1.0, 0.0},
        }},
    },
    vex::deg,
    vex::pct,
    30.0,
});

basic::mechanism::arm::arm_update(arm, {
    {150.0, 0.0, 100.0},
    0.0,
    true,
    true,
});
```

这段示例表达的是：

- 四个电机端口分别接在 `1/2/3/4`
- 连杆长度 `l1 = 200`, `l2e = 180`
- 目标点是 `(150, 0, 100)`
- `q4` 保持当前值
- 启用机构动作

## 11. 当前实现的边界与注意事项

这份代码已经能完成逆解并驱动电机，但仍有几个工程边界需要明确：

- 当前 `arm_update(...)` 默认按当前位置反推上一拍关节角
  - 这要求 `motor_mapping.direction` 不能为 `0`

- 当前电机控制使用 `spinToPosition(...)`
  - 适合位置目标控制
  - 如果后续要做轨迹规划或插补，需要再往上加一层

- 当前没有 controller 到 `ArmCommand` 的映射函数
  - 与 `indexed_intake_command_from_controller(...)` 这类接口不同
  - 如果后续要手柄控制机械臂，需要再补 `command_from_controller`

- 当前 `q4` 仅由命令层决定
  - 位置逆解不会自动决定末端滚转

- 当前默认目标点必须已经表达在肩关节坐标系下
  - 若目标来自其他坐标系，需要在调用 `arm_update(...)` 前先完成坐标变换

## 12. 总结

当前仓库中的机械臂模块已经按现有 `mechanism` 风格封装完成：

- `arm_init(config)`：根据 4 个电机配置初始化机构
- `arm_update(mechanism, command)`：接收目标点命令，内部完成逆解并下发电机角
- `arm_stop(...)`：停止四个关节电机
- `arm_state(...)`：读取上一拍命令和逆解结果

其逆解核心逻辑是：

1. `q1 = atan2(y, x)`，接近基座轴线时保持上一拍
2. 在 `(rho, z)` 平面内用二维二连杆公式求 `q2/q3`
3. 同时计算两组肘部分支，并用限位与连续性代价选解
4. `q4` 不由位置决定，而由命令层保持或指定
5. 最终按 `motor = direction * joint + zero_offset` 映射后发给四个电机

如果后续你还要把这份说明再贴近你们项目文风，我可以继续把这份 `README.md` 改成和仓库里其他说明文档一致的章节和命名习惯。
