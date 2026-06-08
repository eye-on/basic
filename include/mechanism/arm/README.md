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
- `q2`：肩部极角，按大臂相对 `+z` 方向定义
- `q3`：肘部相对角，`q3 = 0` 表示大臂小臂共线伸直
- `q4`：小臂绕自身轴线滚转

并且满足关键约束：

- 大臂和小臂始终在同一个平面内运动
- 这个运动平面始终垂直于肩部安装平面
- 肩关节实际包含两个自由度
  - `q1` 为绕 `z` 轴的底座旋转
- `q2` 为绕一条位于 `xoy` 平面内的轴做俯仰旋转
- `q2` 不是连续全周关节，而是只能在一个半圆范围内运动
  - 这里的半圆不是“绕 `q2` 转轴的 `[-pi/2, pi/2]`”
  - 而是以 `z` 轴为参考的半圆
  - 落实到几何关节角定义时，推荐直接把 `q2` 约束为 `[0, pi]`
- `q3` 也存在机械限位；当前实物范围尚未最终确定，但已知其可动区大于 `270°` 且小于 `360°`

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
  int coordinate_sign;
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
- `coordinate_sign`：坐标镜像符号，取 `+1` 为默认装配，取 `-1` 为镜像装配
- `joint_limits`：几何关节角限位
- `continuity_weights`：分支连续性代价权重
- `motor_mapping`：几何角到电机角的方向和零位映射

### 3.1 镜像装配

同一套连杆和关节定义可以通过 `coordinate_sign` 复用到镜像装配：

- `coordinate_sign = +1`
  - 按默认坐标求解
- `coordinate_sign = -1`
  - 先把输入目标点 `(x, y, z)` 整体取反后再做逆解

这样可以在不改主体公式的前提下，为左右镜像安装的机械臂复用同一套求解流程。

### 3.2 关节限位表达

当前代码里的关节限位使用：

```cpp
struct ArmJointLimit {
  double min;
  double max;
};
```

角度单位与逆解内部一致，均为弧度。

- 若 `min <= max`
  - 有效区间直接解释为闭区间 `[min, max]`
  - 适合表达大臂这种不跨角度断点的半圆限位
  - 当前推荐将 `q2` 直接配置为 `[0, pi]`
- 若 `min > max`
  - 有效区间解释为“跨过 `-pi/pi` 断点的环形区间”
  - 也就是 `angle >= min` 或 `angle <= max`
  - 适合表达小臂这种可动区大于 `270°`、但又不是整圈 `360°` 的限位

因此当前推荐约定是：

- `joint_limits[0]`
  - `q1`，肩部绕 `z` 轴旋转
- `joint_limits[1]`
  - `q2`，大臂俯仰限位
- `joint_limits[2]`
  - `q3`，小臂相对角限位
  - 当前默认按机构实物限位设置为：
    - 正向屈臂极限：小臂距离大臂 `29°`
    - 反向极限：小臂距离大臂 `71°`
  - 由于实际可动区不是简单的 `[-71°, +29°]` 线性区间，而是跨过角度断点的环形区间
  - 因此对应几何限位写成：
    - `q3_min = +29°`
    - `q3_max = -71°`
  - 这里 `min > max` 的含义是：
    - 允许区间为 `[29°, 180°] U [-180°, -71°]`
    - 等价于从 `29°` 连续转到 `289°`
    - 总运动范围为 `260°`
- `joint_limits[3]`
  - `q4`，末端滚转限位

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

先定义：

```text
alpha = atan2(z, rho)
beta  = atan2(l2e * s3, l1 + l2e * c3)
```

其中：

- `alpha` 是目标方向相对 `xoy` 平面的仰角
- `q2` 按相对 `+z` 方向的极角定义

因此当前实现使用：

```text
q2 = pi / 2 - alpha + beta
```

在这一定义下：

- 大臂朝 `+z` 时，`q2` 接近 `0`
- 大臂水平时，`q2` 接近 `pi / 2`
- 大臂朝 `-z` 时，`q2` 接近 `pi`

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

这里的限位检查按“环形角度区间”进行，不是简单线性比较。
因此：

- 大臂 `q2` 可以直接配置成普通半圆区间
- 小臂 `q3` 如果实际可动区跨过 `-pi/pi` 断点，可以用 `min > max` 来表达

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

## 6. 位置点可达性判定

工作空间判定条件为：

```text
abs(l1 - l2e) <= d <= l1 + l2e
```

实现中先算目标点到肩关节原点的距离：

```text
target_distance = sqrt(x^2 + y^2 + z^2)
```

当前实现把“目标点能否到达”拆成两层：

1. 距离工作空间是否可达
2. 是否存在满足关节限位的姿态解

### 6.1 距离工作空间

若超出工作空间：

- `clamp_unreachable_target == false`
  - 返回 `ArmIkStatus::kUnreachable`

- `clamp_unreachable_target == true`
  - 调用 `clamp_target_to_workspace(...)`
  - 将目标点沿原点到目标点方向裁剪到最近的工作空间边界
  - 再基于裁剪后的点求逆解

### 6.2 姿态与关节限位

即使距离上可达，目标点仍可能因为关节限位而不可达。
当前实现会同时计算两组肘部分支，并检查：

- `q2` 是否落在 `[0, pi]`
- `q3` 是否落在肘部机械限位区间
- 其他关节是否满足各自的配置限位

如果距离上可达，但两组分支都不满足关节限位，则该点仍然视为不可达。

返回结果里：

- `target` 是原始输入目标
- `solved_target` 是真正参与逆解的点
- `reachable` 表示原始输入目标点是否精确可达
- `within_distance_workspace` 表示目标点距离是否落在二连杆工作空间内
- `within_joint_limits` 表示最终选中的解是否满足关节限位
- `clamped_target` 表示是否先被裁剪到距离工作空间边界后再求解

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

当前电机映射配置为：

```cpp
struct ArmMotorMapping {
  double direction;
  double units_per_radian;
  double zero_offset;
  double gearbox_ratio;
};
```

再按下面公式映射到电机角：

```text
motor = direction * units_per_radian * joint + zero_offset
```

对应配置：

```cpp
struct ArmMotorMapping {
  double direction;
  double units_per_radian;
  double zero_offset;
};
```

映射函数：

```cpp
ArmJointAngles arm_inverse_kinematics_map_to_motor(...)
```

`arm_update(...)` 发给电机的就是 `solution.motor_angles`。

这套映射可以直接支持电机单圈原始编码标定：

- 若 `command_units = vex::deg`
  - 默认可令 `units_per_radian = 180 / pi`
- 若 `command_units = vex::raw`
  - 可以把 `zero_offset` 设为电机在自然初始姿态下的 raw 读数
  - `units_per_radian` 设为该关节每转 1 弧度对应多少 raw 编码值

这样就能在单圈范围内，把电机掉电后仍保留的绝对原始角作为你自己的关节坐标参考。

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
  bool within_distance_workspace;
  bool within_joint_limits;
  bool clamped_target;
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
  - 原始输入目标点是否精确可达
  - 若启用了目标裁剪，可能出现 `status` 允许继续动作，但 `reachable == false`
- `within_distance_workspace`
  - 目标点距离是否落在二连杆半径工作空间内
- `within_joint_limits`
  - 最终解是否满足全部关节限位
- `clamped_target`
  - 是否因为距离不可达而先被裁剪到边界
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
        1,
        {{
            {-3.14, 3.14},
            {0.0, 3.14},
            {0.51, -1.24},
            {-3.14, 3.14},
        }},
        {{1.0, 1.0, 1.0, 0.0}},
        {{
            {1.0},
            {1.0},
            {1.0},
            {1.0},
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
- `coordinate_sign = 1`，表示默认装配方向
- `q2` 示例限位为半圆区间 `[0, pi]`
- `q3` 示例限位用跨断点区间 `{0.51, -1.24}` 表达 `29° -> 289°` 的 `260°` 可动范围
- `motor_mapping` 示例只显式给 `direction = 1.0`
  - `units_per_radian` 与 `zero_offset` 使用结构默认值
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

- 当前 `joint_limits` 约束的是几何关节角，不是电机编码器角
  - 如果电机安装方向、减速关系或零位有偏置，需要先通过 `motor_mapping` 和关节定义对齐后再配置限位

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
5. 最终按 `motor = direction * units_per_radian * joint + zero_offset` 映射后发给四个电机

调试时可按下面理解返回结果：

- `status` 表示这次求解/执行路径属于哪种情况
- `reachable` 只表示原始输入目标点是否精确可达

如果后续你还要把这份说明再贴近你们项目文风，我可以继续把这份 `README.md` 改成和仓库里其他说明文档一致的章节和命名习惯。
