#ifndef BASIC_INCLUDE_MECHANISM_PNEUMATIC_GRIPPER_H_
#define BASIC_INCLUDE_MECHANISM_PNEUMATIC_GRIPPER_H_

#include "device_config.h"
#include "hardware/shared/state_types.h"

namespace basic::mechanism {

/// 夹爪状态：松开 / 抓握
enum class PneumaticGripperState {
  kRelease,
  kGrasp,
};

struct PneumaticGripperConfig {
  basic::device::DigitalOutConfig output;  // 气缸电磁阀（三线数字口）
  bool inverted{false};  // 电磁阀电平极性：true = 抓握对应输出低电平（视阀/气缸接线）
};

struct PneumaticGripperCommand {
  bool toggle{false};  // press_r1 → 边沿触发翻转 抓握/松开
};

struct PneumaticGripperState {
  PneumaticGripperState mode{PneumaticGripperState::kRelease};
};

class PneumaticGripper {
 public:
  explicit PneumaticGripper(const PneumaticGripperConfig& config);

  vex::digital_out& output();
  const vex::digital_out& output() const;

  PneumaticGripperConfig& config();
  const PneumaticGripperConfig& config() const;

  PneumaticGripperState& state();
  const PneumaticGripperState& state() const;

 private:
  PneumaticGripperConfig config_;
  vex::digital_out output_;
  PneumaticGripperState state_;
};

/// 初始化气缸夹爪
PneumaticGripper pneumatic_gripper_init(const PneumaticGripperConfig& config);

/// 从遥控器输入生成控制指令（键位：R1 = 边沿切换 抓握/松开）
PneumaticGripperCommand pneumatic_gripper_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input);

/// 主更新：处理 toggle 边沿并下发电磁阀电平
void pneumatic_gripper_update(
    PneumaticGripper& mechanism,
    const PneumaticGripperCommand& command);

/// 直接设置状态（autonomous 用）
void pneumatic_gripper_set_state(
    PneumaticGripper& mechanism,
    PneumaticGripperState state);

/// 停止并复位为松开（切断电磁阀）
void pneumatic_gripper_stop(PneumaticGripper& mechanism);

/// 状态访问（mutable / const）
PneumaticGripperState& pneumatic_gripper_state(PneumaticGripper& mechanism);
const PneumaticGripperState& pneumatic_gripper_state(
    const PneumaticGripper& mechanism);

}  // namespace basic::mechanism

#endif  // BASIC_INCLUDE_MECHANISM_PNEUMATIC_GRIPPER_H_
