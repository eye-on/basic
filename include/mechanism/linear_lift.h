#ifndef BASIC_INCLUDE_MECHANISM_LINEAR_LIFT_H_
#define BASIC_INCLUDE_MECHANISM_LINEAR_LIFT_H_

#include "device_config.h"
#include "hardware/shared/state_types.h"

namespace basic::mechanism {

struct LinearLiftMotorSlot {
  basic::device::MotorConfig motor;
  double position_min{0.0};  // 行程下限（启停位置参数）
  double position_max{0.0};  // 行程上限
};

struct LinearLiftConfig {
  LinearLiftMotorSlot lift_motor1;
  LinearLiftMotorSlot lift_motor2;
  double closed_loop_speed_pct{60.0};      // 闭环运动速度
  double open_loop_speed_pct{60.0};        // 手动启停固定速度
  vex::rotationUnits position_units{vex::deg};
  double sync_max_deviation{50.0};         // 两电机同步最大偏差
};

struct LinearLiftCommand {
  bool toggle_up{false};    // press_up → 切换上升
  bool toggle_down{false};  // press_down → 切换下降
  bool enabled{true};
};

struct LinearLiftState {
  double target_position{0.0};       // 用户请求的原始目标（仅记录用）
  bool open_loop_up{false};
  bool open_loop_down{false};
  double motor1_position{0.0};
  double motor2_position{0.0};
  bool at_target{true};
  bool synced{true};
};

class LinearLift {
 public:
  explicit LinearLift(const LinearLiftConfig& config);

  vex::motor& lift_motor1();
  vex::motor& lift_motor2();

  const vex::motor& lift_motor1() const;
  const vex::motor& lift_motor2() const;

  LinearLiftConfig& config();
  const LinearLiftConfig& config() const;

  LinearLiftState& state();
  const LinearLiftState& state() const;

 private:
  LinearLiftConfig config_;
  vex::motor lift_motor1_;
  vex::motor lift_motor2_;
  LinearLiftState state_;
};

LinearLift linear_lift_init(const LinearLiftConfig& config);

LinearLiftCommand linear_lift_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input);

void linear_lift_update(LinearLift& mechanism, const LinearLiftCommand& command);

void linear_lift_set_position(LinearLift& mechanism, double position);

void linear_lift_stop(LinearLift& mechanism, vex::brakeType brake_type = vex::hold);

LinearLiftState& linear_lift_state(LinearLift& mechanism);
const LinearLiftState& linear_lift_state(const LinearLift& mechanism);

}  // namespace basic::mechanism

#endif
