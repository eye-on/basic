#ifndef BASIC_INCLUDE_MECHANISM_DUAL_PNEUMATIC_H_
#define BASIC_INCLUDE_MECHANISM_DUAL_PNEUMATIC_H_

#include "device_config.h"
#include "hardware/shared/state_types.h"

namespace basic::mechanism {

/// 框住机构模式：松开（气缸缩回）/ 框住（气缸伸出）
enum class DualPneumaticMode {
  kReleased,
  kEngaged,
};

/// 双气缸框住机构配置：两个三线数字口各驱动一个电磁阀
struct DualPneumaticConfig {
  basic::device::DigitalOutConfig cylinder_a;  // 气缸 A 电磁阀
  basic::device::DigitalOutConfig cylinder_b;  // 气缸 B 电磁阀
  bool engaged_signal{true};  // 框住时输出电平（按电磁阀/气管接法调整）
  DualPneumaticMode initial_mode{DualPneumaticMode::kReleased};  // 上电初始状态：默认展开(松开)
};

/// 框住机构指令：本周期切换一次 框住/松开（边沿触发）
struct DualPneumaticCommand {
  bool toggle{false};  // press_b → 边沿触发翻转 框住/松开
};

/// 框住机构状态
struct DualPneumaticState {
  DualPneumaticMode mode{DualPneumaticMode::kReleased};
  bool output_signal{false};  // 当前电磁阀电平
  int toggle_count{0};        // 已完成的切换次数（调试用）
};

class DualPneumatic {
 public:
  explicit DualPneumatic(const DualPneumaticConfig& config);

  vex::digital_out& cylinder_a();
  vex::digital_out& cylinder_b();
  const vex::digital_out& cylinder_a() const;
  const vex::digital_out& cylinder_b() const;

  DualPneumaticConfig& config();
  const DualPneumaticConfig& config() const;

  DualPneumaticState& state();
  const DualPneumaticState& state() const;

 private:
  vex::digital_out cylinder_a_;
  vex::digital_out cylinder_b_;
  DualPneumaticConfig config_;
  DualPneumaticState state_;
};

/// 初始化框住机构
DualPneumatic dual_pneumatic_init(const DualPneumaticConfig& config);

/// 从遥控器输入生成控制指令（键位：B = 边沿切换 框住/松开）
DualPneumaticCommand dual_pneumatic_command_from_controller(
    const basic::hardware::shared::ControllerInputState& input);

/// 主更新：处理 toggle 边沿并下发两个电磁阀电平
void dual_pneumatic_update(
    DualPneumatic& mechanism,
    const DualPneumaticCommand& command);

/// 直接设置模式（autonomous 用）
void dual_pneumatic_set_mode(
    DualPneumatic& mechanism,
    DualPneumaticMode mode);

/// 停止：复位为松开并切断两个电磁阀
void dual_pneumatic_stop(DualPneumatic& mechanism);

/// 状态访问（mutable / const）
DualPneumaticState& dual_pneumatic_state(DualPneumatic& mechanism);
const DualPneumaticState& dual_pneumatic_state(
    const DualPneumatic& mechanism);

}  // namespace basic::mechanism

#endif  // BASIC_INCLUDE_MECHANISM_DUAL_PNEUMATIC_H_
