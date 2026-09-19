#ifndef BASIC_SRC_INPUT_CONTROLLER_H_
#define BASIC_SRC_INPUT_CONTROLLER_H_

#include "hardware/shared/state_types.h"

namespace basic::input {

/// 按键帧保护默认帧数：按下与松开都需连续 N 帧确认才生效
/// （10ms 循环下 3 帧 = 30ms，滤掉抖动/接触噪声；<=1 表示不保护）
inline constexpr int kButtonDebounceFrames = 3;

/// 读取手柄并更新共享状态（摇杆 + 按键电平 + press_* 按下沿）
/// - 12 个按键全部做帧保护：按下、松开都需连续 button_debounce_frames 帧一致
/// - press_* 边沿由保护后的电平产生，因此开关类操作同样受保护
void controller_update(
    vex::brain& brain,
    vex::controller& controller,
    basic::hardware::shared::ControllerInputState& state,
    int button_debounce_frames = kButtonDebounceFrames);

}  // namespace basic::input

#endif
