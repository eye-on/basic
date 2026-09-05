#ifndef BASIC_SRC_HARDWARE_BED_ROBOT_HARDWARE_H_
#define BASIC_SRC_HARDWARE_BED_ROBOT_HARDWARE_H_

#include "chassis/bed_chassis.h"
#include "vex.h"

namespace basic::hardware::bed {

inline constexpr int kRefreshTime = 10;

struct RobotHardware {
  vex::brain brain;
  vex::controller controller{vex::controllerType::primary};
  basic::chassis::BedChassis bed_chassis;

  // TODO(bed): 实际接线后替换下列占位端口与正反方向。
  // 每轮 2 电机驱动同一个轮：同轴同向安装则两个电机 reversed 相同；
  // 对角轮方向互为镜像（X 底盘惯例：FL/BL 与 FR/BR 相反）。
  // 装配参考 looklook（x_chassis 版）：FL 在前左、FR 前右、BL 后左、BR 后右。
  RobotHardware()
      : bed_chassis(basic::chassis::bed_chassis_init({
            {{ // 左前 FL：2 电机
                {vex::PORT1, vex::ratio6_1, false},  // TODO: 端口/正反
                {vex::PORT2, vex::ratio6_1, false},
            }},
            {{ // 右前 FR：2 电机
                {vex::PORT3, vex::ratio6_1, true},   // TODO: 端口/正反
                {vex::PORT4, vex::ratio6_1, true},
            }},
            {{ // 左后 BL：2 电机
                {vex::PORT5, vex::ratio6_1, false},  // TODO: 端口/正反
                {vex::PORT6, vex::ratio6_1, false},
            }},
            {{ // 右后 BR：2 电机
                {vex::PORT7, vex::ratio6_1, true},   // TODO: 端口/正反
                {vex::PORT8, vex::ratio6_1, true},
            }},
            10,   // deadzone
            1.0,  // forward_sensitivity
            1.0,  // strafe_sensitivity
            1.0,  // turn_sensitivity
        })) {}

  void show_ready() {
    controller.Screen.setCursor(5, 1);
    controller.Screen.print("      bed ready!");
  }
};

}  // namespace basic::hardware::bed

#endif  // BASIC_SRC_HARDWARE_BED_ROBOT_HARDWARE_H_
