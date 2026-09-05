#ifndef BASIC_SRC_HARDWARE_BED_ROBOT_HARDWARE_H_
#define BASIC_SRC_HARDWARE_BED_ROBOT_HARDWARE_H_

#include "chassis/bed_chassis.h"
#include "mechanism/arm_2dof.h"
#include "mechanism/intake.h"
#include "mechanism/pneumatic_gripper.h"
#include "vex.h"

namespace basic::hardware::bed {

inline constexpr int kRefreshTime = 10;

struct RobotHardware {
  vex::brain brain;
  vex::controller controller{vex::controllerType::primary};
  basic::chassis::BedChassis bed_chassis;
  basic::mechanism::Intake intake;
  basic::mechanism::PneumaticGripper pneumatic_gripper;
  basic::mechanism::Arm2Dof arm_2dof;

  // 实际接线（2026-09-05）：右后 BR=1/2、右前 FR=5/6、左后 BL=7/8、左前 FL=9/11。
  // 每轮 2 电机驱动同一个轮：同轴同向安装则两个电机 reversed 相同；
  // 对角轮方向互为镜像（X 底盘惯例：FL/BL 与 FR/BR 相反）——TODO: 实测后核对。
  // intake / arm_2dof 电机未接线：统一占位 PORT20，接线后替换。
  RobotHardware()
      : bed_chassis(basic::chassis::bed_chassis_init({
            {{ // 左前 FL：2 电机（端口 9, 11）
                {vex::PORT9, vex::ratio6_1, false},   // TODO: 方向核对
                {vex::PORT11, vex::ratio6_1, false},
            }},
            {{ // 右前 FR：2 电机（端口 5, 6）
                {vex::PORT5, vex::ratio6_1, true},    // TODO: 方向核对
                {vex::PORT6, vex::ratio6_1, true},
            }},
            {{ // 左后 BL：2 电机（端口 7, 8）
                {vex::PORT7, vex::ratio6_1, false},   // TODO: 方向核对
                {vex::PORT8, vex::ratio6_1, false},
            }},
            {{ // 右后 BR：2 电机（端口 1, 2）
                {vex::PORT1, vex::ratio6_1, true},    // TODO: 方向核对
                {vex::PORT2, vex::ratio6_1, true},
            }},
            10,   // deadzone
            1.0,  // forward_sensitivity
            1.0,  // strafe_sensitivity
            1.0,  // turn_sensitivity
        })),
        // 吸入模块：2 电机开环定速；未接线占位 20，TODO: 接线后替换端口/正反/转速
        intake(basic::mechanism::intake_init({
            {vex::PORT20, vex::ratio6_1, false},  // TODO: 电机 A 端口/正反
            {vex::PORT20, vex::ratio6_1, false},  // TODO: 电机 B 端口/正反
            100.0,                                // intake_speed_pct（开环）
        })),
        // 气缸夹爪：抓握/松开两态，R1 边沿切换；TODO(bed): 接线后替换 ADI 口
        // 与电磁阀极性（inverted：抓握对应高电平=false）
        pneumatic_gripper(basic::mechanism::pneumatic_gripper_init({
            {brain.ThreeWirePort.F},  // TODO: 气缸电磁阀三线口
            false,                    // inverted
        })),
        // 二自由度机械臂：开环速度（暂时）+ 闭环位置（待启用）；
        // 键位：上/下=关节1，X/B=关节2；未接线占位 20，TODO: 接线后替换端口/正反
        arm_2dof(basic::mechanism::arm_2dof_init({
            {vex::PORT20, vex::ratio36_1, false},  // TODO: 关节 1 电机（齿比按实际）
            {vex::PORT20, vex::ratio36_1, false},  // TODO: 关节 2 电机（齿比按实际）
            basic::mechanism::Arm2DofMode::kOpenLoopVelocity,  // 暂时开环速度
            40.0,   // velocity_speed_pct
            vex::deg,
            30.0,   // position_speed_pct
        })) {}

  void show_ready() {
    controller.Screen.setCursor(5, 1);
    controller.Screen.print("      bed ready!");
  }
};

}  // namespace basic::hardware::bed

#endif  // BASIC_SRC_HARDWARE_BED_ROBOT_HARDWARE_H_
