#include "hardware/robot_selector.h"

#include "chassis/bed_chassis.h"
#include "hardware/bed/autonomous.h"
#include "hardware/bed/robot_hardware.h"
#include "hardware/bed/robot_state.h"
#include "input/controller.h"
#include "mechanism/arm_2dof.h"
#include "mechanism/dual_pneumatic.h"
#include "mechanism/intake.h"
#include "mechanism/linear_lift.h"
#include "mechanism/pneumatic_gripper.h"

#include <cmath>
#include <cstdio>

namespace basic::hardware::bed {

namespace {

class BedRobot;
BedRobot& current_bed();

class BedRobot final : public basic::app::Robot {
 public:
  void initialize() override {
    hardware_.show_ready();
    hardware_.calibrate_inertial_sensor();  // IMU 标定 + 归零（未插则跳过）
    calibrate_stick_center();  // 上电标定摇杆零位，消除静差
  }

  /// 摇杆零位标定（静差补偿）：采样 ~300ms 取均值作为零位偏置，
  /// 之后所有输入先减偏置再进死区，保证左右行程对称、松手不漂。
  /// 注意：标定期间（上电约 0.3s）请勿触碰摇杆；
  ///       偏置超过 ±15% 视为异常（摇杆未回中/未连接）→ 不使用偏置。
  void calibrate_stick_center() {
    constexpr int kSamples = 30;       // 30 × 10ms ≈ 300ms
    constexpr int kMaxOffsetPct = 15;  // 合理零位偏差上限
    constexpr int kSampleDelayMs = 10;

    int sum_forward = 0;
    int sum_strafe = 0;
    int sum_turn = 0;
    for (int i = 0; i < kSamples; ++i) {
      // bed 轴映射：轴2=前后，轴1=平移，轴4=旋转
      sum_forward += hardware_.controller.Axis2.position(vex::percentUnits::pct);
      sum_strafe += hardware_.controller.Axis1.position(vex::percentUnits::pct);
      sum_turn += hardware_.controller.Axis4.position(vex::percentUnits::pct);
      vex::this_thread::sleep_for(kSampleDelayMs);
    }

    int off_forward = sum_forward / kSamples;
    int off_strafe = sum_strafe / kSamples;
    int off_turn = sum_turn / kSamples;

    if (off_forward > kMaxOffsetPct || off_forward < -kMaxOffsetPct) {
      off_forward = 0;
    }
    if (off_strafe > kMaxOffsetPct || off_strafe < -kMaxOffsetPct) {
      off_strafe = 0;
    }
    if (off_turn > kMaxOffsetPct || off_turn < -kMaxOffsetPct) {
      off_turn = 0;
    }

    hardware_.bed_chassis.set_axis_offsets(off_forward, off_strafe, off_turn);
  }

  void bind_background_tasks() override {
    // 临时：手动控制脱离竞赛线程——程序启动即可操控
    // 恢复时：删除本线程，并在 bind_competition 中恢复 drivercontrol 绑定
    vex::thread manual_thread(start_manual_control_thread);
    // 调试打印独立线程（100Hz）：printf 可能因串口阻塞，放这里以免拖慢控制循环
    vex::thread debug_thread(start_debug_print_thread);
  }

  void bind_competition(vex::competition& competition) override {
    competition_ = &competition;
    competition.autonomous(start_autonomous_entry);
    // 临时：手动控制由常驻线程接管，避免双重控制（恢复时取消注释）
    // competition.drivercontrol(start_driver_control_entry);
  }

 private:
  static void start_driver_control_entry() {
    current_bed().run_driver_control_loop();
  }

  // 临时：常驻手动控制线程入口（不依赖竞赛状态）
  static void start_manual_control_thread() {
    current_bed().run_manual_control_loop();
  }

  // 调试打印线程入口（100Hz CSV）
  static void start_debug_print_thread() {
    current_bed().run_debug_print_loop();
  }

  static void start_autonomous_entry() {
    current_bed().run_autonomous_routine();
  }

  /// 单次手动控制迭代：读手柄 → 底盘 → 各机构（联锁在内）
  void control_step() {
    basic::input::controller_update(hardware_.brain, hardware_.controller,
                                    state_.controller,
                                    basic::input::kButtonDebounceFrames,
                                    kAxisSnapPct);  // 松手 ±1 抖动吸附为 0

    // 调试键位：L1 开关打印；L2 循环切换固定速度测试模式
    //   （0 = 手动跟摇杆 → 30 → 50 → 100 pct → 0 …，见 kTestSpeedPct）
    //   R1 = 航向目标重锚定（当前车头方向设为锁定目标）；R2 = 航向闭环开关
    if (state_.controller.press_l1) {
      debug_print_enabled_ = !debug_print_enabled_;
      debug_toggle_seq_ += 1;  // 让打印线程补一行 "# print ON/OFF"
    }
    if (state_.controller.press_l2) {
      test_speed_index_ = (test_speed_index_ + 1) % kTestSpeedCount;
    }
    if (state_.controller.press_r1) {
      basic::chassis::yaw_hold_reset(hardware_.yaw_hold,
                                     hardware_.imu.rotation(vex::deg));
    }
    if (state_.controller.press_r2) {
      hardware_.yaw_hold.config.enabled = !hardware_.yaw_hold.config.enabled;
    }

    basic::chassis::BedChassisCommand chassis_command =
        basic::chassis::bed_chassis_command_from_controller(
            state_.controller,
            basic::chassis::bed_chassis_state(hardware_.bed_chassis).stop_brake_type);

    // IMU 航向保持：**每周期实时闭环**（静止/平移/原地旋转都在修正）
    //   只有左摇杆旋转轴（axis4）能改变目标 yaw；被推歪会被闭环拉回目标
    //   R1 重锚定 / R2 开关；IMU 未插或在标定时自动不介入
    const basic::hardware::shared::ControllerInputState& input = state_.controller;
    auto& chassis = hardware_.bed_chassis;
    basic::chassis::YawHoldInput yaw_input;
    // 减零位偏置后再吸附一次：偏置本身可能是 ±1，否则残留抖动会被积分进目标 yaw
    const int turn_excess = input.axis4 - chassis.axis_offset_turn();
    yaw_input.turn_input_pct =
        (std::abs(turn_excess) <= kAxisSnapPct) ? 0 : turn_excess;
    yaw_input.yaw_deg = hardware_.imu.rotation(vex::deg);
    yaw_input.yaw_rate_dps = hardware_.imu.gyroRate(vex::zaxis, vex::dps);
    yaw_input.imu_ready = hardware_.imu.installed() && !hardware_.imu.isCalibrating();
    const double yaw_correction =
        basic::chassis::yaw_hold_update(hardware_.yaw_hold, yaw_input);
    state_.yaw_hold = hardware_.yaw_hold.state;

    const double fixed_pct = kTestSpeedPct[test_speed_index_];
    if (fixed_pct <= 0.0 || kYawHoldInTestMode) {
      // 手动模式与测试模式都叠加航向修正（关闭时只在 R2/改 kYawHoldInTestMode）
      chassis_command.turn_correction_pct = yaw_correction;
    }

    if (fixed_pct > 0.0) {
      // 固定速度测试模式：忽略摇杆，四轮直接给同一 pct（前进方向，命令值完全确定）
      //   turn_correction_pct 由 x_drive_update 在直通模式下同样叠加
      chassis_command.use_direct_output = true;
      chassis_command.direct_fl_pct = fixed_pct;
      chassis_command.direct_fr_pct = fixed_pct;
      chassis_command.direct_bl_pct = fixed_pct;
      chassis_command.direct_br_pct = fixed_pct;
    }
    basic::chassis::bed_chassis_update(hardware_.bed_chassis, chassis_command);

    basic::mechanism::intake_update(
        hardware_.intake,
        basic::mechanism::intake_command_from_controller(state_.controller));
    basic::mechanism::pneumatic_gripper_update(
        hardware_.pneumatic_gripper,
        basic::mechanism::pneumatic_gripper_command_from_controller(
            state_.controller));
    basic::mechanism::dual_pneumatic_update(
        hardware_.dual_pneumatic,
        basic::mechanism::dual_pneumatic_command_from_controller(
            state_.controller));

    // 联锁：框住机构处于展开(松开)态时，禁止机械臂展开
    basic::mechanism::arm_2dof_set_extend_allowed(
        hardware_.arm_2dof,
        basic::mechanism::dual_pneumatic_state(hardware_.dual_pneumatic).mode ==
            basic::mechanism::DualPneumaticMode::kEngaged);

    basic::mechanism::arm_2dof_update(
        hardware_.arm_2dof,
        basic::mechanism::arm_2dof_command_from_controller(state_.controller));

    // 抬升机构：上 = 升 / 下 = 降（模块默认键位映射）
    basic::mechanism::linear_lift_update(
        hardware_.lift,
        basic::mechanism::linear_lift_command_from_controller(state_.controller));

    // 注意：打印在独立线程（run_debug_print_loop），这里不做任何 printf
  }

  /// 调试打印线程：100Hz CSV（L1 开关；关闭时只空转）
  /// 为什么单独线程：printf 在串口缓冲满时会阻塞，放在控制线程会拖慢/卡住底盘控制
  /// 行类型（每行首字段为类型）：
  ///   D 行（100Hz）：t_ms,mode, 同轮两个电机的实测 rpm ×4 组, 每组相位 deg, 饱和组数
  ///   C 行（10Hz） ：t_ms, 同轮两个电机的电流（占最大电流的 %）×4 组
  /// 用途：定位"单圈某个角度摩擦偏大"造成的掉速
  ///   1) 对齐 pos 看掉速峰是否按固定周期重复：每轮一圈 → 轮侧问题；
  ///      每电机一圈（= 外置齿比倍）→ 齿轮箱/电机侧问题
  ///   2) 同组两个电机的 rpm/电流 对比：明显不一致 → 其中一个电机在拖/打滑/齿轮相位问题
  ///   3) 电流随相位的变化 = 卡点需要的扭矩，比 rpm 更早暴露问题
  ///   4) sat > 0 = 该轮速度目标已顶到极限（没有余量）
  /// 说明：全部整数打印（避免工程链接未启用 -u_printf_float 导致 %f 打不出来）；
  ///       pos 为本次打印开始后的相对电机角度（deg，单调），便于直接对齐相位
  void run_debug_print_loop() {
    while (true) {
      update_debug_print();
      vex::this_thread::sleep_for(kDebugPrintPeriodMs);
    }
  }

  void update_debug_print() {
    // L1 切换时补一行事件（ON/OFF 都要打，所以放在 enabled 判断之前）
    if (debug_toggle_seq_ != debug_seen_toggle_seq_) {
      debug_seen_toggle_seq_ = debug_toggle_seq_;
      if (debug_print_enabled_) {
        printf("# print ON\n");
      } else {
        printf("# print OFF\n");
      }
    }

    // R2 切换航向环时补一行事件（axis-only 模式下也能从日志看到）
    if (hardware_.yaw_hold.config.enabled != debug_yaw_seen_enabled_) {
      debug_yaw_seen_enabled_ = hardware_.yaw_hold.config.enabled;
      if (hardware_.yaw_hold.config.enabled) {
        printf("# yaw hold -> ON\n");
      } else {
        printf("# yaw hold -> OFF\n");
      }
    }

    if (!debug_print_enabled_) {
      debug_session_active_ = false;
      return;
    }

    auto& chassis = hardware_.bed_chassis;
    const auto& s = basic::chassis::bed_chassis_state(chassis);
    auto& fl = chassis.fl_motors();
    auto& fr = chassis.fr_motors();
    auto& bl = chassis.bl_motors();
    auto& br = chassis.br_motors();
    auto round_int = [](double value) {
      return static_cast<int>(value + (value >= 0.0 ? 0.5 : -0.5));
    };
    auto rpm_of = [&round_int](vex::motor& motor) {
      return round_int(motor.velocity(vex::rpm));
    };
    auto cur_of = [&round_int](vex::motor& motor) {
      return round_int(basic::control::get_current(motor, vex::pct));
    };
    auto saturated = [](double pct) { return (std::abs(pct) >= 99.0) ? 1 : 0; };
    const int sat = saturated(s.fl_pct) + saturated(s.fr_pct) +
                    saturated(s.bl_pct) + saturated(s.br_pct);
    const int now_ms = static_cast<int>(vex::timer::system());

    const double pos_fl =
        basic::chassis::detail::group_position_deg(chassis.fl_motors());
    const double pos_fr =
        basic::chassis::detail::group_position_deg(chassis.fr_motors());
    const double pos_bl =
        basic::chassis::detail::group_position_deg(chassis.bl_motors());
    const double pos_br =
        basic::chassis::detail::group_position_deg(chassis.br_motors());

    // 每次开启打印视为一段新会话：时间与角度都从 0 起算，并打印表头
    if (!debug_session_active_) {
      debug_session_active_ = true;
      debug_t0_ms_ = now_ms;
      debug_pos0_fl_ = pos_fl;
      debug_pos0_fr_ = pos_fr;
      debug_pos0_bl_ = pos_bl;
      debug_pos0_br_ = pos_br;
      debug_last_mode_ = test_speed_index_;
      debug_row_counter_ = 0;
      if (kDebugAxisOnly) {
        printf("# profile: axis-only —— 只打印手柄摇杆行（A，100Hz）\n");
        printf("# A,t_ms,axis1,axis2,axis3,axis4,fl_x10,fr_x10,bl_x10,br_x10,corr_x10\n");
        printf("# axis1=右X(平移) axis2=右Y(前后) axis3=左Y(未用) axis4=左X(旋转)；fl/fr/bl/br=下发 pct×10\n");
      } else {
        printf("# profile: full —— D(100Hz)+A(25Hz)+C/Y/K(10Hz)\n");
        printf("# D,t_ms,mode,rpm_fla,rpm_flb,rpm_fra,rpm_frb,rpm_bla,rpm_blb,rpm_bra,rpm_brb,pos_fl,pos_fr,pos_bl,pos_br,sat\n");
        printf("# C,t_ms,cur_fla,cur_flb,cur_fra,cur_frb,cur_bla,cur_blb,cur_bra,cur_brb\n");
        printf("# Y,t_ms,yaw_x10,target_x10,err_x10,corr_x10,axis4,axis2,hold_on,active,imu_ok\n");
        printf("# K,t_ms,l1,l2,r1,r2,axis1,axis2,axis4,print_on\n");
        printf("# A,t_ms,axis1,axis2,axis3,axis4,fl_x10,fr_x10,bl_x10,br_x10,corr_x10   (25Hz 摇杆+下发)\n");
        printf("# mode: 0=manual(摇杆) 1=30%% 2=50%% 3=100%% | a/b=同轮两个电机 | rpm=整数rpm | pos=电机deg(相对本次开始) | cur=占最大电流%%\n");
      }
      // 航向环状态（开机即报，用于确认是否真的启用）
      debug_yaw_seen_enabled_ = hardware_.yaw_hold.config.enabled;
      printf("# yaw hold: cfg_enabled=%d imu_ok=%d test_mode=%d snap=%d (R2 运行时切换)\n",
             hardware_.yaw_hold.config.enabled ? 1 : 0,
             (hardware_.imu.installed() && !hardware_.imu.isCalibrating()) ? 1 : 0,
             kYawHoldInTestMode ? 1 : 0, kAxisSnapPct);
    }

    // 模式切换打一行注释（便于在数据里分段）
    if (test_speed_index_ != debug_last_mode_) {
      debug_last_mode_ = test_speed_index_;
      printf("# mode -> %d\n", debug_last_mode_);
    }

    const int t_ms = now_ms - debug_t0_ms_;

    // [只打印摇杆] 模式：每周期输出一行 A（100Hz），其余行一律不打印
    if (kDebugAxisOnly) {
      printf("A,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
             t_ms,
             state_.controller.axis1, state_.controller.axis2,
             state_.controller.axis3, state_.controller.axis4,
             round_int(s.fl_pct * 10.0), round_int(s.fr_pct * 10.0),
             round_int(s.bl_pct * 10.0), round_int(s.br_pct * 10.0),
             round_int(state_.yaw_hold.correction_pct * 10.0));
      return;
    }

    printf("D,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
           t_ms, test_speed_index_,
           rpm_of(fl[0]), rpm_of(fl[1]), rpm_of(fr[0]), rpm_of(fr[1]),
           rpm_of(bl[0]), rpm_of(bl[1]), rpm_of(br[0]), rpm_of(br[1]),
           round_int(pos_fl - debug_pos0_fl_), round_int(pos_fr - debug_pos0_fr_),
           round_int(pos_bl - debug_pos0_bl_), round_int(pos_br - debug_pos0_br_),
           sat);

    // 摇杆行（25Hz，每 kDebugAxisEveryN 个数据行一条）：手柄四轴原始值 + 最终四轮指令 pct×10
    //   axis1=右X(平移) axis2=右Y(前后) axis3=左Y(未用) axis4=左X(旋转)
    //   fl/fr/bl/br = 整形与航向修正之后的实际下发值（×10），与 D 行同一 t_ms 对齐
    //   带宽预算：D 行 ~85B@100Hz ≈ 8.5kB/s（实测 10.00ms 无丢帧），本行 ~54B@25Hz ≈ 1.4kB/s
    if ((debug_row_counter_ % kDebugAxisEveryN) == 0) {
      printf("A,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
             t_ms,
             state_.controller.axis1, state_.controller.axis2,
             state_.controller.axis3, state_.controller.axis4,
             round_int(s.fl_pct * 10.0), round_int(s.fr_pct * 10.0),
             round_int(s.bl_pct * 10.0), round_int(s.br_pct * 10.0),
             round_int(state_.yaw_hold.correction_pct * 10.0));
    }

    // 电流行：每 kDebugCurrentEveryN 个数据行一条（100Hz/10 = 10Hz）
    debug_row_counter_ += 1;
    if (debug_row_counter_ >= kDebugCurrentEveryN) {
      debug_row_counter_ = 0;
      printf("C,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
             t_ms,
             cur_of(fl[0]), cur_of(fl[1]), cur_of(fr[0]), cur_of(fr[1]),
             cur_of(bl[0]), cur_of(bl[1]), cur_of(br[0]), cur_of(br[1]));
      // 航向行（同为 10Hz）：IMU 航向保持的调参依据
      const auto& yh = state_.yaw_hold;
      printf("Y,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
             t_ms,
             round_int(yh.yaw_deg * 10.0), round_int(yh.target_yaw_deg * 10.0),
             round_int(yh.error_deg * 10.0), round_int(yh.correction_pct * 10.0),
             state_.controller.axis4, state_.controller.axis2,
             hardware_.yaw_hold.config.enabled ? 1 : 0,
             yh.active ? 1 : 0,
             (hardware_.imu.installed() && !hardware_.imu.isCalibrating()) ? 1 : 0);
      // 按键行（10Hz）：诊断用——看 L1 有没有被读到、打印开关状态
      printf("K,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
             t_ms,
             state_.controller.l1 ? 1 : 0, state_.controller.l2 ? 1 : 0,
             state_.controller.r1 ? 1 : 0, state_.controller.r2 ? 1 : 0,
             state_.controller.axis1, state_.controller.axis2,
             state_.controller.axis4, debug_print_enabled_ ? 1 : 0);
    }
  }

  /// 竞赛线程路径（恢复后使用）：仅在使能且 driver control 时运行
  void run_driver_control_loop() {
    while (should_run_driver_control()) {
      control_step();
      vex::this_thread::sleep_for(kRefreshTime);
    }

    stop_all_outputs(vex::coast);
  }

  /// 临时：常驻手动控制循环（不判断竞赛状态），程序启动即运行
  void run_manual_control_loop() {
    while (true) {
      control_step();
      vex::this_thread::sleep_for(kRefreshTime);
    }
  }

  void run_autonomous_routine() {
    if (competition_ == nullptr) {
      return;
    }

    basic::hardware::bed::autonomous::run_routine(hardware_, state_, *competition_);
    stop_all_outputs(vex::hold);
  }

  bool should_run_driver_control() const {
    return competition_ != nullptr && competition_->isEnabled() && competition_->isDriverControl();
  }

  void stop_all_outputs(vex::brakeType brake_type) {
    state_.controller = basic::hardware::shared::ControllerInputState{};
    basic::chassis::bed_chassis_stop(hardware_.bed_chassis, brake_type);
    basic::mechanism::intake_stop(hardware_.intake, vex::coast);
    basic::mechanism::pneumatic_gripper_stop(hardware_.pneumatic_gripper);
    basic::mechanism::arm_2dof_stop(hardware_.arm_2dof, vex::hold);
    basic::mechanism::linear_lift_stop(hardware_.lift, vex::hold);
    basic::mechanism::dual_pneumatic_stop(hardware_.dual_pneumatic);
  }

  RobotHardware hardware_;
  RobotState state_;
  vex::competition* competition_{nullptr};

  // 调试打印线程（L1 开关）：数据行周期 10ms = 100Hz；电流/航向行每 10 个数据行一条 = 10Hz
  static constexpr int kDebugPrintPeriodMs = 10;
  static constexpr int kDebugCurrentEveryN = 10;
  static constexpr int kDebugAxisEveryN = 4;  // 摇杆行节流：100Hz/4 = 25Hz
  bool debug_print_enabled_{true};  // 默认开机即打印；L1 边沿切换（要按 L1 才开始就改回 false）
  bool debug_session_active_{false};
  int debug_t0_ms_{0};
  int debug_last_mode_{0};
  int debug_row_counter_{0};
  int debug_toggle_seq_{0};       // 控制线程递增；打印线程据此补打 "# print ON/OFF"
  int debug_seen_toggle_seq_{0};  // 打印线程已处理到的序号
  bool debug_yaw_seen_enabled_{false};  // 航向环开关的上次状态（R2 事件行用）
  double debug_pos0_fl_{0.0};
  double debug_pos0_fr_{0.0};
  double debug_pos0_bl_{0.0};
  double debug_pos0_br_{0.0};

  // 固定速度测试模式（L2 循环切换）：索引 0 = 手动跟摇杆
  int test_speed_index_{0};

  friend BedRobot& current_bed();
};

BedRobot& current_bed() {
  static BedRobot robot;
  return robot;
}

}  // namespace

basic::app::Robot& get_robot() {
  return current_bed();
}

}  // namespace basic::hardware::bed
