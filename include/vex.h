#ifndef VEX_COMPAT_H_
#define VEX_COMPAT_H_

#include "main.h"
#include "pros/serial.h"

#include <cstdarg>
#include <cstdint>
#include <cstdio>

namespace vex {

enum brakeType { coast, brake, hold };
enum directionType { fwd, rev };
enum percentUnits { pct };
enum rotationUnits { deg, degrees };
enum velocityUnits { velocity_pct = 0 };
enum timeUnits { msec };
enum controllerType { primary };

constexpr int PORT1 = 1;
constexpr int PORT2 = 2;
constexpr int PORT3 = 3;
constexpr int PORT4 = 4;
constexpr int PORT5 = 5;
constexpr int PORT6 = 6;
constexpr int PORT7 = 7;
constexpr int PORT8 = 8;
constexpr int PORT9 = 9;
constexpr int PORT10 = 10;
constexpr int PORT11 = 11;
constexpr int PORT12 = 12;
constexpr int PORT13 = 13;
constexpr int PORT14 = 14;
constexpr int PORT15 = 15;
constexpr int PORT16 = 16;
constexpr int PORT17 = 17;
constexpr int PORT18 = 18;
constexpr int PORT19 = 19;
constexpr int PORT20 = 20;
constexpr int PORT21 = 21;

enum gearSetting { ratio36_1, ratio18_1, ratio6_1 };

inline pros::motor_brake_mode_e_t to_pros_brake(brakeType mode) {
  switch (mode) {
    case brake:
      return pros::E_MOTOR_BRAKE_BRAKE;
    case hold:
      return pros::E_MOTOR_BRAKE_HOLD;
    case coast:
    default:
      return pros::E_MOTOR_BRAKE_COAST;
  }
}

inline pros::v5::MotorGears to_pros_gear(gearSetting ratio) {
  switch (ratio) {
    case ratio36_1:
      return pros::v5::MotorGears::red;
    case ratio6_1:
      return pros::v5::MotorGears::blue;
    case ratio18_1:
    default:
      return pros::v5::MotorGears::green;
  }
}

class motor {
 public:
  motor()
      : device_(1, pros::v5::MotorGears::green,
                pros::v5::MotorUnits::degrees) {}

  motor(int port, gearSetting ratio = ratio18_1, bool reversed = false)
      : device_(static_cast<std::int8_t>(reversed ? -port : port),
                to_pros_gear(ratio), pros::v5::MotorUnits::degrees),
        port_(port) {}

  void spin(directionType dir, double value, percentUnits) {
    const double signed_value = (dir == rev ? -value : value);
    device_.move_velocity(static_cast<std::int32_t>(signed_value * 2.0));
  }

  void stop(brakeType mode = coast) {
    device_.set_brake_mode(to_pros_brake(mode));
    device_.brake();
  }

  void spinFor(double value, rotationUnits, bool wait = true) {
    device_.move_relative(value, 100);
    if (wait) {
      while (device_.get_actual_velocity() != 0) {
        pros::delay(10);
      }
    }
  }

  void spinFor(double value, rotationUnits, double velocity, velocityUnits,
               bool wait = true) {
    device_.move_relative(value, static_cast<std::int32_t>(velocity * 2.0));
    if (wait) {
      while (device_.get_actual_velocity() != 0) {
        pros::delay(10);
      }
    }
  }

  double position(rotationUnits) const { return device_.get_position(); }

  int index() const { return port_; }

 private:
  pros::Motor device_;
  int port_{1};
};

class inertial {
 public:
  explicit inertial(int port) : device_(static_cast<std::uint8_t>(port)) {}

  void calibrate() { device_.reset(false); }

  bool isCalibrating() const { return device_.is_calibrating(); }

  void resetHeading() { device_.tare_heading(); }

  void resetRotation() { device_.tare_rotation(); }

  double rotation(rotationUnits) const { return device_.get_rotation(); }

 private:
  pros::Imu device_;
};

class brain {
 public:
  class screen {
   public:
    void clearScreen() const { pros::lcd::clear(); }

    void setCursor(int row, int col) {
      row_ = row < 1 ? 1 : row;
      col_ = col < 1 ? 1 : col;
    }

    void print(const char* fmt, ...) const {
      char buffer[128];
      va_list args;
      va_start(args, fmt);
      std::vsnprintf(buffer, sizeof(buffer), fmt, args);
      va_end(args);
      pros::lcd::print(row_ - 1, "%*s%s", col_ - 1, "", buffer);
    }

    void newLine() { setCursor(row_ + 1, 1); }

   private:
    mutable int row_{1};
    mutable int col_{1};
  };

  class timer_api {
   public:
    double value() const { return static_cast<double>(pros::millis()) / 1000.0; }
  };

  int timer(timeUnits) const { return static_cast<int>(pros::millis()); }

  screen Screen;
  timer_api Timer;
};

class controller {
 public:
  class axis {
   public:
    axis() = default;
    axis(pros::Controller* device, pros::controller_analog_e_t channel)
        : device_(device), channel_(channel) {}

    int position(percentUnits) const {
      return device_ ? device_->get_analog(channel_) : 0;
    }

   private:
    pros::Controller* device_{nullptr};
    pros::controller_analog_e_t channel_{pros::E_CONTROLLER_ANALOG_LEFT_X};
  };

  class button {
   public:
    button() = default;
    button(pros::Controller* device, pros::controller_digital_e_t button)
        : device_(device), button_(button) {}

    bool pressing() const {
      return device_ ? static_cast<bool>(device_->get_digital(button_)) : false;
    }

   private:
    pros::Controller* device_{nullptr};
    pros::controller_digital_e_t button_{pros::E_CONTROLLER_DIGITAL_A};
  };

  class screen {
   public:
    explicit screen(pros::Controller* device) : device_(device) {}

    void setCursor(int row, int col) {
      row_ = row < 1 ? 1 : row;
      col_ = col < 1 ? 1 : col;
    }

    void print(const char* fmt, ...) const {
      if (!device_) return;
      char buffer[128];
      va_list args;
      va_start(args, fmt);
      std::vsnprintf(buffer, sizeof(buffer), fmt, args);
      va_end(args);
      device_->set_text(static_cast<std::uint8_t>(row_ - 1),
                        static_cast<std::uint8_t>(col_ - 1), buffer);
    }

   private:
    pros::Controller* device_{nullptr};
    mutable int row_{1};
    mutable int col_{1};
  };

  explicit controller(controllerType)
      : device_(pros::E_CONTROLLER_MASTER),
        Axis1(&device_, pros::E_CONTROLLER_ANALOG_RIGHT_X),
        Axis2(&device_, pros::E_CONTROLLER_ANALOG_RIGHT_Y),
        Axis3(&device_, pros::E_CONTROLLER_ANALOG_LEFT_Y),
        Axis4(&device_, pros::E_CONTROLLER_ANALOG_LEFT_X),
        ButtonL1(&device_, pros::E_CONTROLLER_DIGITAL_L1),
        ButtonL2(&device_, pros::E_CONTROLLER_DIGITAL_L2),
        ButtonR1(&device_, pros::E_CONTROLLER_DIGITAL_R1),
        ButtonR2(&device_, pros::E_CONTROLLER_DIGITAL_R2),
        ButtonX(&device_, pros::E_CONTROLLER_DIGITAL_X),
        ButtonY(&device_, pros::E_CONTROLLER_DIGITAL_Y),
        ButtonA(&device_, pros::E_CONTROLLER_DIGITAL_A),
        ButtonB(&device_, pros::E_CONTROLLER_DIGITAL_B),
        ButtonLeft(&device_, pros::E_CONTROLLER_DIGITAL_LEFT),
        ButtonRight(&device_, pros::E_CONTROLLER_DIGITAL_RIGHT),
        ButtonUp(&device_, pros::E_CONTROLLER_DIGITAL_UP),
        ButtonDown(&device_, pros::E_CONTROLLER_DIGITAL_DOWN),
        Screen(&device_) {}

  pros::Controller device_;
  axis Axis1;
  axis Axis2;
  axis Axis3;
  axis Axis4;
  button ButtonL1;
  button ButtonL2;
  button ButtonR1;
  button ButtonR2;
  button ButtonX;
  button ButtonY;
  button ButtonA;
  button ButtonB;
  button ButtonLeft;
  button ButtonRight;
  button ButtonUp;
  button ButtonDown;
  screen Screen;
};

class timer {
 public:
  void system() { start_ = pros::millis(); }

 private:
  std::uint32_t start_{0};
};

class thread {
 public:
  explicit thread(void (*fn)()) : task_([fn] { fn(); }) {}

 private:
  pros::Task task_;
};

namespace this_thread {
inline void sleep_for(int ms) { pros::delay(static_cast<std::uint32_t>(ms)); }
}  // namespace this_thread

}  // namespace vex

inline void vexGenericSerialEnable(int port, int) { pros::c::serial_enable(port); }
inline void vexGenericSerialBaudrate(int port, int baud) {
  pros::c::serial_set_baudrate(port, baud);
}
inline int vexGenericSerialReceiveAvail(int port) {
  return pros::c::serial_get_read_avail(port);
}
inline int vexGenericSerialReadChar(int port) {
  return pros::c::serial_read_byte(port);
}

#define waitUntil(condition)      \
  do {                            \
    pros::delay(5);               \
  } while (!(condition))

#define COMPETITION

#endif
