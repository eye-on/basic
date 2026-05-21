/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       86135                                                     */
/*    Created:      2025/9/10 17:33:24                                        */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "hardware/robot_selector.h"

int main() {
  basic::app::Robot& robot = basic::hardware::get_current_robot();
  robot.initialize();
  robot.bind_background_tasks();

#ifdef COMPETITION
  static vex::competition competition;
  robot.bind_competition(competition);
#endif

  return 0;
}
