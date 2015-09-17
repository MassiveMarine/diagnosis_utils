#ifndef ROBOT_HARDWARE_H
#define ROBOT_HARDWARE_H

#include <hardware_interface/robot_hw.h>

namespace tug_robot_control
{
class RobotHardware: public hardware_interface::RobotHW
{
public:
  RobotHardware(){}

  virtual ~RobotHardware()
  {
  }
};
}

#endif
