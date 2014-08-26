#ifndef ROBOT_HARDWARE_H
#define ROBOT_HARDWARE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

namespace tug_robot_control
{
class RobotHardware: public hardware_interface::RobotHW
{
public:
  RobotHardware();

  virtual ~RobotHardware(){};

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
};
}

#endif
