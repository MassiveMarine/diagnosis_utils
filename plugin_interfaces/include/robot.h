#ifndef PLUGIN_INTERFACEs_ROBOT_H
#define PLUGIN_INTERFACEs_ROBOT_H

//#include <cassert>
#include <string>
//#include <hardware_interface/internal/hardware_resource_manager.h>
//#include <hardware_interface/joint_state_interface.h>

namespace plugin_interfaces
{

/*class RobotHandle// : public JointStateHandle
{
public:
  RobotHandle() : cmd_(0) {}


  RobotHandle(double* cmd)
    : cmd_(cmd)
  {
    if (!cmd_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command data pointer is null.");
    }
  }

  void setCommand(double command) {assert(cmd_); *cmd_ = command;}
  double getCommand() const {assert(cmd_); return *cmd_;}

private:
  double* cmd_;
};*/



class RobotInterface : public JointCommandInterface
{
public:
  RobotInterface()
  {};

  ~RobotInterface()
  {};
};


class PluginInterfacesException: public std::exception
{
public:
  PluginInterfacesException(const std::string& message)
    : msg(message) {}

  virtual ~PluginInterfacesException() throw() {}

  virtual const char* what() const throw()
  {
    return msg.c_str();
  }


private:
  std::string msg;
};
}

#endif
