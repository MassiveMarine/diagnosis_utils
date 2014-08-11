#ifndef PLUGIN_INTERFACE_H
#define PLUGIN_INTERFACE_H

#include <plugin_interface/plugin_base.h>
//#include <hardware_interface/internal/demangle_symbol.h>
//#include <hardware_interface/robot_hw.h>
//#include <hardware_interface/hardware_interface.h>
#include <ros/ros.h>


namespace plugin_interface
{

template <class T>
class Plugin: public PluginBase
{
public:
  Plugin()  {state_ = CONSTRUCTED;}
  virtual ~Plugin<T>(){}

  virtual bool init(T* hw, ros::NodeHandle &plugin_nh) {return true;};

  virtual bool init(T* hw, ros::NodeHandle& root_nh, ros::NodeHandle &plugin_nh) {return true;};


//protected:
  /** \brief Initialize the controller from a RobotHW pointer
   *
   * This calls \ref init with the hardware interface for this controller if it
   * can extract the correct interface from \c robot_hw.
   *
   */
/*  virtual bool initRequest(hardware_interface::RobotHW* robot_hw,
                           ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh,
                           std::set<std::string> &claimed_resources)
  {
    // check if construction finished cleanly
    if (state_ != CONSTRUCTED){
      ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
      return false;
    }

    // get a pointer to the hardware interface
    T* hw = robot_hw->get<T>();
    if (!hw)
    {
      ROS_ERROR("This controller requires a hardware interface of type '%s'."
                " Make sure this is registered in the hardware_interface::RobotHW class.",
                getHardwareInterfaceType().c_str());
      return false;
    }

    // return which resources are claimed by this controller
    hw->clearClaims();
    if (!init(hw, controller_nh) || !init(hw, root_nh, controller_nh))
    {
      ROS_ERROR("Failed to initialize the controller");
      return false;
    }
    claimed_resources = hw->getClaims();
    hw->clearClaims();

    // success
    state_ = INITIALIZED;
    return true;
  }*/

//  virtual std::string getHardwareInterfaceType() const
//  {
//    return hardware_interface::internal::demangledTypeName<T>();
//  }

private:
  Plugin<T>(const Plugin<T> &c);
  Plugin<T>& operator =(const Plugin<T> &c);

};

}

#endif
