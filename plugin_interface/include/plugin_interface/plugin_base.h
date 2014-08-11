#ifndef PLUGIN_INTERFACE_PLUGIN_BASE_H
#define PLUGIN_INTERFACE_PLUGIN_BASE_H

#include <ros/node_handle.h>
//#include <hardware_interface/robot_hw.h>


namespace plugin_interface
{

/** \brief Abstract %Controller Interface
 *
 *
 *
 */

class PluginBase
{
public:
  PluginBase(): state_(CONSTRUCTED){}
  virtual ~PluginBase(){}

  virtual void aFunction(const ros::Time& time) {};
 // virtual void starting(const ros::Time& time) {};

  /** \brief This is called periodically by the realtime thread when the controller is running
   *
   * \param time The current time
   * \param period The time passed since the last call to \ref update
   */
//  virtual void update(const ros::Time& time, const ros::Duration& period) = 0;

  /** \brief This is called from within the realtime thread just after the last
   * update call before the controller is stopped
   *
   * \param time The current time
   */
 // virtual void stopping(const ros::Time& time) {};

  /** \brief Check if the controller is running
   * \returns true if the controller is running
   */
/*  bool isRunning()
  {
    return (state_ == RUNNING);
  }*/

  /// Calls \ref update only if this controller is running.
 /* void updateRequest(const ros::Time& time, const ros::Duration& period)
  {
    if (state_ == RUNNING)
      update(time, period);
  }*/

  /// Calls \ref starting only if this controller is initialized or already running
  /*bool startRequest(const ros::Time& time)
  {
    // start succeeds even if the controller was already started
    if (state_ == RUNNING || state_ == INITIALIZED){
      starting(time);
      state_ = RUNNING;
      return true;
    }
    else
      return false;
  }*/

  /// Calls \ref stopping only if this controller is initialized or already running
/*  bool stopRequest(const ros::Time& time)
  {
    // stop succeeds even if the controller was already stopped
    if (state_ == RUNNING || state_ == INITIALIZED){
      stopping(time);
      state_ = INITIALIZED;
      return true;
    }
    else
      return false;
  }*/

  /*\}*/

  /** \name Non Real-Time Safe Functions
   *\{*/

  /// Get the name of this controller's hardware interface type
//  virtual std::string getHardwareInterfaceType() const = 0;

  /** \brief Request that the controller be initialized
   *
   * \param hw The hardware interface to the robot.
   *
   * \param root_nh A NodeHandle in the root of the controller manager namespace.
   * This is where the ROS interfaces are setup (publishers, subscribers, services).
   *
   * \param controller_nh A NodeHandle in the namespace of the controller.
   * This is where the controller-specific configuration resides.
   *
   * \param[out] claimed_resources The resources claimed by this controller.
   *
   * \returns True if initialization was successful and the controller
   * is ready to be started.
   */
//  virtual bool initRequest(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh,
//                           std::set<std::string>& claimed_resources) = 0;

  /*\}*/

  /// The current execution state of the controller
  enum {CONSTRUCTED, INITIALIZED, RUNNING} state_;


private:
  PluginBase(const PluginBase &c);
  PluginBase& operator =(const PluginBase &c);

};

}


#endif
