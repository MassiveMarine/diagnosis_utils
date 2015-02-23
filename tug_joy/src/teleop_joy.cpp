#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#define COMMAND_FREQ 40 // Hz
class TeleopJoy
{
private:
  // Axis and buttons configuration as for Saitek PP21:
  enum JoystickAxis
  {
    JOYSTICK_AXIS_LEFT_ANALOG_STICK_HORZ = 0,
    JOYSTICK_AXIS_LEFT_ANALOG_STICK_VERT = 1,
    JOYSTICK_AXIS_RIGHT_ANALOG_STICK_HORZ = 3,
    JOYSTICK_AXIS_RIGHT_ANALOG_STICK_VERT = 2,
    JOYSTICK_AXIS_D_PAD_HORZ = 4,
    JOYSTICK_AXIS_D_PAD_VERT = 5,
  };

  enum JoystickButton
  {
    JOYSTICK_BUTTON_1 = 7,
    JOYSTICK_BUTTON_2 = 4,
    JOYSTICK_BUTTON_3 = 6,
    JOYSTICK_BUTTON_4 = 5,
    JOYSTICK_BUTTON_UL_SHOULDER = 4, // upper left
    JOYSTICK_BUTTON_LL_SHOULDER = 5, // lower left
    JOYSTICK_BUTTON_UR_SHOULDER = 6, // upper right
    JOYSTICK_BUTTON_LR_SHOULDER = 7, // lower right
    JOYSTICK_BUTTON_BACK = 8,
    JOYSTICK_BUTTON_START = 9,
    JOYSTICK_BUTTON_LEFT_ANALOG_STICK = 10,
    JOYSTICK_BUTTON_RIGHT_ANALOG_STICK = 11,
  };

  geometry_msgs::Twist cmdvel_;
  ros::NodeHandle nh_;
  ros::Publisher cmdvel_pub_;

  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  ros::Subscriber joy_sub_;

  double max_tv_, max_rv_, min_tv_, min_rv_, basic_tv_, basic_rv_;
  bool send_cmd_;

public:
  TeleopJoy()
  {
    ros::NodeHandle private_nh("~");

//    private_nh.param<double>("MaxTransVel", max_tv_, 1.0);
    private_nh.param<double>("MaxTransVel", max_tv_, 2.0);
    private_nh.param<double>("MinTransVel", min_tv_, 0.1);
//    private_nh.param<double>("MaxRotVel", max_rv_, 1.5708);
    private_nh.param<double>("MaxRotVel", max_rv_, 3.1415);
    private_nh.param<double>("MinRotVel", min_rv_, 0.4);
    private_nh.param<double>("InitTransVel", basic_tv_, 0.5);
    private_nh.param<double>("InitRotVel", basic_rv_, 0.8);
    private_nh.param<bool>("SendCmd", send_cmd_, true);

    cmdvel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10,
        &TeleopJoy::joyCallback, this);
  }

  ~TeleopJoy()
  {
  }

  void sendCommand()
  {
    if (send_cmd_)
      cmdvel_pub_.publish(cmdvel_);
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_joy");
  TeleopJoy teleop_joy;

  // set the frequency to loop at
  ros::Rate loop_rate(COMMAND_FREQ);

  while (ros::ok())
  {
    teleop_joy.sendCommand();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return (0);
}

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  cmdvel_.linear.x = basic_tv_
      * joy->axes.at(JOYSTICK_AXIS_LEFT_ANALOG_STICK_VERT);
  cmdvel_.linear.y = basic_tv_
      * joy->axes.at(JOYSTICK_AXIS_RIGHT_ANALOG_STICK_HORZ);
  cmdvel_.angular.z = basic_rv_
      * joy->axes.at(JOYSTICK_AXIS_LEFT_ANALOG_STICK_HORZ);

  if (joy->buttons.at(JOYSTICK_BUTTON_2) == 1)
  {
    basic_tv_ += max_tv_ / 10;
  }
  if (joy->buttons.at(JOYSTICK_BUTTON_3) == 1)
  {
    basic_tv_ -= max_tv_ / 10;
  }
  if (joy->buttons.at(JOYSTICK_BUTTON_4) == 1)
  {
    basic_rv_ += max_rv_ / 10;
  }
  if (joy->buttons.at(JOYSTICK_BUTTON_1) == 1)
  {
    basic_rv_ -= max_rv_ / 10;
  }
  if (joy->buttons.at(JOYSTICK_BUTTON_START) == 1)
  {
    send_cmd_ = !send_cmd_;
  }

  if (basic_tv_ > max_tv_)
    basic_tv_ = max_tv_;
  if (basic_rv_ > max_rv_)
    basic_rv_ = max_rv_;
  if (basic_tv_ < min_tv_)
    basic_tv_ = min_tv_;
  if (basic_rv_ < min_rv_)
    basic_tv_ = min_rv_;

  cmdvel_pub_.publish(cmdvel_);
}

