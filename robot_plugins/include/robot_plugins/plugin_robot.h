#ifndef ROBOT_PLUGINS_PLUGIN_ROBOT_H_
#define ROBOT_PLUGINS_PLUGIN_ROBOT_H_

#include <ros/ros.h>
#include <robot_plugins/plugin_base.h>

namespace plugin_robot
{
class Robot: public plugin_base::RegularPlugin
{
public:
	Robot()
	{
	};
	void initialize(std::string name)
	{
		name_ = name;
	}
	std::string getName()
	{
		return name_;
	};
private:
	std::string name_;
};
};
#endif
