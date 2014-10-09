#ifndef KRIKKIT_H_
#define KRIKKIT_H_

#include <ros/ros.h>
#include <ros/node_handle.h>

class Krikkit
{
public:
  Krikkit(ros::NodeHandle node_handle);

private:
  ros::NodeHandle node_handle_;
};
#endif /* KRIKKIT_H_ */
