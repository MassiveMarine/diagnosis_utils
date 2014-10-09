#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <ros/spinner.h>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <tug_can_interface/can_nic_driver_host.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "colin_test");
    ros::NodeHandle node_handle("~");

    ROS_INFO_STREAM("Initialized node");

    ROS_INFO_STREAM("Creating CAN interface");
	tug_can_interface::CanInterfacePtr can_interface =
			boost::make_shared<tug_can_interface::CanNicDriverHost>(
				"tug_can_nic_drivers::SocketCanNicDriver", "can0", 0,
				ros::Duration(1.0), node_handle);

	can_subscription_ = can_interface->subscribe(100 + 1, boost::bind(&CanTransport::canMessageCallback, this, _1));



}
