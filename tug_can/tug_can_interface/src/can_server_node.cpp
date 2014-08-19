#include <tug_can_interface/can_server_node.h>
#include <iostream>
#include <boost/make_shared.hpp>
#include <ros/init.h>
#include <tug_can_interface/can_nic_driver_host.h>
#include <tug_can_interface/can_server.h>

int main(int argc, char ** argv)
{
    try
    {
        ros::init(argc, argv, "can_server");
        tug_can_interface::CanServerNode node;
        node.run();
        return 0;
    }
    catch (std::exception & ex)
    {
        ROS_FATAL_STREAM("Unhandled exception in main: " << ex.what());
        std::cerr << "Unhandled exception in main: " << ex.what() << std::endl;
    }
    catch (...)
    {
        ROS_FATAL_STREAM("Unhandled exception in main");
        std::cerr << "Unhandled exception in main" << std::endl;
    }
    return 1;
}

namespace tug_can_interface
{

CanServerNode::CanServerNode()
    : node_handle_("~")
{
}

void CanServerNode::run()
{
    loadParameters();

    can_nic_driver_host_ = boost::make_shared<CanNicDriverHost>(
                parameters_.driver_, parameters_.device_,
                CanNicDriverHost::parseBaudRate(parameters_.baud_rate_),
                ros::Duration(parameters_.io_timeout_), node_handle_);

    can_server_ = boost::make_shared<CanServer>(node_handle_, can_nic_driver_host_);

    ros::spin();
}

void CanServerNode::loadParameters()
{
    getRequiredParameter("driver", parameters_.driver_);
    getRequiredParameter("device", parameters_.device_);
    getRequiredParameter("baud_rate", parameters_.baud_rate_);
    getOptionalParameter("io_timeout", parameters_.io_timeout_, 1.0);
}

template <typename T>
void CanServerNode::getRequiredParameter(const std::string & key, T & value)
{
    if (!getParameter(key, value))
    {
        ROS_FATAL_STREAM("Missing required node parameter " << key);
        throw std::runtime_error("Missing required node parameter " + key);
    }
}

template <typename T>
void CanServerNode::getOptionalParameter(const std::string & key, T & value, T default_value)
{
    if (!getParameter(key, value))
    {
        value = default_value;
        ROS_WARN_STREAM("Missing optional node parameter " << key << ", using default value '" << default_value << "'");
    }
}

template <typename T>
bool CanServerNode::getParameter(const std::string & key, T & value)
{
    return node_handle_.getParam(key, value);
}

}
