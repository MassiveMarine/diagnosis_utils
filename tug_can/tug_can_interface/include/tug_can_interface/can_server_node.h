#ifndef TUG_CAN_INTERFACE__CAN_SERVER_NODE_H_
#define TUG_CAN_INTERFACE__CAN_SERVER_NODE_H_

#include <ros/node_handle.h>
#include <tug_can_interface/forwards.h>

namespace tug_can_interface
{

class CanServerNode
{
public:
    CanServerNode();

    void run();

private:
    struct Parameters
    {
        std::string driver_;
        std::string device_;
        std::string baud_rate_;
        double io_timeout_;
    };

    void loadParameters();
    template <typename T> void getRequiredParameter(const std::string & key, T & value);
    template <typename T> void getOptionalParameter(const std::string & key, T & value, T default_value);
    template <typename T> bool getParameter(const std::string & key, T & value);

    ros::NodeHandle node_handle_;
    Parameters parameters_;
    CanNicDriverHostPtr can_nic_driver_host_;
    CanServerPtr can_server_;
};

}

#endif
