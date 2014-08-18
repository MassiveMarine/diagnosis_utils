#ifndef TUG_CAN_INTERFACE__CAN_SERVER_H_
#define TUG_CAN_INTERFACE__CAN_SERVER_H_

#include <map>
#include <boost/thread/mutex.hpp>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#include <tug_can_interface/can_interface.h>
#include <tug_can_msgs/CanMessage.h>
#include <tug_can_msgs/ForwardCanMessages.h>

namespace tug_can_interface
{

class CanServer
{
public:
    static const std::string SEND_TOPIC;
    static const std::string RECEIVE_TOPIC;
    static const std::string FORWARD_SERVICE;

    CanServer(ros::NodeHandle & node_handle, const CanInterfacePtr & can_interface);
    virtual ~CanServer();

protected:
    struct Forward
    {
        CanInterface::SubscriptionPtr can_subscription_;
        ros::Publisher publisher_;
    };

    typedef boost::shared_ptr<Forward> ForwardPtr;
    typedef std::map<std::string, ForwardPtr> ForwardMap;

    void sendCallback(const tug_can_msgs::CanMessageConstPtr & can_message);
    void canCallback(Forward * forward, const tug_can_msgs::CanMessageConstPtr & can_message);
    bool forwardCanMessagesCallback(
            tug_can_msgs::ForwardCanMessages::Request & req,
            tug_can_msgs::ForwardCanMessages::Response & res);
    std::string getForwardTopic(const std::vector<uint32_t> & ids);

    ros::NodeHandle node_handle_;
    ros::ServiceServer forward_service_server_;
    ros::Subscriber send_topic_subscriber_;

    CanInterfacePtr can_interface_;
    Forward receive_forward_;

    boost::mutex forwards_mutex_;
    ForwardMap forwards_;
};

typedef boost::shared_ptr<CanServer> CanServerPtr;

}

#endif
