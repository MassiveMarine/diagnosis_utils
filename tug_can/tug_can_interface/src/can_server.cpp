#include <tug_can_interface/can_server.h>
#include <sstream>
#include <stdexcept>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <tug_can_msgs/ForwardCanMessages.h>

namespace tug_can_interface
{

const std::string CanServer::SEND_TOPIC("send");
const std::string CanServer::RECEIVE_TOPIC("receive");
const std::string CanServer::FORWARD_SERVICE("forward_can_messages");

CanServer::CanServer(ros::NodeHandle & node_handle, const CanInterfacePtr & can_interface)
    : node_handle_(node_handle), can_interface_(can_interface)
{
    try
    {
        forward_service_server_ = node_handle_.advertiseService(FORWARD_SERVICE, &CanServer::forwardCanMessagesCallback, this);
        send_topic_subscriber_ = node_handle_.subscribe(SEND_TOPIC, 20, &CanServer::sendCallback, this);
        receive_forward_.publisher_ = node_handle_.advertise<tug_can_msgs::CanMessage>(RECEIVE_TOPIC, 50);
        receive_forward_.can_subscription_ = can_interface_->subscribeToAll(
                    boost::bind(&CanServer::canCallback, this, &receive_forward_, _1));
    }
    catch (...)
    {
        receive_forward_.can_subscription_.reset();
        receive_forward_.publisher_.shutdown();
        send_topic_subscriber_.shutdown();
        forward_service_server_.shutdown();
        can_interface_.reset();
        throw;
    }
}

CanServer::~CanServer()
{
}

void CanServer::sendCallback(const tug_can_msgs::CanMessageConstPtr & can_message)
{
    can_interface_->sendMessage(can_message);
}

void CanServer::canCallback(Forward * forward, const tug_can_msgs::CanMessageConstPtr & can_message)
{
    boost::lock_guard<boost::mutex> lock(forwards_mutex_);
    forward->publisher_.publish(can_message);
}

bool CanServer::forwardCanMessagesCallback(
        tug_can_msgs::ForwardCanMessages::Request & req,
        tug_can_msgs::ForwardCanMessages::Response & res)
{
    boost::lock_guard<boost::mutex> lock(forwards_mutex_);

    try
    {
        res.topic = getForwardTopic(req.ids);
        ForwardPtr & forward = forwards_[res.topic];
        if (forward)
        {
            ROS_INFO("Requested forward already exists.");
        }
        else
        {
            forward = boost::make_shared<Forward>();
            forward->publisher_ = node_handle_.advertise<tug_can_msgs::CanMessage>(res.topic, 10);
            forward->can_subscription_ = can_interface_->subscribe(req.ids,
                        boost::bind(&CanServer::canCallback, this, forward.get(), _1));
        }
        return true;
    }
    catch (std::exception & ex)
    {
        ROS_ERROR_STREAM("Creating forward failed: " << ex.what());
    }
    return false;
}

std::string CanServer::getForwardTopic(const std::vector<uint32_t> & ids)
{
    if (ids.empty())
        throw std::invalid_argument("ids must not be empty");

    std::ostringstream topic;
    topic << RECEIVE_TOPIC << "/";
    for (size_t i = 0; i < ids.size(); ++i)
    {
        if (i != 0)
            topic << "_";
        topic << ids[i];
    }
    return node_handle_.resolveName(topic.str());
}


}
