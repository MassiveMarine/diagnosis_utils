#include <tug_can_interface/can_server.h>
#include <sstream>
#include <stdexcept>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/weak_ptr.hpp>
#include <ros/single_subscriber_publisher.h>
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
        receive_forward_ = boost::make_shared<Forward>();
        receive_forward_->publisher_ = node_handle_.advertise<tug_can_msgs::CanMessage>(RECEIVE_TOPIC, 50);
        receive_forward_->can_subscription_ = can_interface_->subscribeToAll(
                    boost::bind(&CanServer::canCallback, this, ForwardWPtr(receive_forward_), _1));
    }
    catch (...)
    {
        receive_forward_.reset();
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

void CanServer::canCallback(const ForwardWPtr & forward, const tug_can_msgs::CanMessageConstPtr & can_message)
{
    boost::lock_guard<boost::mutex> lock(forwards_mutex_);
    ForwardPtr fwd = forward.lock();
    if (fwd)
        fwd->publisher_.publish(can_message);
}

bool CanServer::forwardCanMessagesCallback(
        tug_can_msgs::ForwardCanMessages::Request & req,
        tug_can_msgs::ForwardCanMessages::Response & res)
{
    boost::lock_guard<boost::mutex> lock(forwards_mutex_);

    try
    {
        res.topic = getForwardTopic(req.ids);
        if (forwards_.find(res.topic) != forwards_.end())
        {
            ROS_INFO("Requested forward already exists.");
        }
        else
        {
            ForwardPtr forward = boost::make_shared<Forward>();
            forward->publisher_ = node_handle_.advertise<tug_can_msgs::CanMessage>(res.topic, 10,
                        boost::bind(&CanServer::subscriberConnectedCallback, this, _1),
                        boost::bind(&CanServer::subscriberDisconnectedCallback, this, _1));
            forward->can_subscription_ = can_interface_->subscribe(req.ids,
                        boost::bind(&CanServer::canCallback, this, ForwardWPtr(forward), _1));
            forwards_[res.topic] = forward;
        }
        return true;
    }
    catch (std::exception & ex)
    {
        ROS_ERROR_STREAM("Creating forward failed: " << ex.what());
    }
    return false;
}

void CanServer::subscriberConnectedCallback(const ros::SingleSubscriberPublisher & subscriber)
{
    ROS_DEBUG_STREAM("Subscriber connected to topic " << subscriber.getTopic());
}

void CanServer::subscriberDisconnectedCallback(const ros::SingleSubscriberPublisher & subscriber)
{
    boost::lock_guard<boost::mutex> lock(forwards_mutex_);

    ForwardMap::iterator it = forwards_.find(subscriber.getTopic());
    if (it != forwards_.end() && it->second->publisher_.getNumSubscribers() == 0)
    {
        ROS_INFO_STREAM("Nobody listening anymore to topic " <<
                        subscriber.getTopic() << ", removing forward");
        forwards_.erase(it);
    }
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
