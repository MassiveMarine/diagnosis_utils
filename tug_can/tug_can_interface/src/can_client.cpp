#include <tug_can_interface/can_client.h>
#include <sstream>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <ros/names.h>
#include <ros/single_subscriber_publisher.h>
#include <tug_can_interface/can_server.h>

namespace tug_can_interface
{

const std::string CanClient::LOG_PREFIX("tug_can_interface::CanClient: ");
const uint32_t CanClient::PUBLISHER_QUEUE_SIZE(10);
const uint32_t CanClient::SUBSCRIBERS_QUEUE_SIZE(10);
const ros::Duration CanClient::SERVICE_EXISTENCE_TIMEOUT(3);
const ros::Duration CanClient::TOPIC_CONNECTION_TIMEOUT(3);

CanClient::CanClient(const std::string & can_server_namespace, bool have_spin_thread, bool wait_for_server)
    : can_server_namespace_(can_server_namespace), wait_for_server_(wait_for_server)
{
    try
    {
        if (have_spin_thread)
        {
            spinner_ = boost::make_shared<Spinner>();
            node_handle_.setCallbackQueue(&spinner_->callback_queue_);
        }

        try
        {
            can_publisher_ = node_handle_.advertise<tug_can_msgs::CanMessage>(
                        ros::names::resolve(can_server_namespace_, CanServer::SEND_TOPIC),
                        PUBLISHER_QUEUE_SIZE,
                        boost::bind(&CanClient::canPublisherSubscriberConnectedCallback, this, _1),
                        boost::bind(&CanClient::canPublisherSubscriberDisconnectedCallback, this, _1));
        }
        catch (ros::Exception & ex)
        {
            std::ostringstream message;
            message << LOG_PREFIX << "constructor: failed to create CAN publisher: got ros::Exception: " << ex.what();
            throw Exception(message.str());
        }

        if (wait_for_server_)
        {
            ROS_DEBUG_STREAM(LOG_PREFIX << "constructor: waiting for server");

            boost::unique_lock<boost::mutex> lock(lock_);
            while (!can_server_subscribed_)
                can_server_subscribed_condition_.wait(lock);
        }
    }
    catch (...)
    {
        can_publisher_.shutdown();
        node_handle_.setCallbackQueue(0);
        spinner_.reset();
        throw;
    }
}

CanClient::~CanClient()
{
}

void CanClient::sendMessage(const tug_can_msgs::CanMessageConstPtr & can_message)
{
    checkMessage(can_message);
    can_publisher_.publish(can_message);
}

CanSubscriptionPtr CanClient::subscribe(const std::vector<uint32_t> & ids,
                                  const MessageCallback & callback)
{
    if (ids.empty())
      throw Exception(LOG_PREFIX + "subscribe: no id given");

    ROS_DEBUG_STREAM(LOG_PREFIX << "subscribe: starting ForwardCanMessages service client");
    ros::ServiceClient forward_can_messages_service;

    try
    {
      forward_can_messages_service =
              node_handle_.serviceClient<tug_can_msgs::ForwardCanMessages>(
                  ros::names::resolve(can_server_namespace_,
                                      CanServer::FORWARD_SERVICE));
    }
    catch (ros::Exception & ex)
    {
      throw Exception(
          LOG_PREFIX
              + "subscribe: failed to create service client: got ros::Exception: "
              + ex.what());
    }

    ROS_DEBUG_STREAM(LOG_PREFIX << "subscribe: waiting for ForwardCanMessages server");
    if (!forward_can_messages_service.waitForExistence(SERVICE_EXISTENCE_TIMEOUT))
    {
      throw Exception(
          LOG_PREFIX + "subscribe: failed connect to service: timeout");
    }

    tug_can_msgs::ForwardCanMessages args;
    args.request.ids = ids;

    if (!forward_can_messages_service.call(args))
      throw Exception(LOG_PREFIX + "subscribe: ForwardCanMessages service call failed");

    try
    {
        return subscribe(args.response.topic, callback);
    }
    catch (ros::Exception & ex)
    {
        throw Exception(LOG_PREFIX + "subscribe: failed to subscribe to "
                        "ForwardCanMessages topic: got ros::Exception: " +
                        ex.what());
    }
}

CanSubscriptionPtr CanClient::subscribeToAll(const MessageCallback & callback)
{
    try
    {
        return subscribe(ros::names::resolve(can_server_namespace_, CanServer::RECEIVE_TOPIC), callback);
    }
    catch (ros::Exception & ex)
    {
        std::ostringstream message;
        message << "subscribeToAll: failed to create subscriber: got ros::Exception: " << ex.what();
        throw Exception(message.str());
    }
}

CanSubscriptionPtr CanClient::subscribe(const std::string & topic,
                                        const MessageCallback & callback)
{
    ros::Subscriber subscriber = node_handle_.subscribe(topic,
                SUBSCRIBERS_QUEUE_SIZE, callback, ros::VoidConstPtr(),
                ros::TransportHints().tcpNoDelay());
    if (wait_for_server_)
        waitForServer(subscriber);
    return boost::make_shared<SubscriptionImpl>(subscriber);
}

void CanClient::waitForServer(ros::Subscriber & subscriber)
{
    // Wait until server has connected to our subscriber (may take several
    // 100ms):
    ros::Time start_time = ros::Time::now();
    while (subscriber.getNumPublishers() == 0)
    {
        boost::this_thread::sleep(boost::posix_time::milliseconds(50));
        ros::Duration elapsed_time = ros::Time::now() - start_time;
        if (elapsed_time >= TOPIC_CONNECTION_TIMEOUT)
            throw Exception(LOG_PREFIX + "waitForServer: topic connection timeout");
    }
}

void CanClient::canPublisherSubscriberConnectedCallback(
    const ros::SingleSubscriberPublisher & subscriber)
{
    ROS_DEBUG_STREAM(
                LOG_PREFIX << "subscriber connected to can_publisher_: " << subscriber.getSubscriberName());
    if (subscriber.getSubscriberName() == can_server_namespace_)
    {
        boost::lock_guard<boost::mutex> lock(lock_);
        can_server_subscribed_ = true;
        can_server_subscribed_condition_.notify_all();
    }
}

void CanClient::canPublisherSubscriberDisconnectedCallback(
    const ros::SingleSubscriberPublisher & subscriber)
{
    ROS_DEBUG_STREAM(
                LOG_PREFIX << "subscriber disconnected from can_publisher_: " << subscriber.getSubscriberName());
    if (subscriber.getSubscriberName() == can_server_namespace_)
    {
        ROS_WARN_STREAM(LOG_PREFIX << "CanServer disconnected from can_publisher_");
        boost::lock_guard<boost::mutex> lock(lock_);
        can_server_subscribed_ = false;
        can_server_subscribed_condition_.notify_all();
    }
}

CanClient::SubscriptionImpl::SubscriptionImpl(const ros::Subscriber & subscriber)
    : subscriber_(subscriber)
{
}

CanClient::SubscriptionImpl::~SubscriptionImpl()
{
}

CanClient::Spinner::Spinner()
    : callback_queue_(), async_spinner_(1, &callback_queue_)
{
}


}
