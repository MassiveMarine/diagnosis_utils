#ifndef TUG_CAN_INTERFACE__CAN_CLIENT_H_
#define TUG_CAN_INTERFACE__CAN_CLIENT_H_

#include <string>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/spinner.h>
#include <ros/subscriber.h>
#include <tug_can_interface/can_interface.h>
#include <tug_can_interface/can_subscription.h>
#include <tug_can_interface/forwards.h>

namespace tug_can_interface
{

class CanClient : public CanInterface
{
public:
    CanClient(const std::string & can_server_namespace, bool have_spin_thread = true, bool wait_for_server = true);
    virtual ~CanClient();

    virtual void sendMessage(const tug_can_msgs::CanMessageConstPtr & can_message);

    virtual CanSubscriptionPtr subscribe(const std::vector<uint32_t> & ids,
                                      const MessageCallback & callback);
    virtual CanSubscriptionPtr subscribeToAll(const MessageCallback & callback);

private:
    static const std::string LOG_PREFIX;
    static const uint32_t PUBLISHER_QUEUE_SIZE;
    static const uint32_t SUBSCRIBERS_QUEUE_SIZE;
    static const ros::Duration SERVICE_EXISTENCE_TIMEOUT;
    static const ros::Duration TOPIC_CONNECTION_TIMEOUT;

    class SubscriptionImpl : public CanSubscription
    {
    public:
        SubscriptionImpl(const ros::Subscriber & subscriber);
        virtual ~SubscriptionImpl();

        ros::Subscriber subscriber_;
    };

    class Spinner
    {
    public:
        Spinner();

        ros::CallbackQueue callback_queue_;
        ros::AsyncSpinner async_spinner_;
    };

    CanSubscriptionPtr subscribe(const std::string & topic, const MessageCallback & callback);
    void waitForServer(ros::Subscriber & subscriber);
    void canPublisherSubscriberConnectedCallback(
        const ros::SingleSubscriberPublisher & subscriber);
    void canPublisherSubscriberDisconnectedCallback(
        const ros::SingleSubscriberPublisher & subscriber);

    boost::mutex lock_;
    std::string can_server_namespace_;
    bool wait_for_server_;
    ros::NodeHandle node_handle_;
    ros::Publisher can_publisher_;
    boost::shared_ptr<Spinner> spinner_;

    boost::condition_variable can_server_subscribed_condition_;
    bool can_server_subscribed_;
};

}

#endif
