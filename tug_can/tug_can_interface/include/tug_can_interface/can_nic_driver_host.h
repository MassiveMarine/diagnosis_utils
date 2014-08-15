#ifndef TUG_CAN_INTERFACE__CAN_NIC_DRIVER_HOST_H_
#define TUG_CAN_INTERFACE__CAN_NIC_DRIVER_HOST_H_

#include <map>
#include <stdexcept>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <pluginlib/class_loader.h>
#include <tug_can_interface/can_interface.h>
#include <tug_can_interface/can_nic_driver.h>

namespace tug_can_interface
{

class CanNicDriverHost : public CanInterface
{
public:
    CanNicDriverHost(const std::string & driver_plugin_name,
                     const std::string & device_name, int baud_rate,
                     const ros::Duration & io_timeout,
                     ros::NodeHandle & node_handle);

    /**
     * Parses a baud rate and returns it as a number. The following formats are
     * supported:
     * "500k" => 500000
     * "1M" => 1000000
     * "10000" => 10000
     *
     * @throw std::invalid_argument if the supplied value cannot be parsed.
     */
    static int parseBaudRate(const std::string & baud_rate_string);

    virtual void sendMessage(const tug_can_msgs::CanMessageConstPtr & can_message);
    virtual SubscriptionPtr subscribe(const std::vector<uint32_t> & ids,
                                      const MessageCallback & callback);
    virtual SubscriptionPtr subscribeToAll(const MessageCallback & callback);

private:
    class SubscriptionImpl : public Subscription
    {
    public:
        SubscriptionImpl(const MessageCallback & callback);
        virtual ~SubscriptionImpl();

        MessageCallback callback_;
    };

    typedef boost::weak_ptr<SubscriptionImpl> SubscriptionImplWPtr;
    typedef std::vector<SubscriptionImplWPtr> SubscriptionVector;
    typedef std::map<uint32_t, SubscriptionVector> SubscriptionMap;

    void dispatchMessage(const tug_can_msgs::CanMessageConstPtr & can_message);
    void dispatchMessage(SubscriptionVector & subscription_vector, const tug_can_msgs::CanMessageConstPtr & can_message);

    std::string device_name_;
    int baud_rate_;
    ros::Duration io_timeout_;
    ros::NodeHandle node_handle_;

    pluginlib::ClassLoader<CanNicDriver> driver_loader_;
    CanNicDriverPtr driver_;

    SubscriptionMap subscriptions_;
    SubscriptionVector subscriptions_to_all_;
};

typedef boost::shared_ptr<CanNicDriverHost> CanNicDriverHostPtr;

}

#endif
