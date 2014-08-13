#ifndef TUG_CAN_INTERFACE__CAN_INTERFACE_H_
#define TUG_CAN_INTERFACE__CAN_INTERFACE_H_

#include <stdexcept>
#include <vector>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <tug_can_msgs/CanMessage.h>

namespace tug_can_interface
{

/**
 * Used to send and receive messages to and from a CAN bus.
 *
 * This class is an abstract base class for a simple interface to a CAN bus.
 * Subclasses must implement the following methods:
 * {@code virtual void sendMessage(const tug_can_msgs::CanMessageConstPtr &
 *         can_message);}
 * Sends a message to the CAN bus. Must check the message for validity (possibly
 * using the supplied checkMessage() method), throwing std::invalid_argument on
 * failure. If an I/O or other error occurs, CanInterface::Exception
 * should be thrown.
 * {@code virtual SubscriptionPtr subscribe(const std::vector<uint32_t> & ids,
 *         const MessageCallback & callback);}
 * Registers a callback to be called when a message with one of the specified
 * IDs is received. Must return a boost::shared_ptr to a Subscription
 * implementation, which must unregister the callback on destruction.
 * {@code virtual SubscriptionPtr subscribeToAll(const MessageCallback &
 *         callback);}
 * Registers a callback to be called every time a message is received. Must
 * return a boost::shared_ptr to a Subscription implementation, which must
 * unregister the callback on destruction.
 *
 * Several helper methods are defined for ease of use.
 */
class CanInterface
{
public:
    typedef boost::function<void(const tug_can_msgs::CanMessageConstPtr &)> MessageCallback;

    class Exception : public std::runtime_error
    {
    public:
        Exception(const std::string & what_arg);
    };

    class Subscription
    {
    public:
        virtual ~Subscription();
    };

    typedef boost::shared_ptr<Subscription> SubscriptionPtr;

    virtual ~CanInterface();

    virtual void sendMessage(const tug_can_msgs::CanMessageConstPtr & can_message) = 0;

    void sendMessage(uint32_t id, bool rtr, bool extended,
                     const std::vector<uint8_t> & data);
    void sendMessage(uint32_t id, bool rtr, bool extended,
                     const std::vector<uint8_t> & data, size_t start_index, size_t size);
    void sendMessage(uint32_t id, bool rtr, bool extended, const uint8_t * data,
                     size_t size);

    virtual SubscriptionPtr subscribe(const std::vector<uint32_t> & ids,
                                      const MessageCallback & callback) = 0;
    virtual SubscriptionPtr subscribeToAll(const MessageCallback & callback) = 0;

    SubscriptionPtr subscribe(uint32_t id, const MessageCallback & callback);

    static tug_can_msgs::CanMessagePtr createMessage(uint32_t id, bool rtr, bool extended);

    static void checkMessage(const tug_can_msgs::CanMessageConstPtr & can_message);
};

typedef boost::shared_ptr<CanInterface> CanInterfacePtr;

}

#endif
