#ifndef TUG_CAN_INTERFACE__CAN_NIC_DRIVER_H_
#define TUG_CAN_INTERFACE__CAN_NIC_DRIVER_H_

#include <string>
#include <boost/noncopyable.hpp>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <tug_can_msgs/CanMessage.h>

namespace tug_can_interface
{

/**
 * Interface for CAN NIC (network interface card) driver plugins.
 *
 * Implementations must guarantee thread safety. In particular, implementations must ensure that blocking methods like
 * readMessage() and writeMessage() will not block each other -- e.g., writing a message or closing the device must be
 * possible while readMessage() is waiting for a message.
 *
 * readMessage() and writeMessage() should be interruptible via boost::thread::interrupt(). If this is not possible,
 * they must return within io_timeout after close() has been called.
 *
 * Implementations should prepend all log output with a prefix corresponding to their name.
 */
class CanNicDriver : public boost::noncopyable
{
public:
    class Exception : public std::runtime_error
    {
    public:
        Exception(const std::string & message)
            : std::runtime_error(message)
        {
        }
    };

    class IllegalArgumentException : public Exception
    {
    public:
        IllegalArgumentException(const std::string & message)
            : Exception(message)
        {
        }
    };

    class IllegalStateException : public Exception
    {
    public:
        IllegalStateException(const std::string & message)
            : Exception(message)
        {
        }
    };

    class IoException : public Exception
    {
    public:
        IoException(const std::string & message)
            : Exception(message)
        {
        }
    };

    CanNicDriver()
    {
    }

    virtual ~CanNicDriver()
    {
    }

    /**
     * Initializes the plugin. Does not open the device; this is done in open(). Implementations should not allow
     * multiple invocation of this method.
     *
     * @param node_handle  a ROS NodeHandle. Can be used to supply further
     *         parameters to the plugin.
     * @param device_name  the device's name (e.g., "/dev/can0" or "192.168.5.10:2001")
     * @param baud_rate    the baud rate in symbols per second (e.g., 500000
     *         for 500k). Can be zero if not applicable for the device, or if
     *         a preset value should be used.
     * @param io_timeout   timeout within which readMessage() and writeMessage()
     *         should return after close() has been called.
     * @throw IllegalStateException  if device is already initialized or open.
     * @throw IllegalArgumentException  if one of the parameters contains an unsupported value.
     */
    virtual void initialize(ros::NodeHandle & node_handle,
                            const std::string & device_name, int baud_rate,
                            const ros::Duration & io_timeout) = 0;

    /**
     * Opens the CAN interface device.
     *
     * @throw IllegalStateException  if the plugin is not initialized, or the
     *         device is already open.
     * @throw IoException  if an I/O error occured when trying to open the
     *         device. Implementations should return to an initialized state, so
     *         the client code can try again later.
     */
    virtual void open() = 0;

    /**
     * Closes the CAN interface device.
     *
     * Doesn't throw to keep things simple. Implementations should handle all errors (possibly logging them).
     */
    virtual void close() = 0;

    /**
     * Reads a single message from the CAN bus. Blocks if no message is currently available.
     *
     * Must either be interruptible via boost::thread::interrupt(), or return within IO_TIMEOUT after close() has been
     * called.
     *
     * @param message  buffer for the read message.
     * @throw IllegalStateException  if the device is not open.
     * @throw IoException  if an I/O error occurred while trying to read a message.
     */
    virtual void read(const tug_can_msgs::CanMessagePtr & message) = 0;

    /**
     * Writes a single message to the CAN bus. May block if the device's buffers are full.
     *
     * Must either be interruptible via boost::thread::interrupt(), or return within IO_TIMEOUT after close() has been
     * called.
     *
     * @param message  the message to write.
     * @throw IllegalStateException  if the device is not open.
     * @throw IllegalArgumentException  if the message is invalid (e.g., contains more than 8 bytes of payload).
     * @throw IoException  if an I/O error occurred while trying to write the message.
     */
    virtual void write(const tug_can_msgs::CanMessageConstPtr & message) = 0;
};

}

#endif
