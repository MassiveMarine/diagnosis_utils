#ifndef TUG_CAN_INTERFACE__CAN_NIC_DRIVER_H_
#define TUG_CAN_INTERFACE__CAN_NIC_DRIVER_H_

#include <stdexcept>
#include <string>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <tug_can_msgs/CanMessage.h>
#include <tug_can_interface/forwards.h>

namespace tug_can_interface
{

/**
 * Interface for CAN NIC (network interface card) driver plugins.
 *
 * Implementations must guarantee thread safety. In particular, implementations must ensure that blocking methods like
 * read() and write() will not block each other -- e.g., writing a message or closing the device must be
 * possible while read() is waiting for a message.
 *
 * read() and write() should be interruptible via boost::thread::interrupt(). If this is not possible,
 * they must return within io_timeout after close() has been called.
 *
 * Impementations must implement loopback functionality, that is, read() must
 * also return messages sent via write(). If possible, this should be
 * implemented using hardware features.
 *
 * Implementations should prepend all log output with a prefix corresponding to their name.
 */
class CanNicDriver : public boost::noncopyable
{
public:
    class Exception : public std::runtime_error
    {
    public:
        enum Cause
        {
            ILLEGAL_ARGUMENT, ///< The supplied arguments contain invalid or unsupported values.
            ILLEGAL_STATE, ///< Driver is in wrong state for requested action (e.g., initialize() was called after open()).
            DEVICE_CLOSED, ///< Device was closed (or not yet opened).
            DEVICE_UNAVAILABLE, ///< Device is not available (no device with the given name, or device has been disconnected).
            BUS_ERROR, ///< Device reported a bus error.
            IO_ERROR ///< An I/O error occurred.
        };

        Exception(Cause cause, const std::string & message);

        virtual const char * what() const throw();
        std::string getMessage() const;
        Cause getCause() const;

        static std::string getCauseName(Cause cause);

    private:
        Cause cause_;
    };

    CanNicDriver();
    virtual ~CanNicDriver();

    /**
     * Initializes the plugin and opens the device. May be called again after
     * close() was called.
     *
     * @param device_name  the device's name (e.g., "/dev/can0" or "192.168.5.10:2001")
     * @param baud_rate    the baud rate in symbols per second (e.g., 500000
     *         for 500k). Can be zero if not applicable for the device, or if
     *         a preset value should be used.
     * @param io_timeout   timeout within which read() and write()
     *         should return after close() has been called.
     * @param node_handle  a ROS NodeHandle. Can be used to supply further
     *         parameters to the plugin.
     * @throw Exception with cause
     *         ILLEGAL_ARGUMENT if one of the parameters contains an unsupported
     *                 value;
     *         ILLEGAL_STATE if a device is already open;
     *         DEVICE_UNAVAILABLE if a device with the given name does not
     *                 exist or cannot be opened;
     *         IO_ERROR if an I/O error occured when trying to open the
     *                 device; implementations should return to a known state,
     *                 so the client code may try again later.
     */
    virtual void open(const std::string & device_name, int baud_rate,
                      const ros::Duration & io_timeout,
                      const ros::NodeHandle & node_handle) = 0;

    /**
     * Closes the CAN interface device. Does nothing if device is not open.
     *
     * Doesn't throw to keep things simple. Implementations should handle all
     * errors (possibly logging them).
     */
    virtual void close() = 0;

    /**
     * Reads a single message from the CAN bus. Blocks until a message is
     * available.
     *
     * Must either be interruptible via boost::thread::interrupt(), or return
     * within io_timeout after close() has been called.
     *
     * Implementations should try to fill in the timestamp with the most
     * accurate value available.
     *
     * @param message  buffer for the read message.
     * @throw Exception with cause
     *         IO_ERROR if an I/O error occurred;
     *         DEVICE_UNAVAILABLE if the device does not respond, or has been
     *                 removed from the system;
     *         DEVICE_CLOSED if the device is not yet open, or close() has been called;
     *         BUS_ERROR if the device reports a bus error; this is usually a
     *                 temporary condition, so clients may log this condition
     *                 and then retry reading.
     */
    virtual void read(const tug_can_msgs::CanMessagePtr & message) = 0;

    /**
     * Writes a single message to the CAN bus. May block if the device's buffers
     * are full. The message's timestamp is ignored.
     *
     * Must either be interruptible via boost::thread::interrupt(), or return
     * within io_timeout after close() has been called.
     *
     * @param message  the message to write.
     * @throw Exception with cause
     *         ILLEGAL_ARGUMENT if the message contains invalid or unsupported
     *                 values (e.g., the device doesn't support extended frames).
     *         IO_ERROR if an I/O error occurred while trying to write the message;
     *         DEVICE_UNAVAILABLE if the device does not respond, or has been
     *                 removed from the system;
     *         DEVICE_CLOSED if the device is not open.
     * @throw IllegalArgumentException  if the message is invalid (e.g., contains more than 8 bytes of payload).
     */
    virtual void write(const tug_can_msgs::CanMessageConstPtr & message) = 0;
};

}

#endif
