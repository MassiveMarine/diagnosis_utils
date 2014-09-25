#ifndef TUG_CAN_NIC_DRIVERS__SOCKET_CAN_NIC_DRIVER_H_
#define TUG_CAN_NIC_DRIVERS__SOCKET_CAN_NIC_DRIVER_H_

#include <tug_can_interface/can_nic_driver.h>
#include <boost/thread/mutex.hpp>

namespace tug_can_nic_drivers
{

class SocketCanNicDriver : public tug_can_interface::CanNicDriver
{
public:
    SocketCanNicDriver();

    virtual ~SocketCanNicDriver();

    /// Supports "*" as a device name, opening all available CAN interfaces.
    virtual void open(const std::string & device_name, int baud_rate,
                      const ros::Duration & io_timeout,
                      const ros::NodeHandle & node_handle);

    virtual void close();

    virtual void read(const tug_can_msgs::CanMessagePtr & message);

    virtual void write(const tug_can_msgs::CanMessageConstPtr & message);

private:
    enum State
    {
        STATE_CLOSED,
        STATE_OPEN,
        STATE_CLOSING
    };

    void ensureOpen();
    void closeSocket();
    void checkErrno(const char * operation);

    boost::mutex mutex_;
    State state_;
    int io_timeout_;

    boost::mutex read_mutex_;
    boost::mutex write_mutex_;
    int socket_;
};

}

#endif
