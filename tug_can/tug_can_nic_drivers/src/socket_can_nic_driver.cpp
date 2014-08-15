#include <tug_can_nic_drivers/socket_can_nic_driver.h>
#include <errno.h>
#include <limits.h>
#include <sstream>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>

namespace tug_can_nic_drivers
{

SocketCanNicDriver::SocketCanNicDriver()
    : socket_(-1)
{
}

SocketCanNicDriver::~SocketCanNicDriver()
{
    close();
}

void SocketCanNicDriver::open(const std::string & device_name, int baud_rate,
                  const ros::Duration & io_timeout,
                  ros::NodeHandle & node_handle)
{
    if (socket_ >= 0)
        throw Exception(Exception::ILLEGAL_STATE, "Tried to open CAN device being already open");

    if (baud_rate != 0)
        throw Exception(Exception::ILLEGAL_ARGUMENT, "baud_rate must be 0");

    double io_timeout_in_usecs = io_timeout.toSec() * 1000000.0;
    if (io_timeout_in_usecs < 0 || io_timeout_in_usecs > static_cast<double>(INT_MAX))
        throw Exception(Exception::ILLEGAL_ARGUMENT, "io_timeout is out of range");
    io_timeout_ = static_cast<int>(io_timeout_in_usecs);

    struct ifreq ifr;
    if (device_name.size() >= sizeof(ifr.ifr_name))
        throw Exception(Exception::ILLEGAL_ARGUMENT, "device_name is too long");

    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0)
    {
        std::ostringstream message;
        message << "Seems like the kernel does not support SocketCAN (errno: " << errno << ")";
        throw Exception(Exception::DEVICE_UNAVAILABLE, message.str());
    }

    // Enable reception of error information:
    can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT | CAN_ERR_CRTL | CAN_ERR_PROT | CAN_ERR_ACK | CAN_ERR_BUSOFF | CAN_ERR_BUSERROR;
    if (setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) != 0)
        ROS_WARN_STREAM("Failed to set SocketCAN error mask (errno: " << errno << ")");

    // Enable reception of own sent messages:
    int recv_own_msgs = 1;
    if (setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs)) != 0)
        ROS_WARN_STREAM("Failed to enable reception of sent frames (errno: " << errno << ")");

    if (device_name == "*")
    {
        // Bind to all available interfaces:
        ifr.ifr_ifindex = 0;
    }
    else
    {
        // Find index of CAN interface name:
        strcpy(ifr.ifr_name, device_name.c_str());
        if (ioctl(socket_, SIOCGIFINDEX, &ifr) == -1)
        {
            std::ostringstream message;
            message << "Could not find index of CAN interface named '" <<
                       device_name << "' (errno: " << errno << ")";
            throw Exception(Exception::DEVICE_UNAVAILABLE, message.str());
        }
    }

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) != 0)
    {
        std::ostringstream message;
        message << "Could not bind to CAN interface (errno: " << errno << ")";
        throw Exception(Exception::DEVICE_UNAVAILABLE, message.str());
    }
}

void SocketCanNicDriver::close()
{
    if (socket_ >= 0)
    {
        if (shutdown(socket_, SHUT_RDWR) != 0)
            ROS_WARN_STREAM("Error while trying to close CAN device (errno: " << errno << ")");
        socket_ = -1;
    }
}

void SocketCanNicDriver::read(const tug_can_msgs::CanMessagePtr & message)
{
    while (true)
    {
        // Check if socket has been closed in the meantime:
        if (socket_ < 0)
            throw Exception(Exception::DEVICE_CLOSED, "Called read with no device open");

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(socket_, &fds);

        struct timeval timeout;
        timeout.tv_sec = io_timeout_ / 1000000;
        timeout.tv_usec = io_timeout_ % 1000000;

        if (select(socket_ + 1, &fds, 0, 0, &timeout) < 0)
        {
            if (errno == EBADF)
            {
                shutdown(socket_, SHUT_RDWR);
                socket_ = -1;
                throw Exception(Exception::DEVICE_CLOSED, "select() said the socket is bad");
            }
            else
            {
                std::ostringstream message;
                message << "select() failed (errno: " << errno << ")";
                throw Exception(Exception::IO_ERROR, message.str());
            }
        }
        else if (FD_ISSET(socket_, &fds))
        {
            struct can_frame frame;
            ssize_t read_bytes = ::read(socket_, &frame, sizeof(frame));
            if (read_bytes < 0)
            {
                std::ostringstream message;
                message << "read() failed (errno: " << errno << ")";
                throw Exception(Exception::IO_ERROR, message.str());
            }
            else if (read_bytes != sizeof(frame))
                throw Exception(Exception::IO_ERROR, "read() returned invalid number of bytes");

            if (frame.can_id & CAN_ERR_FLAG)
                throw Exception(Exception::BUS_ERROR, "CAN device reported a bus error"); // TODO: more specific error message

            message->time = ros::Time::now();
            message->id = frame.can_id & CAN_ERR_MASK;
            message->extended = (frame.can_id & CAN_EFF_FLAG) ? 1 : 0;
            message->rtr = (frame.can_id & CAN_RTR_FLAG) ? 1 : 0;
            message->data.assign(&frame.data[0], &frame.data[frame.can_dlc]);
        }
        else
        {
            // Timeout => try again
        }
    }
}

void SocketCanNicDriver::write(const tug_can_msgs::CanMessageConstPtr & message)
{
    // TODO
}

}
