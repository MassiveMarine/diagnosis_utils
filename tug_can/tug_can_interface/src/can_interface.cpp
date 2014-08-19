#include <tug_can_interface/can_interface.h>
#include <boost/make_shared.hpp>

namespace tug_can_interface
{

CanInterface::Exception::Exception(const std::string & what_arg)
    : std::runtime_error(what_arg)
{
}

CanInterface::~CanInterface()
{
}

void CanInterface::sendMessage(uint32_t id, bool rtr, bool extended,
                 const std::vector<uint8_t> & data)
{
    tug_can_msgs::CanMessagePtr can_message = createMessage(id, rtr, extended);
    can_message->data = data;
    sendMessage(can_message);
}

void CanInterface::sendMessage(uint32_t id, bool rtr, bool extended,
                 const std::vector<uint8_t> & data, size_t start_index, size_t size)
{
    tug_can_msgs::CanMessagePtr can_message = createMessage(id, rtr, extended);
    if (start_index < data.size())
    {
      if ((start_index + size) <= data.size())
      {
        can_message->data.assign(data.begin() + start_index,
            data.begin() + start_index + size);
      }
      else
        can_message->data.assign(data.begin() + start_index, data.end());
    }
    sendMessage(can_message);
}

void CanInterface::sendMessage(uint32_t id, bool rtr, bool extended, const uint8_t * data,
                 size_t size)
{
    tug_can_msgs::CanMessagePtr can_message = createMessage(id, rtr, extended);
    can_message->data.assign(data, data + size);
    sendMessage(can_message);
}

CanSubscriptionPtr CanInterface::subscribe(uint32_t id, const MessageCallback & callback)
{
    return subscribe(std::vector<uint32_t>(1, id), callback);
}

tug_can_msgs::CanMessagePtr CanInterface::createMessage(uint32_t id, bool rtr, bool extended)
{
  tug_can_msgs::CanMessagePtr can_message = boost::make_shared<tug_can_msgs::CanMessage>();
  can_message->time = ros::Time::now();
  can_message->id = id;
  can_message->rtr = rtr ? 1 : 0;
  can_message->extended = extended ? 1 : 0;
  return can_message;
}

void CanInterface::checkMessage(const tug_can_msgs::CanMessageConstPtr & can_message)
{
    if (can_message->id >= (1U << 29) || (!can_message->extended && can_message->id >= (1U << 11)))
        throw std::invalid_argument("CanInterface::sendMessage: id is out of bounds");
    if (can_message->data.size() > 8)
        throw std::invalid_argument("CanInterface::sendMessage: data is longer than 8 bytes");
}


}
