#include <tug_can_interface/can_nic_driver.h>

namespace tug_can_interface
{

CanNicDriver::Exception::Exception(Cause cause, const std::string & message)
    : runtime_error(message), cause_(cause)
{
}

const char * CanNicDriver::Exception::what() const throw()
{
    return ("[" + getCauseName(cause_) + "] " + this->runtime_error::what()).c_str();
}

std::string CanNicDriver::Exception::getMessage() const
{
    return this->runtime_error::what();
}

CanNicDriver::Exception::Cause CanNicDriver::Exception::getCause() const
{
    return cause_;
}

std::string CanNicDriver::Exception::getCauseName(Cause cause)
{
    switch (cause)
    {
    case ILLEGAL_ARGUMENT: return "ILLEGAL_ARGUMENT";
    case ILLEGAL_STATE: return "ILLEGAL_STATE";
    case DEVICE_CLOSED: return "DEVICE_CLOSED";
    case DEVICE_UNAVAILABLE: return "DEVICE_UNAVAILABLE";
    case BUS_ERROR: return "BUS_ERROR";
    case IO_ERROR: return "IO_ERROR";
    }
    return "__INVALID_CAUSE__";
}

CanNicDriver::CanNicDriver()
{
}

CanNicDriver::~CanNicDriver()
{
}

}
