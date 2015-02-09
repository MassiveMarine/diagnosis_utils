#include <tug_can_interface/can_nic_driver_host.h>
#include <limits.h>
#include <sstream>
#include <boost/make_shared.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <tug_can_interface/can_nic_driver.h>
#include <tug_can_interface/can_subscription.h>

namespace tug_can_interface
{

CanNicDriverHost::CanNicDriverHost(const std::string & driver_plugin_name,
                                   const std::string & device_name, int baud_rate,
                                   const ros::Duration & io_timeout,
                                   const ros::NodeHandle & node_handle)
    : device_name_(device_name), baud_rate_(baud_rate), io_timeout_(io_timeout),
      node_handle_(node_handle), running_(true), dump_messages_enabled_(false)
{
    try
    {
        driver_class_loader_ = boost::make_shared<DriverClassLoader>("tug_can_interface", "tug_can_interface::CanNicDriver");
        driver_ = driver_class_loader_->createInstance(driver_plugin_name);
        driver_->open(device_name_, baud_rate_, io_timeout_, node_handle_);
        read_thread_ = boost::thread(&CanNicDriverHost::readAndDispatchMessages, this);
    }
    catch (pluginlib::PluginlibException & ex)
    {
        // We assume that loading fails because of a misspelled class name:
        driver_class_loader_.reset();
        throw std::invalid_argument(ex.what());
    }
    catch (CanNicDriver::Exception & ex)
    {
        // If the problem is an invalid value, fail; else, try again repeatedly
        // in the background thread:
        if (ex.getCause() == CanNicDriver::Exception::ILLEGAL_ARGUMENT)
        {
            driver_.reset();
            driver_class_loader_.reset();
            throw std::invalid_argument(ex.getMessage());
        }
    }
    catch (...)
    {
        if (driver_)
            driver_->close();
        driver_.reset();
        driver_class_loader_.reset();
        throw;
    }
}

CanNicDriverHost::~CanNicDriverHost()
{
    //std::cerr << "CanNicDriverHost: Shutting down CAN read thread" << std::endl;
    {
        boost::lock_guard<boost::mutex> lock(mutex_);
        if (driver_)
            driver_->close();
        running_ = false;
    }

    if (read_thread_.joinable())
    {
        read_thread_.interrupt();
        read_thread_.join();
    }
    //std::cerr << "CanNicDriverHost: Shutdown complete." << std::endl;
}

int CanNicDriverHost::parseBaudRate(const std::string & baud_rate_string)
{
    std::istringstream s(baud_rate_string);
    unsigned int numeric_value;

    s >> numeric_value;
    if (s.fail())
        throw std::invalid_argument("Baud rate does not start with a number");

    unsigned int factor = 1;
    if (!s.eof())
    {
        std::string factor_string;
        s >> factor_string;
        if (s.fail())
            throw std::invalid_argument("Baud rate contains invalid content after the number");

        if (factor_string == "k")
            factor = 1000;
        else if (factor_string == "M")
            factor = 1000000;
        else
            throw std::invalid_argument("Baud rate contains invalid unit prefix; only 'k' and 'M' are supported");
    }

    // Check for overflow:
    unsigned int result = numeric_value * factor;
    if (result < numeric_value || result > INT_MAX)
        throw std::invalid_argument("Baud rate exceeds numeric limits");

    return static_cast<int>(result);
}

void CanNicDriverHost::dumpMessage(std::ostream & dump, const tug_can_msgs::CanMessageConstPtr & can_message)
{
    dump << "id: " << can_message->id;
    if (can_message->extended)
        dump << " EXT";
    if (can_message->rtr)
        dump << " RTR";
    dump << ", data: [" << std::hex;
    for (tug_can_msgs::CanMessage::_data_type::const_iterator it = can_message->data.begin(); it != can_message->data.end(); ++it)
        dump << " 0x" << std::setw(2) << std::setfill('0') << static_cast<unsigned>(*it);
    dump << " ]";
}

void CanNicDriverHost::setDumpMessagesEnabled(bool dump_messages_enabled)
{
    boost::lock_guard<boost::mutex> lock1(mutex_);
    boost::lock_guard<boost::recursive_mutex> lock2(subscriptions_mutex_);
    dump_messages_enabled_ = dump_messages_enabled;
}


void CanNicDriverHost::sendMessage(const tug_can_msgs::CanMessageConstPtr & can_message)
{
    checkMessage(can_message);

    boost::lock_guard<boost::mutex> lock(mutex_);
    if (dump_messages_enabled_)
    {
        std::ostringstream dump;
        dumpMessage(dump, can_message);
        ROS_INFO_STREAM("Sending CAN message: " << dump.str());
    }
    if (running_ && driver_)
    {
        try
        {
            driver_->write(can_message);
        }
        catch (std::exception & ex)
        {
            ROS_ERROR("Writing to CAN driver failed: %s", ex.what());
        }
    }
    else
    {
        ROS_ERROR("Called CanInterface::sendMessage while driver not loaded -- something is very wrong here");
    }
}

CanSubscriptionPtr CanNicDriverHost::subscribe(const std::vector<uint32_t> & ids,
                                  const MessageCallback & callback)
{
    boost::lock_guard<boost::recursive_mutex> lock(subscriptions_mutex_);
    SubscriptionImplPtr subscription = boost::make_shared<SubscriptionImpl>(callback);
    for (std::vector<uint32_t>::const_iterator it = ids.begin(); it != ids.end(); ++it)
        subscriptions_[*it].push_back(subscription);
    return subscription;
}

CanSubscriptionPtr CanNicDriverHost::subscribeToAll(const MessageCallback & callback)
{
    boost::lock_guard<boost::recursive_mutex> lock(subscriptions_mutex_);
    SubscriptionImplPtr subscription = boost::make_shared<SubscriptionImpl>(callback);
    subscriptions_to_all_.push_back(subscription);
    return subscription;
}

CanNicDriverHost::SubscriptionImpl::SubscriptionImpl(const MessageCallback & callback)
    : callback_(callback)
{
}

CanNicDriverHost::SubscriptionImpl::~SubscriptionImpl()
{
}

void CanNicDriverHost::readAndDispatchMessages()
{
    //std::cerr << "CAN read thread started" << std::endl;
    tug_can_msgs::CanMessagePtr can_message = boost::make_shared<tug_can_msgs::CanMessage>();
    while (true)
    {
        {
            boost::lock_guard<boost::mutex> lock(mutex_);
            if (!running_)
                break;
        }

        try
        {
            boost::this_thread::interruption_point();

            driver_->read(can_message);
            dispatchMessage(can_message);
            can_message = boost::make_shared<tug_can_msgs::CanMessage>();
        }
        catch (CanNicDriver::Exception & ex)
        {
            // If we're shutting down, the exception probably came from that.
            {
                boost::lock_guard<boost::mutex> lock(mutex_);
                if (!running_)
                    break;
            }

            switch (ex.getCause())
            {
            case CanNicDriver::Exception::IO_ERROR:
                ROS_ERROR("I/O error while reading from CAN device: %s", ex.what());
                break;

            case CanNicDriver::Exception::DEVICE_CLOSED:
            case CanNicDriver::Exception::DEVICE_UNAVAILABLE:
                std::cerr << "CAN device closed or unavailable; trying to open it again after a short time" << std::endl;
                driver_->close();

                boost::this_thread::sleep(boost::posix_time::seconds(1));
                try
                {
                    driver_->open(device_name_, baud_rate_, io_timeout_, node_handle_);
                }
                catch (CanInterface::Exception & ex)
                {
                    ROS_ERROR("Re-opening CAN device failed: %s", ex.what());
                }
                break;

            case CanNicDriver::Exception::BUS_ERROR:
                ROS_WARN_STREAM("CAN device reported bus error: " << ex.getMessage());
                break;

            default:
                ROS_ERROR("Reading from CAN device failed: %s", ex.what());
                break;
            }
        }
        catch (boost::thread_interrupted &)
        {
            // Ignored; running_ should be false if the interruption happened
            // because of destruction, thus ending the loop.
        }
        catch (std::exception & ex)
        {
            ROS_ERROR("Exception while reading from CAN device: %s", ex.what());
        }
        catch (...)
        {
            ROS_ERROR("Exception while reading from CAN device.");
        }
    }
    //std::cerr << "CAN read thread finished" << std::endl;
}

void CanNicDriverHost::dispatchMessage(const tug_can_msgs::CanMessageConstPtr & can_message)
{
    boost::lock_guard<boost::recursive_mutex> lock(subscriptions_mutex_);
    if (dump_messages_enabled_)
    {
        std::ostringstream dump;
        dumpMessage(dump, can_message);
        ROS_INFO_STREAM("Received CAN message: " << dump.str());
    }
    try
    {
        dispatchMessage(subscriptions_to_all_, can_message);
        SubscriptionMap::iterator it = subscriptions_.find(can_message->id);
        if (it != subscriptions_.end())
            dispatchMessage(it->second, can_message);
    }
    catch (std::exception & ex)
    {
        ROS_ERROR("Dispatching CAN message failed: %s", ex.what());
    }
}

void CanNicDriverHost::dispatchMessage(SubscriptionVector & subscription_vector, const tug_can_msgs::CanMessageConstPtr & can_message)
{
    // Using index-based access because vector may grow during callbacks:
    size_t i = 0;
    while (i < subscription_vector.size())
    {
        SubscriptionImplPtr subscription = subscription_vector[i].lock();
        if (subscription)
        {
            try
            {
                subscription->callback_(can_message);
            }
            catch (std::exception & ex)
            {
                ROS_ERROR("CAN message callback threw exception: %s", ex.what());
            }
            catch (...)
            {
                ROS_ERROR("CAN message callback threw exception");
            }

            ++i;
        }
        else
        {
            // Weak pointer expired, so remove it from the list:
            subscription_vector.erase(subscription_vector.begin() + i);
        }
    }
}


}
