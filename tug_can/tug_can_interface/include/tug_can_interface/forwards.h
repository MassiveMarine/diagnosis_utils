#ifndef TUG_CAN_INTERFACE__FORWARDS_H_
#define TUG_CAN_INTERFACE__FORWARDS_H_

#include <boost/shared_ptr.hpp>

namespace tug_can_interface
{

class CanSubscription;
typedef boost::shared_ptr<CanSubscription> CanSubscriptionPtr;

class CanClient;
typedef boost::shared_ptr<CanClient> CanClientPtr;

class CanInterface;
typedef boost::shared_ptr<CanInterface> CanInterfacePtr;

class CanNicDriver;
typedef boost::shared_ptr<CanNicDriver> CanNicDriverPtr;

class CanNicDriverHost;
typedef boost::shared_ptr<CanNicDriverHost> CanNicDriverHostPtr;

class CanServer;
typedef boost::shared_ptr<CanServer> CanServerPtr;

}

#endif
