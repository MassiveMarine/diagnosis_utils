
#ifndef TUG_TIME_TIMEOUT_H
#define TUG_TIME_TIMEOUT_H

#include <boost/function.hpp>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

class Timeout
{
    boost::thread background_thread_;
    boost::mutex the_mutex_;
    boost::condition_variable the_condition_;
    boost::posix_time::time_duration timeout_;
    boost::function<bool()> call_back_;
    bool pause_thread_;

public:
    Timeout(boost::posix_time::time_duration timeout, boost::function<bool()> timeout_call_back);
    void run();
    void set();

};

#endif //TUG_TIME_TIMEOUT_H
