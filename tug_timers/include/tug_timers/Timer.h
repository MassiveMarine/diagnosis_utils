//
// Created by clemens on 24.09.15.
//

#ifndef TUG_TIMERS_TIMER_H
#define TUG_TIMERS_TIMER_H

#include <boost/thread.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/function.hpp>

class Timer
{
    boost::thread background_thread_;
    boost::posix_time::microseconds wait_period_;
    boost::function<void()> call_back_;

public:
    Timer();
    Timer(boost::posix_time::microseconds wait_period, boost::function<void()> call_back);
    Timer& operator=(Timer& other);
    void run();
};

#endif //TUG_TIMERS_TIMER_H
