//
// Created by clemens on 24.09.15.
//

#include <tug_time/Timer.h>
#include <ros/ros.h>

Timer::Timer() : wait_period_(100 * 1000)
{ }

Timer::Timer(boost::posix_time::microseconds wait_period, boost::function<void()> call_back) : wait_period_(wait_period), call_back_(call_back)
{
  background_thread_ = boost::thread(boost::bind(&Timer::run, this));
}

Timer& Timer::operator=(Timer& other)
{
  wait_period_ = other.wait_period_;
  call_back_ = other.call_back_;
  background_thread_ = boost::thread(boost::bind(&Timer::run, this));
  return *this;
}

void Timer::run()
{
  boost::asio::io_service io_service;
  boost::asio::deadline_timer timer(io_service);

  while(ros::ok())
  {
    timer.expires_from_now(wait_period_);
    call_back_();
    timer.wait();
  }
}