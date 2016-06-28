/*
This file is part of the software provided by the tug ais groupe
Copyright (c) 2015, Clemens Muehlbacher, Stefan Loigge
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <tug_time/Timeout.h>
#include <ros/ros.h>
#include <boost/date_time/date.hpp>

Timeout::Timeout(boost::function<bool()> timeout_call_back) : call_back_(timeout_call_back), pause_thread_(true)
{
  background_thread_ = boost::thread(boost::bind(&Timeout::run, this));
}

Timeout::Timeout(boost::posix_time::time_duration timeout, boost::function<bool()> timeout_call_back) :
        timeout_(timeout), call_back_(timeout_call_back), pause_thread_(false)
{
    background_thread_ = boost::thread(boost::bind(&Timeout::run, this));
}

Timeout::Timeout(double timeout, boost::function<bool()> timeout_call_back) :
        call_back_(timeout_call_back), pause_thread_(false)
{
  double seconds = static_cast<double>(static_cast<int64_t>(timeout));
  double milli_seconds = (timeout - seconds) * 1000.;

  timeout_ = boost::posix_time::seconds(static_cast<int64_t>(seconds)) +
    boost::posix_time::milliseconds(static_cast<int64_t>(milli_seconds));
  background_thread_ = boost::thread(boost::bind(&Timeout::run, this));
}

void Timeout::run()
{
    while (ros::ok())
    {
        boost::mutex::scoped_lock lock(the_mutex_);

        if (pause_thread_)
            the_condition_.wait(lock);
        else if (!the_condition_.timed_wait(lock, timeout_))
            pause_thread_ = !call_back_();
    }
}

void Timeout::set()
{
    boost::mutex::scoped_lock lock(the_mutex_);
    pause_thread_ = false;
    the_condition_.notify_one();
}

void Timeout::stop()
{
    boost::mutex::scoped_lock lock(the_mutex_);
    pause_thread_ = true;
    the_condition_.notify_one();
}

void Timeout::setTimeOut(boost::posix_time::time_duration timeout)
{
  boost::mutex::scoped_lock lock(the_mutex_);
  timeout_ = timeout;

  pause_thread_ = false;
  the_condition_.notify_one();
}

void Timeout::setTimeOut(double timeout)
{
  boost::mutex::scoped_lock lock(the_mutex_);
  double seconds = static_cast<double>(static_cast<int64_t>(timeout));
  double milli_seconds = (timeout - seconds) * 1000.;

  ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ":" << __LINE__ <<
    " seconds:" << seconds << " and milli seconds" << milli_seconds);

  timeout_ = boost::posix_time::seconds(static_cast<int64_t>(seconds)) +
    boost::posix_time::milliseconds(static_cast<int64_t>(milli_seconds));

  pause_thread_ = false;
  the_condition_.notify_one();
}
