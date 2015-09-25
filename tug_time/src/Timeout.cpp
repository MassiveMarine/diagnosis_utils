
#include <tug_time/Timeout.h>
#include <ros/ros.h>

Timeout::Timeout(boost::posix_time::time_duration timeout, boost::function<bool()> timeout_call_back) :
        timeout_(timeout),
        call_back_(timeout_call_back),
        pause_thread_(false)
{
    background_thread_ = boost::thread(boost::bind(&Timeout::run, this));
}

void Timeout::run()
{
    while (ros::ok())
    {
        boost::mutex::scoped_lock lock(the_mutex_);

        if(pause_thread_)
            the_condition_.wait(lock);
        else if(!the_condition_.timed_wait(lock,timeout_))
            pause_thread_ = !call_back_();
    }
}

void Timeout::set()
{
    boost::mutex::scoped_lock lock(the_mutex_);
    pause_thread_ = false;
    the_condition_.notify_one();

}

//void timeout_cb()
//{
//    ROS_INFO_STREAM("Timeout");
//}
//
//int main(int argc, char** argv)
//{
//    ros::init(argc, argv, "tug_timeout_test_node");
//    ROS_WARN_STREAM("Hallo");
//    boost::shared_ptr<Timeout> timeout = boost::make_shared<Timeout>(boost::posix_time::seconds(2), timeout_cb);
//
//    while (ros::ok())
//    {
//        sleep(3);
//        timeout->set();
//    }
//}