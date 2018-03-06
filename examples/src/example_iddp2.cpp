#include <XBotCore-interfaces/XDomainCommunication.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <string>
#include <IDDPExample2.h>
#include <XBotCore/XBotCore.h>


XBot::IDDPExample2::IDDPExample2() : 
  pub_vec("rt2_to_rt1_vec3d")
  
{
  
    int period = 1000;
      
    set_thread_name("IDDP2");
    // set thread period - not periodic
    task_period_t t;
    memset(&t, 0, sizeof(t));
    t.period = {0,period};
    set_thread_period(t);
    
    // set thread priority
    set_thread_priority();
    
   
}

void XBot::IDDPExample2::set_thread_name(std::string thread_name)
{
    // save the thread name
    this->thread_name = thread_name;
    // set thread name
    name = this->thread_name.c_str();
}

std::string XBot::IDDPExample2::get_thread_name(void)
{
    return thread_name;
}

void XBot::IDDPExample2::set_thread_period(task_period_t t)
{
    period.task_time = t.task_time;
    period.period = t.period;
}

void XBot::IDDPExample2::set_thread_priority()
{
    // set scheduler policy
#if defined( __XENO__ ) || defined( __COBALT__ )
    schedpolicy = SCHED_FIFO;
    XBot::Logger::warning() << "SCHED_FIFO for IDDPExample2 " <<  XBot::Logger::endl();
#else
    schedpolicy = SCHED_OTHER;
    XBot::Logger::warning() << "SCHED_FIFO for IDDPExample2 " <<  XBot::Logger::endl();
#endif
    
    // set scheduler priority and stacksize
    priority = sched_get_priority_max(schedpolicy);
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
}

void XBot::IDDPExample2::th_init( void * ){

}

void XBot::IDDPExample2::th_loop( void * ){  
    it++;
    
//     XBot::Logger::warning() << "RT 2 looping..." << XBot::Logger::endl();

    vec1.setConstant(it);
    pub_vec.write(vec1);

}

XBot::IDDPExample2::~IDDPExample2() {
    
    Logger::info() << "~IDDPExample2()" << Logger::endl();
}
