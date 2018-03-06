#include <XBotCore-interfaces/XDomainCommunication.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <string>
#include <IDDPExample1.h>
#include <XBotCore/XBotCore.h>


XBot::IDDPExample1::IDDPExample1() : 
  sub_vec("rt2_to_rt1_vec3d")
  
{
  
    int period = 1000;
      
    set_thread_name("IDDP1");
    // set thread period - not periodic
    task_period_t t;
    memset(&t, 0, sizeof(t));
    t.period = {0,period};
    set_thread_period(t);
    
    // set thread priority
    set_thread_priority();
    
   
}

void XBot::IDDPExample1::set_thread_name(std::string thread_name)
{
    // save the thread name
    this->thread_name = thread_name;
    // set thread name
    name = this->thread_name.c_str();
}

std::string XBot::IDDPExample1::get_thread_name(void)
{
    return thread_name;
}

void XBot::IDDPExample1::set_thread_period(task_period_t t)
{
    period.task_time = t.task_time;
    period.period = t.period;
}

void XBot::IDDPExample1::set_thread_priority()
{

    // set scheduler policy
#if defined( __XENO__ ) || defined( __COBALT__ )
    schedpolicy = SCHED_FIFO;
    XBot::Logger::warning() << "SCHED_FIFO for IDDPExample1 " <<  XBot::Logger::endl();
#else
    schedpolicy = SCHED_OTHER;
    XBot::Logger::warning() << "SCHED_OTHER for IDDPExample1 " <<  XBot::Logger::endl();
#endif
    
    // set scheduler priority and stacksize
    priority = sched_get_priority_max(schedpolicy);
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
}

void XBot::IDDPExample1::th_init( void * ){
    
}

void XBot::IDDPExample1::th_loop( void * ){  
    it++;

    if(sub_vec.read(vec2)){
        XBot::Logger::warning() << "RT 1 process: received vector " << vec2.transpose() << XBot::Logger::endl();
    }

   
}

XBot::IDDPExample1::~IDDPExample1() {
    
    Logger::info() << "~IDDPExample1()" << Logger::endl();
}
