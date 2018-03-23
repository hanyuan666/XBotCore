#include <XBotCore-interfaces/XDomainCommunication.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <string>
#include <XBotIDDPExample.h>
#include <XBotCore/XBotCore.h>


XBot::XBotIDDPExample::XBotIDDPExample(std::string path_to_config) : _path_to_config(path_to_config)
{
  
    int period = 1000;
      
    set_thread_name("XBOTIDDP");
    // set thread period - not periodic
    task_period_t t;
    memset(&t, 0, sizeof(t));
    t.period = {0,period};
    set_thread_period(t);
    
    // set thread priority
    set_thread_priority();
    
    // initialize ipc handler
    _ipc_handler = std::make_shared<XBot::XBotIDDP>(_path_to_config);

}

void XBot::XBotIDDPExample::set_thread_name(std::string thread_name)
{
    // save the thread name
    this->thread_name = thread_name;
    // set thread name
    name = this->thread_name.c_str();
}

std::string XBot::XBotIDDPExample::get_thread_name(void)
{
    return thread_name;
}

void XBot::XBotIDDPExample::set_thread_period(task_period_t t)
{
    period.task_time = t.task_time;
    period.period = t.period;
}

void XBot::XBotIDDPExample::set_thread_priority()
{

    // set scheduler policy
#if defined( __XENO__ ) || defined( __COBALT__ )
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    
    // set scheduler priority and stacksize
    priority = sched_get_priority_max(schedpolicy);
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
}

void XBot::XBotIDDPExample::th_init( void * ){
    _ipc_handler->init();

    // generate IDDP robot
    XBot::AnyMapPtr anymap = std::make_shared<XBot::AnyMap>();
    std::shared_ptr<XBot::IXBotJoint> xbot_joint = _ipc_handler;
    std::shared_ptr<XBot::IXBotFT> xbot_ft = _ipc_handler;
    std::shared_ptr<XBot::IXBotIMU> xbot_imu = _ipc_handler;
    std::shared_ptr<XBot::IXBotHand> xbot_hand = _ipc_handler;

    (*anymap)["XBotJoint"] = boost::any(xbot_joint);
    (*anymap)["XBotFT"] = boost::any(xbot_ft);
    (*anymap)["XBotIMU"] = boost::any(xbot_imu);
    (*anymap)["XBotHand"] = boost::any(xbot_hand);
    (*anymap)["EnableReferenceReading"] = boost::any(true); // TBD check it!

    _robot = XBot::RobotInterface::getRobot(_path_to_config, "iddp_robot", anymap, "XBotRT");
     
    sleep(15);
    
    // read from IPC
     _ipc_handler->updateRX();
     
    // update robot
    _robot->sense();
    
    
    
    
    
    _first_loop_time = 0;
    _time = 0;
    _homing_time = 50;
    _robot->getRobotState("home_2", _q_home);
    XBot::Logger::warning() << _q_home << XBot::Logger::endl();
    _robot->getJointPosition(_q0);
    XBot::Logger::warning() << _q0 << XBot::Logger::endl();
    
    
    XBot::Logger::warning() << "INIT completed!" << XBot::Logger::endl();

}

void XBot::XBotIDDPExample::th_loop( void * ){ 
    
    /* Update IPC (receive from RT) */
     _ipc_handler->updateRX();

    /* Read robot state from RT layer and update robot */
    _robot->sense(false);
    
    /////////////////////////////////////////////
    // SPEAKS WITH THE OTHER OROCOS COMPONENTS //
    /////////////////////////////////////////////
// // /*    
// // 
// //     
// //     
// //     // go to homing
// //     if( (_time - _first_loop_time) <= _homing_time ) {
// //         _q = _q0 + 0.5*(1-std::cos(3.1415*(_time - _first_loop_time)/_homing_time))*(_q_home-_q0);
// //         _robot->setPositionReference(_q);
// // //         _robot->move();
// // //         return;
// //             XBot::Logger::warning() << "I am alive! " << _q << XBot::Logger::endl();
// //     }
// //     _time+=0.005;
// //     
// //     
// //      XBot::Logger::error() << "OUT OF HOMING! " << _time << XBot::Logger::endl();*/
    
    
    /* Send received commands to the RT layer (XBotCommunicationRT plugin) */
    _robot->move();
    
    /* Update IPC (send to RT) */
    _ipc_handler->updateTX();
    
}

XBot::XBotIDDPExample::~XBotIDDPExample() {
    
    Logger::info() << "~XBotIDDPExample()" << Logger::endl();
}
