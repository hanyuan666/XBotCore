/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Giuseppe Rigano
 * email:  giuseppe.rigano@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/


#ifndef __X_BOT_IDDP_EXAMPLE_H__
#define __X_BOT_IDDP_EXAMPLE_H__

#include <XCM/XBotUtils.h>
#include <XCM/XBotThread.h>

#include <XBotCore-interfaces/XDomainCommunication.h>

#include <XCM/XBotIPC.h>
#include <XCM/XBotIDDP.h>

#include <XBotInterface/RobotInterface.h>

#include <memory>
#include <eigen3/Eigen/Dense>


namespace XBot
{
    class XBotIDDPExample;  
}

class XBot::XBotIDDPExample : public XBot::Thread_hook
                        
{
public:
    
    XBotIDDPExample(std::string path_to_config);
    virtual ~XBotIDDPExample();    
    
    virtual void th_init ( void * );
    virtual void th_loop ( void * );
        
    /**
     * @brief Getter for the thread name
     * 
     * @param  void
     * @return std::string the thread name
     */
    std::string get_thread_name(void);

private:
    
    // IPC handler
    XBot::XBotIPC::Ptr _ipc_handler;
    // IPC robot
    XBot::RobotInterface::Ptr _robot;
    // config 
    std::string _path_to_config;
    
    
    
    double _time, _homing_time, _first_loop_time;
    Eigen::VectorXd _q0, _q_home, _q, _k, _d, _k0, _d0, _qref;
    
    
    
    /**
     * @brief The thread name
     * 
     */
    std::string thread_name;
    
    /**
     * @brief Setter for the thread name
     * 
     * @param  std::string the thread name
     * @return void
     */
    void set_thread_name(std::string);
        
    /**
     * @brief Setter for the thread period
     * 
     * @param  t the task period
     * @return void
     */
    void set_thread_period(task_period_t t);
    
    /**
     * @brief Setter for the thread priority: RT thread
     * 
     * @param void
     * @return void
     */
    void set_thread_priority();   

};

#endif //__X_BOT_IDDP_EXAMPLE_1_H__
