/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Arturo Laurenzi, Luca Muratore
 * email:  arturo.laurenzi@iit.it, luca.muratore@iit.it
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

#ifndef __X_BOT_RT_COMMUNICATION_H__
#define __X_BOT_RT_COMMUNICATION_H__

#include <XCM/XBotControlPlugin.h>
#include <XCM/XBotESCUtils.h>
#include <XBotCore-interfaces/XBotESC.h>
#include <XBotCore-interfaces/XDomainCommunication.h>

#include <XBotInterface/Utils.h>

#include <memory>

namespace XBot
{
    class XBotRTCommunication;
}

/**
 * @brief XBotCore RT plugin that communicates with the RT enviroment using IDDP pipes
 *
 */
class XBot::XBotRTCommunication : public XBot::XBotControlPlugin
{
public:
    XBotRTCommunication();

    virtual bool init_control_plugin(XBot::Handle::Ptr handle);

    virtual void on_start(double time);

    virtual void on_stop(double time);

    virtual bool close();

    virtual ~XBotRTCommunication();

protected:

    virtual void control_loop(double time, double period);

private:

    double _start_time;

    XBot::RobotInterface::Ptr _robot;
    
    XBot::ESCUtils::Ptr _esc_utils;

    XBot::Utils::SecondOrderFilter<Eigen::VectorXd> _filter_q, _filter_k, _filter_d, _filter_qdot;

    bool _filter_enabled;

    std::map<int,XBot::PublisherIDDP<XBot::RobotState>> _pub_read_from_hal;
    std::map<int,XBot::SubscriberIDDP<XBot::RobotState::pdo_tx>> _sub_write_to_hal;
    std::map<int,XBot::PublisherIDDP<XBot::RobotFT::pdo_rx>> _pub_ft_read_from_hal;
    std::map<int,XBot::PublisherIDDP<XBot::RobotIMU::pdo_rx>> _pub_imu_read_from_hal;
    
    std::map<int, XBot::RobotState> _robot_state_map;
    std::map<int, XBot::RobotState::pdo_tx> _robot_state_tx_map;
    std::map<int, XBot::RobotFT::pdo_rx> _ft_state_map;
    std::map<int, XBot::RobotIMU::pdo_rx> _imu_state_map;


};

#endif //__X_BOT_RT_COMMUNICATION_H__
