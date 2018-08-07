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

#include <XBotPlugin/XBotRTCommunication.h>
#include <XCM/XBotUtils.h>

REGISTER_XBOT_PLUGIN(XBotRTCommunication, XBot::XBotRTCommunication)

XBot::XBotRTCommunication::XBotRTCommunication()
{

}

bool XBot::XBotRTCommunication::init_control_plugin( XBot::Handle::Ptr handle)
{
    // get the robot
    _robot = handle->getRobotInterface();
    
    // initialize esc utils 
    _esc_utils = std::make_shared<XBot::ESCUtils>(_robot);

    // create a PublisherIDDP/PublisherIDDP for each enabled joint in the robot
    for( int id : _robot->getEnabledJointId() ) {
        _pub_read_from_hal[id] = XBot::PublisherIDDP<XBot::RobotState>(std::string("iddp_Motor_id_") + std::to_string(id));
        _sub_write_to_hal[id] = XBot::SubscriberIDDP<XBot::RobotState::pdo_tx>(std::string("iddp_in_Motor_id_") + std::to_string(id));
    
        _robot_state_map[id] = XBot::RobotState();
        _robot_state_tx_map[id] = XBot::RobotState::pdo_tx();
    }
    // and create a PublisherIDDP for hands
    for( const auto& h : _robot->getHand() ) {
        int id = h.second->getHandId();
        _pub_read_from_hal[id] = XBot::PublisherIDDP<XBot::RobotState>(std::string("iddp_Motor_id_") + std::to_string(id));
        _sub_write_to_hal[id] = XBot::SubscriberIDDP<XBot::RobotState::pdo_tx>(std::string("iddp_in_Motor_id_") + std::to_string(id));
    
        _robot_state_map[id] = XBot::RobotState();
        _robot_state_tx_map[id] = XBot::RobotState::pdo_tx();
    }
    // create a PublisherIDDP for each FT
    for( const auto& ft : _robot->getForceTorque() ) {
        int id = ft.second->getSensorId();
        _pub_ft_read_from_hal[id] = XBot::PublisherIDDP<XBot::RobotFT::pdo_rx>(std::string("iddp_Ft_id_") + std::to_string(id));
    
        _ft_state_map[id] = XBot::RobotFT::pdo_rx();
    }
    // create a PublisherIDDP for each IMU
    for( const auto& imu : _robot->getImu() ) {
        int id = imu.second->getSensorId();
        _pub_imu_read_from_hal[id] = XBot::PublisherIDDP<XBot::RobotIMU::pdo_rx>(std::string("iddp_Imu_id_") + std::to_string(id));

        _imu_state_map[id] = XBot::RobotIMU::pdo_rx();
    }
   

    return true;
}

void XBot::XBotRTCommunication::on_start(double time)
{
    _robot->sense();
        
    Eigen::VectorXd q_mot;
    _robot->getMotorPosition(q_mot);
    _robot->setPositionReference(q_mot);
    
    _esc_utils->setRobotStateFromRobotInterface(_robot_state_map);

    for(auto &p : _robot_state_map) {
        _robot_state_tx_map[p.first] = p.second.RobotStateTX;
    }
    
    // send motor state to EXTERNAL RT
    for( auto& p: _robot_state_map) {
       _pub_read_from_hal.at(p.first).write(p.second); 
    }

}

void XBot::XBotRTCommunication::on_stop(double time)
{
}


void XBot::XBotRTCommunication::control_loop(double time, double period)
{
    // READ FROM HAL - PUB ON EXTERNAL RT
    _esc_utils->setRobotStateFromRobotInterface(_robot_state_map);
    _esc_utils->setRobotFTFromRobotInterface(_ft_state_map);
    _esc_utils->setRobotIMUFromRobotInterface(_imu_state_map);
    
    // send motor state to EXTERNAL RT
    for( auto& p: _robot_state_map) {
       _pub_read_from_hal.at(p.first).write(p.second); 
    }
    
    // send FT state to EXTERNAL RT
    for( auto& p: _ft_state_map) {
       _pub_ft_read_from_hal.at(p.first).write(p.second); 
    }
    
    // send IMU state to EXTERNAL RT
    for( auto& p: _imu_state_map) {
       _pub_imu_read_from_hal.at(p.first).write(p.second); 
    }

    // SUBSCRIBE FROM EXTERNAL RT - WRITE TO HAL
    for( auto& p : _sub_write_to_hal) {
        
        p.second.read(_robot_state_tx_map[p.first]);
//         XBot::Logger::error() << _robot_state_tx_map[p.first].pos_ref << XBot::Logger::endl();  
    }

    _esc_utils->setReferenceFromRobotStateTX(_robot_state_tx_map);

    
    // move the robot
    _robot->move();
    
}

bool XBot::XBotRTCommunication::close(void)
{
    return true;
}

XBot::XBotRTCommunication::~XBotRTCommunication()
{
    DPRINTF("~XBotRTCommunication()\n");
}

