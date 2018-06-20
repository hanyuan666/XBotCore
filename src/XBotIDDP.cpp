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

#include <XCM/XBotIDDP.h>

#include <XBotInterface/Utils.h>

XBot::XBotIDDP::XBotIDDP(std::string config_file) : XBot::XBotIPC(config_file)
{
    _xbotinterface = XBot::ModelInterface::getModel(config_file);
}

bool XBot::XBotIDDP::init()
{
    
    for( int id : _xbotinterface->getEnabledJointId() ) {
        if( id > 0 ) {
            XBot::SubscriberIDDP<XBot::RobotState> subscriber_rx(std::string("iddp_Motor_id_") + std::to_string(id));
            fd_read[id] = subscriber_rx;

            XBot::PublisherIDDP<XBot::RobotState::pdo_tx> publisher_tx(std::string("iddp_in_Motor_id_") + std::to_string(id));
            fd_write[id] = publisher_tx;
        }
    }
    

    // hand
    for(auto& hand_j : hand) {
        // initialize all the fd reading/writing for the hands
        XBot::SubscriberIDDP<XBot::RobotState> subscriber_rx(std::string("iddp_Motor_id_") + std::to_string(hand_j.second).c_str());
        fd_read[hand_j.second] = subscriber_rx;

        XBot::PublisherIDDP<XBot::RobotState::pdo_tx> publisher_tx(std::string("iddp_in_Motor_id_") + std::to_string(hand_j.second).c_str());
        fd_write[hand_j.second] = publisher_tx;
    }

    // ft
    for(auto& ft_j : ft) {
        // initialize all the fd reading for the ft
        XBot::SubscriberIDDP<XBot::RobotFT::pdo_rx> subscriber_ft(std::string("iddp_Ft_id_") + std::to_string(ft_j.second).c_str());
        fd_ft_read[ft_j.second] = subscriber_ft;
    }

    // imu
    for(auto& imu_j : imu) {
        // initialize all the fd reading for the imu
        XBot::SubscriberIDDP<XBot::RobotIMU::pdo_rx> subscriber_imu(std::string("iddp_Imu_id_") + std::to_string(imu_j.second).c_str());
        fd_imu_read[imu_j.second] = subscriber_imu;
    }

    
    // motors init
    for(auto& c : robot) {
        for(int i=0; i< c.second.size(); i++) {

            // initialize the pdo_motor
//             if(fd_read.count(c.second[i])) {
                XBot::RobotState current_robot_state;
                if( !(fd_read[c.second[i]].read(current_robot_state)) ) {
                   // give a warning
                }
                pdo_motor[c.second[i]] = std::make_shared<XBot::RobotState>(current_robot_state);
//             }
        }
    }
    
    // hand init
    for(auto& hand_j : hand) {

        // initialize the pdo_motor
//         if(fd_read.count(hand_j.second)) {
            XBot::RobotState current_robot_state;
//             if( !(fd_read[hand_j.second].read(current_robot_state)) ) {
//                 // give a warning
//             }
            pdo_motor[hand_j.second] = std::make_shared<XBot::RobotState>(current_robot_state);
//         }
    }
    
    // ft init
    for(auto& ft_j : ft) {

        // initialize the pdo_ft
//         if(fd_ft_read.count(ft_j.second)) {
            XBot::RobotFT::pdo_rx current_FT_state;
//             if( !(fd_ft_read[ft_j.second].read(current_FT_state)) ) {
//                 // give a warning
//             }
            pdo_FT[ft_j.second] = std::make_shared<XBot::RobotFT::pdo_rx>(current_FT_state);
//         }
    }
    
    // imu init
    for(auto& imu_j : imu) {

        // initialize the pdo_IMU
//         if(fd_imu_read.count(imu_j.second)) {
            XBot::RobotIMU::pdo_rx current_IMU_state;
//             if( !(fd_imu_read[imu_j.second].read(current_IMU_state)) ) {
//                 // give a warning
//             }
            pdo_IMU[imu_j.second] = std::make_shared<XBot::RobotIMU::pdo_rx>(current_IMU_state);
//         }

    }
    

    return true;
}

void XBot::XBotIDDP::updateTX()
{
    // Motor + hands
    for( auto& f: fd_write) {

        // write to the IDDP publisher to command the RobotStateTX in the pdo_motor buffer
        XBot::RobotState::pdo_tx actual_pdo_tx = pdo_motor.at(f.first)->RobotStateTX;
        fd_write.at(f.first).write(actual_pdo_tx);

    }
}

void XBot::XBotIDDP::updateRX()
{
    // Motor + hands
    for( auto& f: fd_read) {
        // NOTE the single joint element can only be controlled by either the RT or the N-RT!

        // reading from the IDDP subscriber pipes to update the RobotStateRX in the pdo_motor buffer
        fd_read.at(f.first).read(*pdo_motor.at(f.first));
    }
    
    // FT
    for(auto& ft_j : fd_ft_read) {
        fd_ft_read.at(ft_j.first).read(*pdo_FT.at(ft_j.first));
    }
    
    // IMU
    for(auto& imu_j : fd_imu_read) {
        fd_imu_read.at(imu_j.first).read(*pdo_IMU.at(imu_j.first));
    }
}

XBot::XBotIDDP::~XBotIDDP()
{
//     Logger::info() << "~XBotIDDP()" << Logger::endl();
}

