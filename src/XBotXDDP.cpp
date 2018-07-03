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

#include <XCM/XBotXDDP.h>

#include <XBotInterface/Utils.h>

XBot::XBotXDDP::XBotXDDP(std::string config_file) : XBot::XBotIPC(config_file)
{
   
}

bool XBot::XBotXDDP::init()
{
    // motors
    for(auto& c : robot) {
        for(int i=0; i< c.second.size(); i++) {

            XBot::SubscriberNRT<XBot::RobotState> subscriber_rx(std::string("Motor_id_") + std::to_string(c.second[i]).c_str());
            fd_read[c.second[i]] = subscriber_rx;

            XBot::PublisherNRT<XBot::RobotState::pdo_tx> publisher_tx(std::string("rt_in_Motor_id_") + std::to_string(c.second[i]).c_str());
            fd_write[c.second[i]] = publisher_tx;

            // initialize the pdo_motor
            if(fd_read.count(c.second[i])) {
                XBot::RobotState current_robot_state;
                while( !(fd_read[c.second[i]].read(current_robot_state)) ) {
                   sleep(1);
                }
                pdo_motor[c.second[i]] = std::make_shared<XBot::RobotState>(current_robot_state);
            }
        }
    }

    //ft
    for(auto& ft_j : ft) {
        // initialize all the fd reading for the ft
        XBot::SubscriberNRT<XBot::RobotFT::pdo_rx> subscriber_ft(std::string("Ft_id_") + std::to_string(ft_j.second).c_str());
        fd_ft_read[ft_j.second] = subscriber_ft;
        
        // initialize the pdo_ft
        if(fd_ft_read.count(ft_j.second)) {
            XBot::RobotFT::pdo_rx current_FT_state;
            while( !(fd_ft_read[ft_j.second].read(current_FT_state)) ) {
                sleep(1);
            }
            pdo_FT[ft_j.second] = std::make_shared<XBot::RobotFT::pdo_rx>(current_FT_state);
        }
    }

    //imu
    for(auto& imu_j : imu) {
        // initialize all the fd reading for the imu
        XBot::SubscriberNRT<XBot::RobotIMU::pdo_rx> subscriber_imu(std::string("Imu_id_") + std::to_string(imu_j.second).c_str());
        fd_imu_read[imu_j.second] = subscriber_imu;
        
        // initialize the pdo_IMU
        if(fd_imu_read.count(imu_j.second)) {
            XBot::RobotIMU::pdo_rx current_IMU_state;
            while( !(fd_imu_read[imu_j.second].read(current_IMU_state)) ) {
                sleep(1);
            }
            pdo_IMU[imu_j.second] = std::make_shared<XBot::RobotIMU::pdo_rx>(current_IMU_state);
        }

    }

    //hand
    for(auto& hand_j : hand) {
        // initialize all the fd reading/writing for the hands
        XBot::SubscriberNRT<XBot::RobotState> subscriber_rx(std::string("Motor_id_") + std::to_string(hand_j.second).c_str());
        fd_read[hand_j.second] = subscriber_rx;

        XBot::PublisherNRT<XBot::RobotState::pdo_tx> publisher_tx(std::string("rt_in_Motor_id_") + std::to_string(hand_j.second).c_str());
        fd_write[hand_j.second] = publisher_tx;

        // initialize the pdo_motor
        if(fd_read.count(hand_j.second)) {
            XBot::RobotState current_robot_state;
            while( !(fd_read[hand_j.second].read(current_robot_state)) ) {
                sleep(1);
            }
            pdo_motor[hand_j.second] = std::make_shared<XBot::RobotState>(current_robot_state);
        }
    }
    
    return true;
}

void XBot::XBotXDDP::updateTX()
{
    // Motor + hands
    for( auto& f: fd_read) {

        // write to the NRT publisher to command the RobotStateTX in the pdo_motor buffer
        XBot::RobotState::pdo_tx actual_pdo_tx = pdo_motor.at(f.first)->RobotStateTX;
        fd_write.at(f.first).write(actual_pdo_tx);

    }
}

void XBot::XBotXDDP::updateRX()
{
    // Motor + hands
    for( auto& f: fd_read) {
        // NOTE the single joint element can only be controlled by either the RT or the N-RT!

        // reading from the NRT subscriber pipes to update the RobotStateRX in the pdo_motor buffer
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

XBot::XBotXDDP::~XBotXDDP()
{
//     Logger::info() << "~XBotXDDP()" << Logger::endl();
}

