/*
 * Copyright (C) 2016 IIT-ADVR
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

#include <XCM/MessageInterfaces/AdvrCommandMessage.h>
#include <XBotInterface/RtLog.hpp>
#include <ros/transport_hints.h> 
#include <XBotInterface/SoLib.h>

REGISTER_SO_LIB_(XBot::CommandAdvr, XBot::GenericControlMessage);

using XBot::Logger;

void XBot::CommandAdvr::callback(XBotCore::CommandAdvrConstPtr msg)
{
    for(int i = 0; i < msg->name.size(); i++){

        int idx = getIndex(msg->name[i]);

        if(idx < 0){
            Logger::warning("ERROR while parsing CommandAdvr message: joint %s undefined\n", msg->name[i].c_str());
            continue;
        }
        
        ControlMode ctrl_mode;
        
        if(msg->ctrl_mode.size() > i){
            ctrl_mode = ControlMode::FromBitset(ControlMode::Bitset(msg->ctrl_mode[i]));
        }
        else
        {
            Logger::warning("ERROR while parsing CommandAdvr message: %d-th control mode undefined\n Send a bitmask [Damp Stiff Torque Vel Pos], e.g. 3 for position and velocity control\n ", i);
            continue;
        }

        if(msg->aux.size() > i){
            _msg.aux[idx] = msg->aux[i];
        }

        _msg.aux_name = msg->aux_name;

        if(msg->damping.size() > i && ctrl_mode.isDampingEnabled()){
            _msg.damping[idx] = msg->damping[i];
        }

        if(msg->effort.size() > i && ctrl_mode.isEffortEnabled()){
            _msg.effort[idx] = msg->effort[i];
        }

        if(msg->position.size() > i && ctrl_mode.isPositionEnabled()){
            _msg.position[idx] = msg->position[i];
        }

        if(msg->stiffness.size() > i && ctrl_mode.isStiffnessEnabled()){
            _msg.stiffness[idx] = msg->stiffness[i];
        }

        if(msg->velocity.size() > i && ctrl_mode.isVelocityEnabled()){
            _msg.velocity[idx] = msg->velocity[i];
        }
        
        _msg.seq_id++;
    }
}

bool XBot::CommandAdvr::service_callback(XBotCore::advr_controller_joint_namesRequest& req,
                                         XBotCore::advr_controller_joint_namesResponse& res)
{
    res = _joint_names_res;
    return true;
}


bool XBot::CommandAdvr::init(const ConfigOptions& cfg, XBot::GenericControlMessage::Type type)
{

    Logger::info("Initializing CommandAdvr message interface\n");
    
    std::string robot_name = XBot::ModelInterface::getModel(cfg)->getUrdf().getName();
    std::string joint_service_name = "/" + robot_name + "/position_controller/get_joint_names";
    std::string command_topic_name = "/xbotcore/" + robot_name + "/command";

    ros::NodeHandle nh;

    
    
    if( type == XBot::GenericControlMessage::Type::Rx ) {
        
        const int QUEUE_SIZE = 100;
        _sub = nh.subscribe(command_topic_name, 
                            QUEUE_SIZE, 
                            &XBot::CommandAdvr::callback, this, 
                            ros::TransportHints().tcpNoDelay());

        auto robot = XBot::RobotInterface::getRobot(cfg, "xddp_robot");
        robot->sense();

        XBot::JointNameMap _joint_pos, _joint_stiffness, _joint_damping;
        robot->getJointPosition(_joint_pos);
        robot->getStiffness(_joint_stiffness);
        robot->getDamping(_joint_damping);

        XBot::ControlMode::Bitset bit_set;
        bit_set.set();
        
        for( const std::string& jname : robot->getEnabledJointNames() ){

            _joint_names_res.name.push_back(jname);
            _msg.name.push_back(jname);
            _msg.aux.push_back(0);
            _msg.damping.push_back(_joint_damping.at(jname));
            _msg.effort.push_back(0);
            _msg.position.push_back(_joint_pos.at(jname));
            _msg.stiffness.push_back(_joint_stiffness.at(jname));
            _msg.velocity.push_back(0);
            _msg.ctrl_mode.push_back(static_cast<uint8_t>(bit_set.to_ulong()));
            
        }
        
        _msg.seq_id = 0;

          _joint_names_srv = nh.advertiseService(joint_service_name, &XBot::CommandAdvr::service_callback, this);
    }


    if( type == XBot::GenericControlMessage::Type::Tx ) {
        
        _pub = nh.advertise<XBotCore::CommandAdvr>(command_topic_name, 1);
        
        ros::ServiceClient client = nh.serviceClient<XBotCore::advr_controller_joint_names>(joint_service_name);
        XBotCore::advr_controller_joint_namesRequest req;

        if (!client.call(req, _joint_names_res)) {
            std::cerr << "ERROR service client " << joint_service_name << " not available! Write() won't work!!" << std::endl;
            return false;
        }
        
        XBot::ControlMode::Bitset bit_set;
        bit_set = ~bit_set.set();

        for( const std::string& jname : _joint_names_res.name ){
            _msg.name.push_back(jname);
            _msg.aux.push_back(0);
            _msg.damping.push_back(0);
            _msg.effort.push_back(0);
            _msg.position.push_back(0);
            _msg.stiffness.push_back(0);
            _msg.velocity.push_back(0);
            _msg.ctrl_mode.push_back(static_cast<uint8_t>(bit_set.to_ulong()));
            
        }
        
        _msg.seq_id = 0;
    }

    // Populate _idx_map
    {
        int idx = 0;
        for( const std::string& joint_name : _joint_names_res.name){
            _idx_map[joint_name] = idx;
            idx++;
        }

    }
    
    Logger::success() << "Successfully initialized CommandAdvr message interface!" << Logger::endl();
    
    return true;

}

int XBot::CommandAdvr::getIndex(const std::string& joint_name)
{
    auto it = _idx_map.find(joint_name);

    if( it != _idx_map.end() ){
        return it->second;
    }
    else{
        Logger::warning() << " in " << __func__ << "! Joint " << joint_name << " is not defined inside the ROS controller!" << Logger::endl();
        return -1;
    }

}

double& XBot::CommandAdvr::aux(int index)
{
    return _msg.aux[index];
}

std::string& XBot::CommandAdvr::aux_name()
{
    return _msg.aux_name;
}

double& XBot::CommandAdvr::damping(int index)
{
    return _msg.damping[index];
}

double& XBot::CommandAdvr::effort(int index)
{
    return _msg.effort[index];
}

double& XBot::CommandAdvr::position(int index)
{
    return _msg.position[index];
}

double& XBot::CommandAdvr::stiffness(int index)
{
    return _msg.stiffness[index];
}

double& XBot::CommandAdvr::velocity(int index)
{
    return _msg.velocity[index];
}

int& XBot::CommandAdvr::seq_id()
{
    return _msg.seq_id;
}

XBot::ControlMode XBot::CommandAdvr::get_ctrl_mode(int index) const
{
    return ControlMode::FromBitset( ControlMode::Bitset(_msg.ctrl_mode[index]) );
}

void XBot::CommandAdvr::set_ctrl_mode(int index, const XBot::ControlMode& ctrl_mode)
{
    _msg.ctrl_mode[index] = static_cast<uint8_t>(ControlMode::AsBitset(ctrl_mode).to_ulong());
}


void XBot::CommandAdvr::publish()
{
    _pub.publish(_msg);
}