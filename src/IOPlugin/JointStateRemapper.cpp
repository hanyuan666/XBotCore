/*
 * Copyright (C) 2017 IIT-ADVR
 * Author:
 * email:
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

#include <IOPlugin/JointStateRemapper.h>

#include <XBotInterface/ModelInterface.h>

#include <sensor_msgs/JointState.h>

/* Specify that the class XBotPlugin::JointStateRemapper is a XBot RT plugin with name "JointStateRemapper" */
REGISTER_XBOT_IO_PLUGIN_(XBotPlugin::JointStateRemapper)

namespace XBotPlugin {
    
void XBotPlugin::JointStateRemapper::callback(const XBotCore::JointStateAdvr::ConstPtr& msg) 
{
    sensor_msgs::JointState js_msg;
    
    js_msg.header.stamp = ros::Time::now();
    js_msg.name = msg->name;
    js_msg.position = msg->motor_position;
    js_msg.velocity = msg->motor_velocity;
    js_msg.effort = msg->effort;
    
    _pub.publish(js_msg);
    
}

bool XBotPlugin::JointStateRemapper::init(  std::string path_to_config_file,
                                            XBot::SharedMemory::Ptr shmem
                                            )
{
    auto model = XBot::ModelInterface::getModel(path_to_config_file);
    std::string robot_name = model->getUrdf().getName();
    
    ros::NodeHandle nh;
    _pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
    _sub = nh.subscribe("/xbotcore/" + robot_name + "/joint_states", 1, &XBotPlugin::JointStateRemapper::callback, this);
    
     nh.setParam("/robot_description", model->getUrdfString());
     
    return true;
}

void XBotPlugin::JointStateRemapper::run()
{
    // in ros:spincOnce we trust
}

void XBotPlugin::JointStateRemapper::close()
{

}



}