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

#ifndef test_io_IOPLUGIN_H_
#define test_io_IOPLUGIN_H_

#include <XCM/IOPlugin.h>
#include <XBotCore/JointStateAdvr.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


namespace XBotPlugin {

/**
 * @brief JointStateRemapper XBot IO Plugin. This plugin extends the CommunicationHandler
 * loop with custom functionalities.
 *
 **/
class JointStateRemapper : public XBot::IOPlugin
{

public:

    virtual bool init(std::string path_to_config_file, 
                      XBot::SharedMemory::Ptr shmem
                      );

    virtual void run();

    virtual void close();

protected:


private:

    ros::Publisher _pub;
    ros::Subscriber _sub;
    
    void callback(const XBotCore::JointStateAdvr::ConstPtr& msg);


};

}

#endif // test_io_PLUGIN_H_