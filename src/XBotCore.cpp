/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
       Arturo Laurenzi (2016-, arturo.laurenzi@iit.it)
       Giuseppe Rigano (2017, giuseppe.rigano@iit.it)
       
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

/**
 *
 * @author Luca Muratore (2016-, luca.muratore@iit.it)
 * @author Arturo Laurenzi (2016-, arturo.laurenzi@iit.it)
 * @author Giuseppe Rigano (2017-, giuseppe.rigano@iit.it)
*/


#include <boost/bind.hpp>
#include <XBotCore/XBotCore.h>
#include <XBotCore/HALInterfaceFactory.h>


#include <XBotInterface/Utils.h>
#include <XBotInterface/RtLog.hpp>


using XBot::Logger;

std::shared_ptr<Loader> XBot::XBotCore::loaderptr;

XBot::XBotCore::XBotCore(std::string config_yaml, 
                         std::shared_ptr<HALBase> halinterface, 
                         XBot::SharedMemory::Ptr shmem,
                         Options options,
                         std::shared_ptr<XBot::TimeProviderFunction<boost::function<double()>>> time_provider
                         
                        ) : 
    _path_to_config(config_yaml),
    _shmem(shmem),
    _options(options),
    halInterface(halinterface)
{        
    _time_provider = time_provider;
    if(!halInterface) exit(1);
}

std::shared_ptr<Loader> XBot::XBotCore::getLoader(){
  
  return loaderptr;  
}

void XBot::XBotCore::init_internal()
{
    // create robot from config file and any map
    XBot::AnyMapPtr anymap = std::make_shared<XBot::AnyMap>();
    
    
    // TODO initilize it somewhere else
    bool xbot_enable_transmission = true;
    
    (*anymap)["HAL"] = boost::any(halInterface);
    (*anymap)["XBotJoint"] = boost::any(halInterface->getJoint());
//     (*anymap)["XBotFT"] = boost::any(halInterface->getSensorId(18));
//     (*anymap)["XBotIMU"] = boost::any(halInterface->getSensorId(104));
//     (*anymap)["XBotHand"] = boost::any(halInterface->getHandId(19));
    (*anymap)["EnableTransmissionPlugins"] = boost::any(xbot_enable_transmission);
    
    //TODO use isRT from RobotControlInterface robotInterface.IsRt()
    _robot = XBot::RobotInterface::getRobot(_path_to_config, "", anymap, "XBotRT");
    
    // create time provider function
    boost::function<double()> time_func = boost::bind(&XBot::XBotCore::get_time, this);
    
    if (!_time_provider){
      // create time provider
      _time_provider = std::make_shared<XBot::TimeProviderFunction<boost::function<double()>>>(time_func);
    }
    
    // create plugin handler
    _pluginHandler = std::make_shared<XBot::PluginHandler>(_robot, _time_provider, _shmem, _options, halInterface);

    _pluginHandler->load_plugins();
    
    //_pluginHandler->init_plugins(hal);
    
    loaderptr = std::make_shared<Loader>(_pluginHandler);
    loaderth = new XBot::XBotLoaderThread();
    loaderth->create(false, 2);
    
    return;
}

void XBot::XBotCore::control_init(void) 
{
     halInterface->base_init();
     init_internal();    
     
     return;
}

double XBot::XBotCore::get_time()
{
    return XBot::get_time_ns() / 1e9;
}

int XBot::XBotCore::control_loop(void) 
{       
    int state = halInterface->base_recv();
    if(state == 0)
      loop_internal();  
    halInterface->base_send();
}

void XBot::XBotCore::loop_internal()
{
    _iter++;
    _pluginHandler->run();  
}

XBot::XBotCore::~XBotCore() {
    
    _pluginHandler->close();
    
    Logger::info() << "~XBotCore()" << Logger::endl();
    
    loaderth->stop();
    loaderth->join();
    delete loaderth;
}