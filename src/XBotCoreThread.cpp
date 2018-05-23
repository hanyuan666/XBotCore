/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
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
 * @author Giuseppe Rigano (2017-, giuseppe.rigano@iit.it)
*/

#include <XBotCore/XBotCoreThread.h>
#include <XBotCore/HALInterfaceFactory.h>
#include <XBotInterface/SoLib.h>
#include <XBotCore/HALThread.h>

XBot::XBotCoreThread::XBotCoreThread(std::string config_yaml, 
                   XBot::SharedMemory::Ptr shared_memory,  
                   XBot::Options options, 
                   HALInterface::Ptr hal,
                   std::shared_ptr<XBot::TimeProviderFunction<boost::function<double()>>> time_provider)   
{
    int period = 1;
    if(options.xbotcore_dummy_mode){      
         period = options.xbotcore_period_us;
    }
      
    set_thread_name("XBOT");
    
    // set thread period - not periodic
    task_period_t t;
    memset(&t, 0, sizeof(t));
    t.period = {0,period};
    set_thread_period(t);
    
    // set thread priority
    set_thread_priority();
    
    // obtain hal
    YAML::Node root_cfg = YAML::LoadFile(config_yaml);
    
    YAML::Node x_bot_core, root;
    
    std::string abs_low_level_config = "";
    
    // check gains in XBotCore node specifing config path YAML
    if(root_cfg["XBotCore"]) {
        x_bot_core = root_cfg["XBotCore"];
            
        if(x_bot_core["config_path"]) {
            
            abs_low_level_config = XBot::Utils::computeAbsolutePath(x_bot_core["config_path"].as<std::string>());
            
            Logger::info() << "Path to low level config is " << x_bot_core["config_path"].as<std::string>() << Logger::endl();
            
            Logger::info() << "Abs path to low level config is " << abs_low_level_config << Logger::endl();
            
            root = YAML::LoadFile(abs_low_level_config);
        }
    }

    
    YAML::Node hal_lib;
    std::string lib_file = "";
    std::string lib_name="";
    std::string  iJoint="";
    
    HALInterface::Ptr __hal;
    HALBase::Ptr halInterface;
    
    std::string path_to_shared_lib;
    path_to_shared_lib = XBot::Utils::computeAbsolutePath("build/install/lib/");
    
    if(options.xbotcore_dummy_mode){ // dummy mode
    
        lib_file = "libXBotDummy";
        lib_name = "DUMMY";
        iJoint = "libXBotDummyJoint";
	
        // NOTE dummy needs high level config
        abs_low_level_config = std::string(config_yaml);        
    }
    else if(options.xbotcore_simulator_mode){ 

        lib_file = "libXBotGazeboRos";
        lib_name = "XBotGazeboRos";
        iJoint = "libXBotGazeboRosJoint";
        
        // NOTE dummy needs high level config
        abs_low_level_config = std::string(config_yaml);            
    }    
    else if(hal) // hal provided by constructor
    {
        __hal = hal;
    }
    else if(!hal && hal_lib) // hal is not provided by the constructor
    {
        hal_lib = root["HALInterface"];
	lib_file = hal_lib["lib_file"].as<std::string>();
        lib_name = hal_lib["lib_name"].as<std::string>();
	if (hal_lib["IJoint"]){
	  iJoint = hal_lib["IJoint"].as<std::string>();
	}else{
	   XBot::Logger::warning()<<"Yaml node IJoint missing in the config. file"<<Logger::endl();
	}
    }
    else{
        throw std::runtime_error("Unable to load HAL");
    }
      
    __hal = HALInterfaceFactory::getFactory(lib_file, lib_name, abs_low_level_config.c_str());
    halInterface = std::dynamic_pointer_cast<HALBase>(__hal);
    std::string lib;
    if(!std::dynamic_pointer_cast<XBot::HALThread>(__hal)){
      lib = path_to_shared_lib + iJoint+".so";
      HALInterface::Ptr joint = SoLib::getFactoryWithArgs<HALInterface>(lib,"JOINT",__hal);
      if (joint){
	halInterface->setJoint(joint);
      }else {
	XBot::Logger::error()<<"Unable to load IJoint lib "<<iJoint<<Logger::endl();
      }
    }
    
    if (hal_lib["IEndEffectors"]){
      const YAML::Node& list = hal_lib["IEndEffectors"];
      XBot::Logger::info(Logger::Severity::HIGH)<<"Loading end_effectors.."<<Logger::endl();
      for(const auto& end_eff : list){
	  std::string iend_eff = end_eff[1].as<std::string>();
	  lib = path_to_shared_lib + iend_eff+".so";
	  HALInterface::Ptr hand = SoLib::getFactoryWithArgs<HALInterface>(lib,"HAND"+iend_eff,halInterface);
	  for(const auto& id : end_eff[0]){
	    int idn =  id.as<int>();
	    if( hand){
	      XBot::Logger::info(Logger::Severity::HIGH)<<"ID: "<<id<<" lib: "<<iend_eff<<Logger::endl();
	      halInterface->setHandId(idn,hand);
	      if (!halInterface->isLoaded(iend_eff)) halInterface->addLib(iend_eff,hand);
	    }else{
	      XBot::Logger::error()<<"Unable to load end_effector ID: "<<id<<" lib: "<<iend_eff<<Logger::endl();
	    }
	  }
      }  
    }
  
    if (hal_lib["ISensors"]){
      const YAML::Node& list = hal_lib["ISensors"];
      XBot::Logger::info(Logger::Severity::HIGH)<<"Loading sensors.."<<Logger::endl();
      for(const auto& sensor : list){
	  std::string isensor = sensor[1].as<std::string>();
	  lib = path_to_shared_lib + isensor+".so";
	  HALInterface::Ptr sensorptr = SoLib::getFactoryWithArgs<HALInterface>(lib,"SENSOR"+isensor,halInterface);
	  for(const auto& id : sensor[0]){
	      int idn =  id.as<int>();
	      if(sensorptr){
		XBot::Logger::info(Logger::Severity::HIGH)<<"ID: "<<idn<<" lib: "<<isensor<<Logger::endl();
		halInterface->setSensorId(idn,sensorptr);
		if (!halInterface->isLoaded(isensor)) halInterface->addLib(isensor,sensorptr);
	      }else{
		XBot::Logger::error()<<"Unable to load sensor ID: "<<idn<<" lib: "<<isensor<<Logger::endl();
	      }
	  }
      }  
    }
    
    
    if(options.xbotcore_simulator_mode){ 
      boost::function<double()> time_func = boost::bind(boost::mem_fn(&HALBase::getTime), boost::ref(halInterface));   
      time_provider = std::make_shared<XBot::TimeProviderFunction<boost::function<double()>>>(time_func);
      
      lib =  path_to_shared_lib + "libXBotGazeboRosFT.so";
      HALInterface::Ptr sensorptrft = SoLib::getFactoryWithArgs<HALInterface>(lib,"SENSORft",halInterface);
      if (!halInterface->isLoaded("ft")) halInterface->addLib("ft",sensorptrft);
      lib =  path_to_shared_lib + "libXBotGazeboRosIMU.so";
      HALInterface::Ptr sensorptrimu = SoLib::getFactoryWithArgs<HALInterface>(lib,"SENSORimu",halInterface);
      if (!halInterface->isLoaded("imu")) halInterface->addLib("imu",sensorptrimu);
    }      
        
    controller = std::shared_ptr<ControllerInterface>(new XBot::XBotCore(config_yaml, 
                                                                         halInterface, 
                                                                         shared_memory, 
                                                                         options, 
                                                                         time_provider)
                                                     );
}

void XBot::XBotCoreThread::set_thread_name(std::string thread_name)
{
    // save the thread name
    this->thread_name = thread_name;
    // set thread name
    name = this->thread_name.c_str();
}

std::string XBot::XBotCoreThread::get_thread_name(void)
{
    return thread_name;
}

void XBot::XBotCoreThread::set_thread_period(task_period_t t)
{
    period.task_time = t.task_time;
    period.period = t.period;
}

void XBot::XBotCoreThread::set_thread_priority()
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

void XBot::XBotCoreThread::th_init( void * ){
  
    controller->control_init();
}

void XBot::XBotCoreThread::th_loop( void * ){  
  
    controller->control_loop();  
}

XBot::XBotCoreThread::~XBotCoreThread() {
    
   Logger::info() << "~XBotCoreThread()" << Logger::endl();
}
