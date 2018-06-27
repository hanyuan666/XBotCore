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

#include <XCM/XBotCommunicationInterfaceFactory.h>
#include <XBotInterface/RtLog.hpp>

using XBot::Logger;

std::map<std::string, void*> CommunicationInterfaceFactory::handles;


std::shared_ptr<XBot::CommunicationInterface> CommunicationInterfaceFactory::getFactory(const std::string& file_name,
                                                                                        const std::string& lib_name,
                                                                                        XBot::RobotInterface::Ptr _robot,
                                                                                        XBot::XBotIPC::Ptr ipc_handler
                                                                                       )
{

    char *error;  

    std::string path_to_so = XBot::Utils::FindLib ( file_name + ".so", "LD_LIBRARY_PATH" );
    if ( path_to_so == "" ) {
        throw std::runtime_error ( file_name + ".so" + " path must be listed inside LD_LIBRARY_PATH" );
    }
    
    void* lib_handle;
    lib_handle = dlopen(path_to_so.c_str(), RTLD_NOW);
    if (!lib_handle) {
        Logger::error() << lib_name << " INTERFACE NOT found! \n" << dlerror() << Logger::endl();
    }
    else     
    {
        Logger::success(Logger::Severity::MID) << lib_name <<" INTERFACE found! " << Logger::endl();
        handles[file_name] = lib_handle;
      
        XBot::CommunicationInterface* (*create)(XBot::RobotInterface::Ptr, XBot::XBotIPC::Ptr);
        create = (XBot::CommunicationInterface* (*)(XBot::RobotInterface::Ptr,XBot::XBotIPC::Ptr))dlsym(lib_handle, "create_instance");
        if ((error = dlerror()) != NULL) {
            fprintf(stderr, "%s\n", error);
            exit(1);
        }
        
        XBot::CommunicationInterface* instance =(XBot::CommunicationInterface*)create(_robot, ipc_handler );
        if( instance != nullptr){
          return std::shared_ptr<XBot::CommunicationInterface>(instance);
        }
     }
    return nullptr;
    
}

void CommunicationInterfaceFactory::unloadLib(const std::string& file_name)
{

  dlclose( handles[file_name] );
  Logger::info() << file_name <<" INTERFACE unloaded! " << Logger::endl();
}
