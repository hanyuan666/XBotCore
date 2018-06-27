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

#include <XBotCore/HALInterfaceFactory.h>
#include <XBotInterface/RtLog.hpp>
#include <XBotInterface/Utils.h>

using XBot::Logger;

std::map<std::string, void*> HALInterfaceFactory::handles;

void (*HALInterfaceFactory::destroy)(HALInterface* instance);



std::shared_ptr<HALInterface> HALInterfaceFactory::getFactory(const std::string& file_name,
                                                              const std::string& lib_name,
                                                              const char * config )
{

    char *error;  
    
    /* Obtain full path to shared lib */
    std::string path_to_so = XBot::Utils::FindLib ( file_name + ".so", "LD_LIBRARY_PATH" );
    if ( path_to_so == "" ) {
        throw std::runtime_error ( file_name + ".so" + " path must be listed inside LD_LIBRARY_PATH" );
    }
    
    
    void* lib_handle;
    lib_handle = dlopen(path_to_so.c_str(), RTLD_NOW);
    if (!lib_handle) {
        Logger::error() << lib_name << " HAL interface NOT found! \n" << dlerror() << Logger::endl();
    }
    else     
    {
        Logger::success(Logger::Severity::HIGH) << lib_name << " HAL interface found! " << Logger::endl();
        handles[file_name] = lib_handle;
      
        HALInterface* (*create)(const char * config);
        create = (HALInterface* (*)(const char * config))dlsym(lib_handle, "create_instance");
        if ((error = dlerror()) != NULL) {
            fprintf(stderr, "%s\n", error);
            exit(1);
        }
        
        destroy = (void (*)(HALInterface* instance))dlsym(lib_handle,"destroy_instance");
        
        HALInterface* instance =(HALInterface*)create(config);
        if( instance != nullptr){
          return std::shared_ptr<HALInterface>(instance);
        }
     }
    return nullptr;
    
}

void HALInterfaceFactory::unloadLib(const std::string& file_name, HALInterface* HALInterface)
{

  destroy(HALInterface);
  dlclose( handles[file_name] );
  std::cout << file_name <<" INTERFACE unloaded! " << std::endl;
}
