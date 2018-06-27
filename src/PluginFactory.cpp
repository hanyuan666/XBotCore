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

#include <XCM/PluginFactory.h>
#include <XBotInterface/RtLog.hpp>

using XBot::Logger;

std::map<std::string, void*> PluginFactory::handles;

void (*PluginFactory::destroy)(XBot::XBotControlPlugin* instance);



std::shared_ptr<XBot::XBotControlPlugin> PluginFactory::getFactory(const std::string& file_name,
                                                              const std::string& lib_name
                                                               )
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
        XBot::Logger::error() << lib_name <<" RT plugin NOT found! \n" << dlerror() << XBot::Logger::endl();
    }
    else     
    {
        Logger::success(Logger::Severity::MID) << lib_name << " RT plugin found! " << Logger::endl();
        handles[file_name] = lib_handle;
      
        XBot::XBotControlPlugin* (*create)();
        create = (XBot::XBotControlPlugin* (*)())dlsym(lib_handle, "create_instance");
        if ((error = dlerror()) != NULL) {
            fprintf(stderr, "%s\n", error);
            exit(1);
        }
        
        destroy = (void (*)(XBot::XBotControlPlugin* instance))dlsym(lib_handle,"destroy_instance");
        
        XBot::XBotControlPlugin* instance =(XBot::XBotControlPlugin*)create();
        if( instance != nullptr){
          return std::shared_ptr<XBot::XBotControlPlugin>(instance);
        }
     }
    return nullptr;
    
}

void PluginFactory::unloadLib(const std::string& file_name, XBot::XBotControlPlugin* plugin)
{

  destroy(plugin);
  dlclose( handles[file_name] );
  Logger::info() << file_name <<" Plugin unloaded! " << Logger::endl();
}

void PluginFactory::unloadLib(const std::string& file_name)
{

  dlclose( handles[file_name] );
  Logger::info() << file_name <<" Plugin unloaded! " << Logger::endl();
}
