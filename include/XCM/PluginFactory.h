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


#ifndef __PLUGIN_FACTORY_H__
#define __PLUGIN_FACTORY_H__

#include <map>
#include <string>
#include <iostream>

#include <dlfcn.h>

#include <XCM/XBotControlPlugin.h>

class PluginFactory {

public:
  
  static std::shared_ptr<XBot::XBotControlPlugin> getFactory(const std::string& file_name, const std::string& lib_name);

  static void unloadLib(const std::string& file_name,XBot::XBotControlPlugin* plugin);
  
  static void unloadLib(const std::string& file_name);

private:

  static void (*destroy)(XBot::XBotControlPlugin* instance);
  
  PluginFactory() = delete;
  
  static std::map<std::string, void*> handles;
  

};


#endif
