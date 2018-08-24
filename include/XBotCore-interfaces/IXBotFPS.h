/*
 * Copyright (C) 2018 IIT-ADVR
 * Author:  Giuseppe Rigano
 * email:   giuseppe.rigano@iit.it
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

#ifndef __I_X_BOT_FPS_H__
#define __I_X_BOT_FPS_H__

#include <XBotInterface/RtLog.hpp>

namespace XBot
{
    class IXBotFootPressureSensor;
}

/**
 * @brief XBotCore FPS Foot Pressure Sensor Interface
 * 
 */
class XBot::IXBotFootPressureSensor
{

public:   
    
    virtual bool get_forcexy(int fps_id, uint8_t forcexy[]) = 0;
        
    virtual bool get_fps_fault(int fps_id, double& fault) = 0;
    
    virtual bool get_fps_rtt(int fps_id, double& rtt) = 0;
    
    virtual ~IXBotFootPressureSensor() {
        if(Logger::GetVerbosityLevel() == Logger::Severity::LOW)
            std::cout << __func__ << std::endl;
    };
};

#endif //__I_X_BOT_FPS_H__
