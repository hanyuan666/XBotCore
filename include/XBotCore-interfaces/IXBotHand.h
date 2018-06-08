/*
 * Copyright (C) 2017 IIT-ADVR
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


#ifndef __I_X_BOT_HAND_H__
#define __I_X_BOT_HAND_H__

#include <vector>
#include <memory>

namespace XBot
{
    class IXBotHand;
}

/**
 * @brief XBotCore Hand Interface
 *
 */
class XBot::IXBotHand
{

public:

    typedef std::shared_ptr<XBot::IXBotHand> Ptr;
    
    virtual bool grasp(int hand_id, double grasp_percentage) = 0;
    virtual bool move_finger(int hand_id, int finger_id, double percentage) { return false; };
    
    virtual bool get_finger_motor_position(int hand_id, int finger_id, double& motor_pos) { return false; };
    virtual bool get_finger_current(int hand_id, int finger_id, double& current) { return false; };
    virtual bool get_finger_analog_sensors(int hand_id, int finger_id, double& an1, double& an2, double& an3) { return false; };
    virtual bool get_finger_position_reference(int hand_id, int finger_id, double& position_reference) { return false; };
    
    virtual double get_grasp_state(int hand_id) = 0; //TBD remove it


};

#endif //__I_X_BOT_HAND_H__
