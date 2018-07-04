/*
 * Copyright (C) 2017 IIT-ADVR
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

#include <WebRobotState.h>

void WebRobotStateTX::serialize(StringBuffer& buffer){
      
        Writer<StringBuffer> writer(buffer);
	writer.StartObject();
	writer.Key("Robot");
        writer.StartObject();  
        serializeArray(writer,"joint_name",joint_name);
        serializeArray(writer,"joint_id",joint_id);
        serializeArray(writer,"link_position",link_position);
        serializeArray(writer,"motor_position", motor_position);
        serializeArray(writer,"link_velocity",link_vel);
        serializeArray(writer,"motor_velocity",motor_vel);
        serializeArray(writer,"temperature",temperature);   
	serializeArray(writer,"effort",effort); 
        serializeArray(writer,"stiffness",stiffness);
        serializeArray(writer,"damping",damping);
        serializeArray(writer,"pos_ref",position_ref);
        serializeArray(writer,"vel_ref",vel_ref);
        serializeArray(writer,"eff_ref",effort_ref);
        serializeArray(writer,"fault",fault);
        serializeArray(writer,"fault_string",fault_string);
        serializeArray(writer,"aux",aux);
        writer.EndObject();
	
	writer.Key("Sensors");
	writer.StartObject();  
	serializeArray(writer,"ft_name", ftsensor.ft_name);
	serializeArray(writer,"ft_id",ftsensor.ft_id);
	ftsensor.serializeArray(writer,"force",ftsensor.force);
	ftsensor.serializeArray(writer,"torque", ftsensor.torque);
	writer.EndObject();  
	writer.EndObject();  
}

//template <typename T>
void WebRobotStateTX::serializeArray(Writer<StringBuffer>& writer, std::string key, std::vector<double>& array){
  
    // writer.StartObject();              
    writer.Key(key.c_str());   
    writer.StartArray();
    for( double val : array ){  
      writer.Double(val);
    }
    writer.EndArray();
    // writer.EndObject();  
}

void WebRobotStateTX::serializeArray(Writer<StringBuffer>& writer, std::string key, std::vector<int>& array){
  
    // writer.StartObject();              
    writer.Key(key.c_str());   
    writer.StartArray();
    for( int val : array ){  
      writer.Int(val);
    }
    writer.EndArray();
    // writer.EndObject();  
}

void WebRobotStateTX::serializeArray(Writer<StringBuffer>& writer, std::string key, std::vector<std::string>& array){
  
    // writer.StartObject();              
    writer.Key(key.c_str());   
    writer.StartArray();
    for( std::string& val : array ){  
      writer.String(val.c_str());
    }
    writer.EndArray();
    // writer.EndObject();  
}

void WebFTSensor::serializeArray(Writer< StringBuffer >& writer, std::string key, std::vector< WebFTSensor::Vector3 >& array)
{
    writer.Key(key.c_str());   
    writer.StartArray();
    for( Vector3 val : array ){  
      writer.StartObject(); 
      writer.Key("Vector");
      writer.StartArray();
      writer.Double(val.x);
      writer.Double(val.y);
      writer.Double(val.z);
      writer.EndArray();
      writer.EndObject(); 
    }
    writer.EndArray();
}