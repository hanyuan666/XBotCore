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

#include <HttpHandler.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace urdf {
    typedef  boost::shared_ptr<Link> LinkSharedPtr;
    typedef  boost::shared_ptr<const Link> LinkConstSharedPtr;
    typedef  boost::shared_ptr<Joint> JointSharedPtr;
    typedef  boost::shared_ptr<const Joint> JointConstSharedPtr;
    typedef  boost::shared_ptr<Material> MaterialSharedPtr;
    typedef  boost::shared_ptr<const Material> MaterialConstSharedPtr;
}


HttpHandler::HttpHandler (std::shared_ptr<SharedData>& sharedData, std::shared_ptr<Buffer<WebRobotStateTX>>& buffer){
      
      this->sharedData = sharedData;
      this->buffer = buffer;
}
 
void HttpHandler::handleGet(std::shared_ptr<ResponseInterface>& response){      
         
      std::shared_ptr<StringBuffer> jsonresp = std::make_shared<StringBuffer>();
      jsonresp->Clear();
      Writer<StringBuffer> writer(*jsonresp);
  
      if(uri.compare("/switch")==0){
        sharedData->insertSwitch(key, val);
        writer.StartObject();  
        writer.Key("Response");
        //TODO check return value insertswitch and answer accordingly
        writer.String("OK");
        writer.EndObject();
      }
      else if(uri.compare("/status")==0){
        writer.StartObject();  
        writer.Key("Response");
        std::string s =sharedData->getAllStatus()[key];
        if (s.empty()) s = "Error: Not Found";
        writer.String(s.c_str());  
        writer.EndObject();
      }
      else if(uri.compare("/master")==0){
        sharedData->setMaster(key);
        writer.StartObject();  
        writer.Key("Response");
        writer.String("OK");
        writer.EndObject();
      }
      else if(uri.compare("/plugins")==0){
                
        writer.StartObject();  
        writer.Key("Plugins");   
        writer.StartArray();
        for( auto const &s : sharedData->getAllStatus()){                 
            const std::string& outer_key = s.first;
            const std::string& inner_map = s.second;
            writer.StartObject();  
            writer.Key("Name");
            writer.String(outer_key.c_str());
            writer.Key("Status");
            writer.String(inner_map.c_str());
            writer.EndObject();
        }
        writer.EndArray();
        writer.EndObject();     
      }
      else if(uri.compare("/state")==0){
              
        WebRobotStateTX rstate;
        bool resp = buffer->remove(rstate);
        if(resp){      
            rstate.serialize(*jsonresp);
        }
      }
      else if(uri.compare("/chains")==0){
        
        writer.StartObject();  
        writer.Key("Chains");   
        writer.StartArray();
        for( auto const &pair : sharedData->getChainMap()){                 
            const std::string& outer_key = pair.first;
            const std::vector< std::vector<std::string> >& inner_map = pair.second;
            writer.StartObject();  
            writer.Key("Chain");
            writer.String(outer_key.c_str());
            writer.Key("Val"); 
            writer.StartArray();
            std::vector<std::string> v = inner_map[0];
            for (int i=0 ;i< v.size(); i++){
                writer.StartObject(); 
                writer.Key("ID");
                std::vector<std::string> idv = inner_map[0];
                std::vector<std::string> nv = inner_map[1];
                std::vector<std::string> lv = inner_map[2];
                std::vector<std::string> vv = inner_map[3];
                std::vector<std::string> ev = inner_map[4];
                std::vector<std::string> sv = inner_map[5];
                std::vector<std::string> dv = inner_map[6];
                std::vector<std::string> pref = inner_map[7];
                std::vector<std::string> vref = inner_map[8];
                std::vector<std::string> eref = inner_map[9];                
                std::vector<std::string> llv = inner_map[10];
                std::vector<std::string> ulv = inner_map[11];
                writer.Int(std::stoi(idv[i]));
                writer.Key("Name");
                writer.String(nv[i].c_str());
                writer.Key("Lval");
                writer.Double(std::stod(lv[i]));
                writer.Key("Vval");
                writer.Double(std::stod(vv[i]));
                writer.Key("Eval");
                writer.Double(std::stod(ev[i]));
                writer.Key("Sval");
                writer.Double(std::stod(sv[i]));
                writer.Key("Dval");
                writer.Double(std::stod(dv[i]));
                writer.Key("pos_ref");
                writer.Double(std::stod(pref[i]));
                writer.Key("vel_ref");
                writer.Double(std::stod(vref[i]));
                writer.Key("eff_ref");
                writer.Double(std::stod(eref[i]));
                writer.Key("Llimit");
                writer.Double(std::stod(llv[i]));
                writer.Key("Ulimit");
                writer.Double(std::stod(ulv[i]));
                writer.EndObject();
            }
            writer.EndArray();;
            writer.EndObject();
        }
        writer.EndArray();
        writer.EndObject();     
      }
      else if(uri.compare("/model")==0){
      
	std::vector<urdf::LinkSharedPtr> array;
	sharedData->model.getLinks(array);
	sharedData->getRobotState();
	
	urdf::LinkConstSharedPtr rlink = sharedData->model.getRoot();
	
	writer.StartObject();  
	writer.Key("Model");   
	writer.StartObject();  
	writer.Key("Root_Link");
	writer.String( rlink->name.c_str());
	writer.Key("Root_Joint");
	writer.String(rlink->child_joints[0]->name.c_str());
	writer.Key("Joint");
	writer.StartArray();
	for ( auto& joint : sharedData->model.joints_){
	    
	    std::string jname = joint.first;
	    urdf::JointSharedPtr jptr = joint.second;
	    std::string parent_link = jptr->parent_link_name;
	    std::string child_link = jptr->child_link_name;
	    urdf::LinkConstSharedPtr linkp = sharedData->model.getLink(parent_link);
	    urdf::LinkConstSharedPtr linkc = sharedData->model.getLink(child_link);
	    
	    urdf::Vector3 axis = jptr->axis;
	    urdf::Vector3 pos = jptr->parent_to_joint_origin_transform.position;
	    double x,y,z,w;
	    jptr->parent_to_joint_origin_transform.rotation.getQuaternion(x,y,z,w);
	    Eigen::Quaterniond q(w,x,y,z);
	    Eigen::AngleAxisd aq(q);
	    double angle = aq.angle();
	    double ax,ay,az;
	    Eigen::AngleAxis<double>::Vector3 raxis = aq.axis();
	    ax = raxis[0];
	    ay = raxis[1];
	    az = raxis[2];
	    double rx,ry,rz;
	    rx = jptr->axis.x;
	    ry = jptr->axis.y;
	    rz = jptr->axis.z;
	    writer.StartObject();  
	    writer.Key("Name");
	    writer.String( jname.c_str());
	    writer.Key("Parent");
	    writer.String( linkp->name.c_str());
	    writer.Key("Child");
	    writer.String( linkc->name.c_str());
	    writer.Key("Pos");
	    writer.StartArray();
	    writer.Double(pos.x);
	    writer.Double(pos.y);
	    writer.Double(pos.z);
	    writer.EndArray();
	    writer.Key("Rot");
	    writer.StartObject();  
	    writer.Key("Axis");
	    writer.StartArray();
	    writer.Double(ax);
	    writer.Double(ay);
	    writer.Double(az);
	    writer.EndArray();
	    writer.Key("Angle");
	    writer.Double(angle);
	    writer.EndObject();
	    writer.Key("Axis");
	    writer.StartArray();
	    writer.Double(rx);
	    writer.Double(ry);
	    writer.Double(rz);
	    writer.EndArray();
	    writer.EndObject();
	}
	writer.EndArray();
	writer.Key("Link");
	writer.StartArray();
	
	for (int i = 0; i<array.size() ; i++){
	    
	    urdf::LinkSharedPtr link = array[i];
	    writer.StartObject();  
	    writer.Key("Name");
	    writer.String( link->name.c_str());
	    
	    if (sharedData->ft_sensors.count(link->name)){
	       writer.Key("Sensor");
	       writer.String( link->name.c_str());
	    }
	    if( link->visual){
	      urdf::Vector3 pos = link->visual->origin.position;
	      double x,y,z,w;
	      link->visual->origin.rotation.getQuaternion(x,y,z,w);
	      Eigen::Quaterniond q(w,x,y,z);
	      Eigen::AngleAxisd aq(q);
	      double angle = aq.angle();
	      double ax,ay,az;
	      Eigen::AngleAxis<double>::Vector3 axis = aq.axis();
	      ax = axis[0];
	      ay = axis[1];
	      az = axis[2];
	      urdf::Vector3 scale(1,1,1);
	      urdf::Geometry* geo = link->visual->geometry.get();
	      std::string mesh_name;
	      if (geo != nullptr){
		urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(geo);
		if (mesh != nullptr){
		  mesh_name = mesh->filename;
		  scale = mesh->scale;
		} 
	      }
	      urdf::MaterialSharedPtr material = link->visual->material;
	      writer.Key("Pos");
	      writer.StartArray();
	      writer.Double(pos.x);
	      writer.Double(pos.y);
	      writer.Double(pos.z);
	      writer.EndArray();
	      writer.Key("Rot");
	      writer.StartObject();  
	      writer.Key("Axis");
	      writer.StartArray();
	      writer.Double(ax);
	      writer.Double(ay);
	      writer.Double(az);
	      writer.EndArray();
	      writer.Key("Angle");
	      writer.Double(angle);
	      writer.EndObject();
	      if( !mesh_name.empty()){
		writer.Key("Mesh");
		writer.String( mesh_name.c_str());
		writer.Key("Scale");
		writer.StartArray();
		writer.Double(scale.x);
		writer.Double(scale.y);
		writer.Double(scale.z);
		writer.EndArray();
		}
	      if(material){
		writer.Key("Material");
		writer.StartArray();
		writer.Double(material->color.r);
		writer.Double(material->color.g);
		writer.Double(material->color.b);
		writer.Double(material->color.a);
		writer.EndArray();
		}
	      }
	      writer.EndObject();
	  }
	  writer.EndArray();
	  writer.EndObject();
	  writer.EndObject();
	}
      
      response = std::make_shared<JsonResponse>(jsonresp);
}
  
void HttpHandler::handlePost(std::shared_ptr<RequestObject>& binary_request){
    
      std::unique_ptr<JsonRequest> getter = std::unique_ptr<JsonRequest>(new JsonRequest(binary_request));
       void* buff = binary_request->GetData();     
       std::cout<<"pos"<<std::string((char*)buff)<<std::endl;
      
      sharedData->clearJointMap();
      
      std::vector<double> vec;
      std::map<int, double> map;
      WebRobotStateRX rstate;
      
      if(uri.compare("/alljoints")==0){     
        
        if(getter->GetDoubleArray("link_position", vec)){       
          sharedData->external_command->add(vec);
          //HACK simulation of holding value for longer time
          sharedData->external_command->add(vec);
        }
        
      }else if(uri.compare("/singlejoint")==0){
        
        //NEW {"joint":[{"id": 15, "pos": 0, , "vel": 0, "eff": 0, "stiff": 0, "damp": 0},{"id": 16, "pos": 0, , "vel": 0, "eff": 0, "stiff": 0, "damp": 0}]}
        //{"joint":[{"id": 15, "val": 0},{"id": 16, "val": 0}]}
       /* if(getter->GetIntDoubleMap("joint", map)){
          for( auto& ref : map){
            sharedData->insertJoint(ref.first,ref.second);
          }
        }*/
        getter->getRobotState(rstate);
        sharedData->setRobotState(rstate);
       
      }else if(uri.compare("/cmd")==0) {
          std::string mess = getter->GetDocument().GetObject()["cmd"].GetString();
          std::string key = getter->GetDocument().GetObject()["Name"].GetString();
          sharedData->insertCmd(key,mess);
      }   
    
}
  