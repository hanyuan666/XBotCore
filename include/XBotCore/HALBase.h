#ifndef __X_BOT_HALBASE_H__
#define __X_BOT_HALBASE_H__

#include <XBotCore-interfaces/IXBotJoint.h>
#include <XBotCore-interfaces/IXBotHand.h>
#include <XBotCore/HALInterface.h>

class HALBase : public HALInterface{

public:
    
    typedef std::shared_ptr<HALBase> Ptr;
    
    XBot::IXBotJoint::Ptr getJoint(){      
      return std::dynamic_pointer_cast<XBot::IXBotJoint>(mjoint);
    }
    
    XBot::IXBotHand::Ptr getHandId(int id){      
      return std::dynamic_pointer_cast<XBot::IXBotHand>(mhands[id]);
    }
    
    HALInterface::Ptr getSensorId(int id){      
      return msensors[id];
    }
    
    bool isLoaded(const std::string& val){
      
      if (mlibs[val] == nullptr) return false;
      return true;
    } 
    
    void addLib(const std::string& val,HALInterface::Ptr lib){
      
      mlibs[val] = lib;
    }
    
    void setJoint( HALInterface::Ptr joint){      
      mjoint = joint;
    }
    
    void setHandId(int id, HALInterface::Ptr hand){      
      mhands[id] = hand;
    }
    
    void setSensorId(int id, HALInterface::Ptr sensor){      
      msensors[id] = sensor;
    }
    
    int base_init(){
      
      init();
      init_devices();  
      post_init();
      
      return 0;
    }
    
    int base_recv(){
      
      recv_from_slave();
      recv_devices();   
      
      return 0;
    }
    
    int base_send(){
      
      send_to_slave();
      send_actuators();
     
      return 0;
    }

private:
  
    std::map<int, HALInterface::Ptr> mhands;
    std::map<int, HALInterface::Ptr> msensors;
    std::map<std::string, HALInterface::Ptr> mlibs;
    
    HALInterface::Ptr mjoint;

    virtual void post_init(){};
    
    void init_devices(){
            
      mjoint->init();
      
      for (std::map<std::string, HALInterface::Ptr>::iterator it = mlibs.begin(); it != mlibs.end(); ++it)
      {
        it->second->init();
      }
      
    }
   
    int recv_devices(){
      
      mjoint->recv_from_slave();
      
      for (std::map<std::string, HALInterface::Ptr>::iterator it = mlibs.begin(); it != mlibs.end(); ++it)
      {
        it->second->recv_from_slave();
      }   
      
    }
    int send_actuators(){
      
      mjoint->send_to_slave();
      
      for (std::map<std::string, HALInterface::Ptr>::iterator it = mlibs.begin(); it != mlibs.end(); ++it)
      {
        it->second->send_to_slave();
      }   
      
    }   
    
};

#endif
