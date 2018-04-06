#ifndef __X_BOT_ROBOTCONTROLINTERFACE_H__
#define __X_BOT_ROBOTCONTROLINTERFACE_H__

#include <memory>

class HALInterface {

public:
    
    typedef std::shared_ptr<HALInterface> Ptr;
    
    std::string _hal_name;
    enum HALType { Robot, Simulator, Dummy, NRT};
    HALType _hal_type;
    
    virtual void init() = 0;
    virtual int recv_from_slave() = 0;
    virtual int send_to_slave() = 0;     
    
};

#endif
