/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
   
*/

/**
 *
 * @author Luca Muratore (2016-, luca.muratore@iit.it)
*/

#ifndef __X_BOT_PLUGIN_HPP__
#define __X_BOT_PLUGIN_HPP__

#include <memory>

#include <XBotCore/XBotCore.h>
#include <XBotCore/IXBotModel.h>

namespace XBot
{
    class XBotPlugin;
}

/**
 * @brief TBD
 * 
 */
class XBot::XBotPlugin
{
    
public:   
    
    XBotPlugin(XBot::IXBotModel* model, XBot::IXBotChain* chain) : model(model), chain(chain)
    {
        
    };
    
    virtual ~XBotPlugin() {};

    
    virtual bool init(void) = 0;
    
    virtual void run(void) = 0;
    
    virtual bool close(void) = 0;
    
protected:
    
    std::shared_ptr<XBot::IXBotModel> model;
    std::shared_ptr<XBot::IXBotChain> chain;
    


};

#endif //__X_BOT_PLUGIN_HPP__