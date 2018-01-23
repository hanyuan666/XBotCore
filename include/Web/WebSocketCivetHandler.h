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

#ifndef __WEBSOCKET_CIVET_HANDLER_WEBSERVER_H__
#define __WEBSOCKET_CIVET_HANDLER_WEBSERVER_H__

#include "CivetServer.h"
#include <memory>
#include <CircularBuffer.h>
#include <SharedData.h>
#include <WebRobotState.h>
#include <Request.h>


#include <XBotCore-interfaces/XDomainCommunication.h>
#include <eigen_conversions/eigen_msg.h>

class WebRobotStateTX;
class SharedData;

class WebSocketHandler : public CivetWebSocketHandler {

  public:
    
    WebSocketHandler(std::shared_ptr<Buffer<WebRobotStateTX>> buffer, std::shared_ptr<SharedData> sharedData){
      
      this->buffer = buffer;
      this->sharedData = sharedData;    
      
      _pipe_names.push_back("w_T_left_ee_ref");
      _pipe_names.push_back("w_T_right_ee_ref");
    
      /*for(std::string pipe_name : _pipe_names){
        _pub_nrt.push_back(XBot::PublisherNRT<Eigen::Affine3d>(pipe_name));
    }*/
    }
           
    virtual bool handleConnection(CivetServer *server, const struct mg_connection *conn);

    virtual void handleReadyState(CivetServer *server, struct mg_connection *conn);

    virtual bool handleData(CivetServer *server,
                            struct mg_connection *conn,
                            int bits,
                            char *data,
                            size_t data_len);

    virtual void handleClose(CivetServer *server, const struct mg_connection *conn);

  private:
    std::shared_ptr<Buffer<WebRobotStateTX>> buffer;
    std::shared_ptr<SharedData> sharedData;
    
    std::vector<std::string> _pipe_names;
    std::vector<XBot::PublisherNRT<Eigen::Affine3d>> _pub_nrt;
};

#endif //__WEBSOCKET_CIVET_HANDLER_WEBSERVER_H__
