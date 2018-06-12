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


#ifndef __X_BOT_IPC_H__
#define __X_BOT_IPC_H__

#include <XBotCore-interfaces/XBotESC.h>
#include <XBotCore-interfaces/IXBotJoint.h>
#include <XBotCore-interfaces/IXBotFT.h>
#include <XBotCore-interfaces/IXBotIMU.h>
#include <XBotCore-interfaces/IXBotHand.h>
#include <XBotCore-interfaces/XDomainCommunication.h>

#include <XBotCoreModel.h>

#include <cstring>


namespace XBot
{
    class XBotIPC;
}


/**
 * @brief XBotCore IPC interface
 * 
 */
class XBot::XBotIPC :  public XBot::IXBotJoint,
                       public XBot::IXBotFT,
                       public XBot::IXBotIMU,
                       public XBot::IXBotHand
                        
{
public:
    
    typedef std::shared_ptr<XBot::XBotIPC> Ptr;
    
    XBotIPC(std::string config_file);
    virtual ~XBotIPC();
    
    std::map<std::string,std::vector<int> > get_robot_map();
    std::map<std::string,int> get_ft_sensors_map();
    XBot::XBotCoreModel get_robot_model();
    
    virtual bool init() = 0;
    
    virtual void updateRX() = 0;
    
    virtual void updateTX() = 0;
    
    // NOTE IXBotJoint getters
    virtual bool get_link_pos(int joint_id, double& link_pos) final;
    
    virtual bool get_motor_pos(int joint_id, double& motor_pos) final;
    
    virtual bool get_link_vel(int joint_id, double& link_vel) final;
    
    virtual bool get_motor_vel(int joint_id, double& motor_vel) final;
    
    virtual bool get_torque(int joint_id, double& torque) final;
    
    virtual bool get_temperature(int joint_id, double& temperature) final;
    
    virtual bool get_fault(int joint_id, double& fault) final;
    
    virtual bool get_rtt(int joint_id, double& rtt) final;
    
    virtual bool get_op_idx_ack(int joint_id, double& op_idx_ack) final;
    
    virtual bool get_aux(int joint_id, double& aux) final;
    
    virtual bool get_gains(int joint_id, std::vector< double >& gain_vector) final;
        
    virtual bool get_pos_ref(int joint_id, double& pos_ref) final;

    virtual bool get_vel_ref(int joint_id, double& vel_ref) final;

    virtual bool get_tor_ref(int joint_id, double& tor_ref) final;
    
    // NOTE IXBotJoint setters
    virtual bool set_pos_ref(int joint_id, const double& pos_ref) final;
    
    virtual bool set_vel_ref(int joint_id, const double& vel_ref) final;
    
    virtual bool set_tor_ref(int joint_id, const double& tor_ref) final;
    
    virtual bool set_gains(int joint_id, const std::vector<double>& gains) final;
    
    virtual bool set_fault_ack(int joint_id, const double& fault_ack) final;
    
    virtual bool set_ts(int joint_id, const double& ts) final;
    
    virtual bool set_op_idx_aux(int joint_id, const double& op_idx_aux) final;
    
    virtual bool set_aux(int joint_id, const double& aux) final;
    
    // NOTE IXBotFT getters
    virtual bool get_ft(int ft_id, std::vector< double >& ft, int channels = 6) final;
    virtual bool get_ft_fault(int ft_id, double& fault) final;
    virtual bool get_ft_rtt(int ft_id, double& rtt) final;
    
    // NOTE IXBotIMU getters
    virtual bool get_imu(int imu_id, std::vector< double >& lin_acc, std::vector< double >& ang_vel, std::vector< double >& quaternion);
    virtual bool get_imu_fault(int imu_id, double& fault);
    virtual bool get_imu_rtt(int imu_id, double& rtt);

    //NOTE IXBotHand
    virtual bool grasp(int hand_id, double grasp_percentage) final;
    virtual double get_grasp_state(int hand_id) final;

protected:

    /**
     * @brief Robot model loaded in XBotCore
     * 
     */
    XBotCoreModel model;
    
    /**
     * @brief The complete path the YAML config file
     * 
     */
    std::string _config_file;
        
    /**
     * @brief The complete path the urdf file to load
     * 
     */
    std::string urdf_path;
    
    /**
     * @brief The complete path the srdf file to load
     * 
     */
    std::string srdf_path;
    
    /**
     * @brief Joint id to joint name map configuration file
     * 
     */
    std::string joint_map_config;

    /**
     * @brief map between the chain name and the id of the enabled joints in the chain 
     * 
     */
    std::map<std::string, std::vector<int>> robot;
    
    /**
     * @brief ft map 
     * 
     */
    std::map<std::string, int> ft;
    
    /**
     * @brief imu map 
     * 
     */
    std::map<std::string, int> imu;
    
    /**
     * @brief hand map 
     * 
     */
    std::map<std::string, int> hand;

    /**
     * @brief number of bytes read from pipes
     * 
     */
    int n_bytes;
    
    /**
     * @brief PDO Motor map
     * 
     */
    std::map<int, std::shared_ptr<XBot::RobotState>> pdo_motor;
    
    /**
     * @brief PDO FT map
     * 
     */
    std::map<int, std::shared_ptr<XBot::RobotFT::pdo_rx>> pdo_FT;
    
    /**
     * @brief PDO IMU map
     * 
     */
    std::map<int, std::shared_ptr<XBot::RobotIMU::pdo_rx>> pdo_IMU;


    /**
     * @brief RobotState struct to read
     * 
     */
    XBot::RobotState _actual_pdo_motor;
    

};

#endif //__X_BOT_IPC_H__
