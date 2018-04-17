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

#ifndef __X_BOT_HAL_THREAD_H__
#define __X_BOT_HAL_THREAD_H__

#include <XCM/XBotUtils.h>
#include <XBotCore/XBotCore.h>
#include <XCM/XBotThread.h>
#include <memory>


namespace XBot
{
    class HALThread;  
}


class XBot::HALThread : public HALInterface, XBot::Thread_hook
                        
{
public:    
       
    HALThread(const char * config);
    
    virtual ~HALThread();    
    
    virtual void th_init ( void * );
    virtual void th_loop ( void * );
        
    /**
     * @brief Getter for the thread name
     * 
     * @param  void
     * @return std::string the thread name
     */
    std::string get_thread_name(void);    

private:    

    std::string thread_name;    
    bool done; 
    XBot::MatLogger::Ptr _hal_log;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    pthread_mutex_t w_mutex;
    XBot::XBotCoreModel _XBotModel;    
    std::string _urdf_path;
    std::string _srdf_path;
    std::string _joint_map_config;
    std::string _path_to_config;
    std::vector<std::string> _jointNames;
    std::vector<int> _jointId;
    int num_joint;
      
    std::map<int , double> _jointValMap;
    std::map<int , double> _jointRefMap;
    std::map<int , double> _jointVelMap;
    std::map<int , double> _jointVelRefMap;
    std::map<int , double> _jointTorqMap;
    std::map<int , double> _jointTorqRefMap;
    std::map<int , std::vector< double >> _gainMap;
    std::map<int , std::vector< double >> _gainRefMap;
    std::map<int, double> _status_grasp;

    //SHARED DATA
    std::map<int , double> _shjointValMap;
    std::map<int , double> _shjointRefMap;    
    std::map<int , double> _shtorqueRefMap;      
    std::map<int , double> _shtorqueValMap;
    std::map<int , double> _shvelRefMap;      
    std::map<int , double> _shvelValMap;
    std::map<int , std::vector< double >> _shgainMap;
    std::map<int , std::vector< double >> _shgainRefMap;
    
    void init();
    int recv_from_slave();
    int send_to_slave();    
    
    bool parseYAML ( const std::string& path_to_cfg );
    
protected:
    
    void set_thread_name(std::string);
    void set_thread_period(int c_period);
    void set_thread_priority(); 
    virtual void init_internal() = 0;    
    virtual int read() = 0;
    virtual int write() = 0;       
    
    void set_robot_state(const double* jnt, const double* torq, const double* vel, const double* stiff = nullptr, const double* damp = nullptr);
    void get_robot_state(double* jnt, double* torq, double* vel, double* stiff, double* damp);
    void set_robot_jnt_ref(const double* jntref);
    void set_robot_state(const float* jnt, const float* torq, const float* vel, const float* stiff = nullptr, const float* damp = nullptr);
    void get_robot_state(float* jnt, float* torq, float* vel, float* stiff, float* damp);
    void set_robot_jnt_ref(const float* jntref);;
    
    // NOTE IXBotHand getters/setters
    virtual double get_grasp_state(int hand_id);
    virtual bool   grasp(int hand_id, double grasp_percentage);
    
    // NOTE IXBotFT getters
    virtual bool get_ft(int ft_id, std::vector< double >& ft, int channels = 6);
    virtual bool get_ft_fault(int ft_id, double& fault);
    virtual bool get_ft_rtt(int ft_id, double& rtt);
    
    // NOTE IXBotIMU getters
    virtual bool get_imu(int imu_id, std::vector< double >& lin_acc, std::vector< double >& ang_vel, std::vector< double >& quaternion);
    virtual bool get_imu_fault(int imu_id, double& fault);
    virtual bool get_imu_rtt(int imu_id, double& rtt);
    
    //NOTE IXBotJoint getters
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
    
    virtual bool get_vel_ref(int joint_id, double& vel_ref)final;
    
    virtual bool get_tor_ref(int joint_id, double& tor_ref)final;
    
    // NOTE IXBotJoint setters
    virtual bool set_pos_ref(int joint_id, const double& pos_ref) final;
    
    virtual bool set_vel_ref(int joint_id, const double& vel_ref) final;
    
    virtual bool set_tor_ref(int joint_id, const double& tor_ref) final;
    
    virtual bool set_gains(int joint_id, const std::vector<double>& gains) final;
    
    virtual bool set_fault_ack(int joint_id, const double& fault_ack) final;
    
    virtual bool set_ts(int joint_id, const double& ts) final;
    
    virtual bool set_op_idx_aux(int joint_id, const double& op_idx_aux) final;
    
    virtual bool set_aux(int joint_id, const double& aux) final;

};

#endif
