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

#include <XBotCore/HALThread.h>

XBot::HALThread::HALThread(const char * config)   
{
    _path_to_config = XBot::Utils::getXBotConfig();
    int c_period = 1;
    done = false;      
    set_thread_name("R-HAL");    
    set_thread_period(c_period);   
    set_thread_priority();    
}

bool XBot::HALThread::parseYAML ( const std::string& path_to_cfg )
{
    std::ifstream fin(path_to_cfg);
    if (fin.fail()) {
        std::cerr << "ERROR in " << __func__ << "! Can NOT open " << path_to_cfg << "!" << std::endl;
        return false;
    }

    YAML::Node root_cfg = YAML::LoadFile(path_to_cfg);
    YAML::Node x_bot_interface;
    if(root_cfg["XBotInterface"]) {
        x_bot_interface = root_cfg["XBotInterface"];
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : YAML file  " << path_to_cfg << "  does not contain XBotInterface mandatory node!!" << std::endl;
        return false;
    }

    // check the urdf_filename
    if(x_bot_interface["urdf_path"]) {
        XBot::Utils::computeAbsolutePath(x_bot_interface["urdf_path"].as<std::string>(),
                            "/",
                            _urdf_path);
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : XBotInterface node of  " << path_to_cfg << "  does not contain urdf_path mandatory node!!" << std::endl;
        return false;
    }

    // check the srdf_filename
    if(x_bot_interface["srdf_path"]) {
        XBot::Utils::computeAbsolutePath(x_bot_interface["srdf_path"].as<std::string>(),
                            "/",
                            _srdf_path);
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : XBotInterface node of  " << path_to_cfg << "  does not contain srdf_path mandatory node!!" << std::endl;
        return false;
    }

    // check joint_map_config
    if(x_bot_interface["joint_map_path"]) {
        XBot::Utils::computeAbsolutePath(x_bot_interface["joint_map_path"].as<std::string>(),
                            "/",
                            _joint_map_config);
    }
    else {
        std::cerr << "ERROR in " << __func__ << " : XBotInterface node of  " << path_to_cfg << "  does not contain joint_map_path mandatory node!!" << std::endl;
        return false;
    }

}

void XBot::HALThread::set_thread_name(std::string thread_name)
{
    // save the thread name
    this->thread_name = thread_name;
    // set thread name
    name = this->thread_name.c_str();
}

std::string XBot::HALThread::get_thread_name(void)
{
    return thread_name;
}

void XBot::HALThread::set_thread_period(int c_period)
{
    // set thread period - not periodic
    task_period_t t;
    memset(&t, 0, sizeof(t));
    t.period = {0,c_period};
    period.task_time = t.task_time;
    period.period = t.period;
}

void XBot::HALThread::set_thread_priority()
{

    // set scheduler policy
#if defined( __XENO__ ) || defined( __COBALT__ )
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    
    // set scheduler priority and stacksize
    priority = sched_get_priority_max(schedpolicy);
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
}

void XBot::HALThread::init(){
  
     // load config file
    YAML::Node root = YAML::LoadFile(_path_to_config);   
    
    // parse the YAML file to initialize internal variables
    parseYAML(_path_to_config);
    // initialize the model
    if (!_XBotModel.init(_urdf_path, _srdf_path, _joint_map_config)) {
        printf("ERROR: model initialization failed, please check the urdf_path and srdf_path in your YAML config file.\n");
        return;
    }
    // generate the robot
    _XBotModel.generate_robot();
    _XBotModel.get_enabled_joint_names(_jointNames);
    _XBotModel.get_enabled_joint_ids(_jointId);
    num_joint = _jointId.size();
    
    for( int n : _jointId){
      _jointValMap[n] = 0.0;
      _jointRefMap[n] = 0.0;
      _jointVelRefMap[n] = 0.0;
      _jointTorqMap[n] = 0.0;
      _jointTorqRefMap[n] = 0.0;
      _gainMap[n] = {0, 0, 0, 0, 0};
      _gainRefMap[n] = {0, 0, 0, 0, 0};
      
      _shjointValMap[n] = 0.0;
      _shjointRefMap[n] = 0.0;
      _shtorqueRefMap[n] = 0.0;
      _shtorqueValMap[n] = 0.0;
      _shgainMap[n] = {0, 0, 0, 0, 0};
      _shgainRefMap[n] = {0, 0, 0, 0, 0};
    }
    
    for( auto& pair : _XBotModel.get_hands()){      
      std::string hand_name= pair.first;
      int hand_id = pair.second;
      _status_grasp[hand_id] = 0.0;      
    }    
    
    pthread_mutex_init(&w_mutex, NULL);
    pthread_mutex_init(&mutex, NULL);
    pthread_cond_init( &cond, NULL);
    this->create(true,1);
    recv_from_slave();
}

int XBot::HALThread::recv_from_slave(){
  
    pthread_mutex_lock( &mutex );
    while(!done){
        pthread_cond_wait( & cond, & mutex ); 
    }
    done = false;
    _jointValMap = _shjointValMap;
    _jointVelMap = _shvelValMap;
    _jointTorqMap = _shtorqueValMap;
    _gainMap = _shgainMap;
    pthread_mutex_unlock( & mutex );     
}

int XBot::HALThread::send_to_slave(){
  
    pthread_mutex_lock( &w_mutex );
    _shjointRefMap = _jointRefMap;
    _shtorqueRefMap = _jointTorqRefMap;
    _shvelRefMap = _jointVelRefMap;
    _shgainRefMap = _gainRefMap;
    pthread_mutex_unlock( &w_mutex );   
}

void XBot::HALThread::th_init( void * ){
    
    Logger::info() << "INIT R-HAL thread"<< Logger::endl();
    _hal_log = XBot::MatLogger::getLogger("/tmp/hal_log");
    init_internal();
}

void XBot::HALThread::th_loop( void * ){  
   
    int resp = read();
    write();
    pthread_mutex_lock( &mutex );
    pthread_cond_signal( &cond ); 
    done = !resp;
    pthread_mutex_unlock( & mutex );
    _hal_log->add("hal_time", XBot::get_time_ns() / 1e9 );
}

XBot::HALThread::~HALThread() {
    
    Logger::info() << "~HALThread()" << Logger::endl();
    this->stop();
    this->join();
}

void XBot::HALThread::get_robot_state(double* jnt, double* torq, double* vel, double* stiff, double* damp)
{
    pthread_mutex_lock( &w_mutex );
    for( int n=0; n<num_joint; n++){
      if(jnt != nullptr)
        jnt[n] = _shjointRefMap[n];
      if(torq != nullptr)
        torq[n] = _shtorqueRefMap[n];
      if(vel != nullptr)
        vel[n] = _shvelRefMap[n];
      if(stiff != nullptr)
        stiff[n] = _shgainMap[n][0];
      if(damp != nullptr)
        damp[n] = _shgainMap[n][1];
    }
    pthread_mutex_unlock( & w_mutex );  
}

void XBot::HALThread::set_robot_state(const double* jnt, const double* torq, const double* vel, const double* stiff, const double* damp)
{
    pthread_mutex_lock( &mutex );    
    for (int t=0; t<num_joint; t++){
      if(jnt != nullptr)
        _shjointValMap[t] = jnt[t];
      if(torq != nullptr)
        _shtorqueValMap[t] = torq[t];
      if( vel != nullptr)
        _shvelValMap[t] = vel[t];
      if(stiff!= nullptr)
      _shgainRefMap[t][0] = stiff[t];
      if(damp!= nullptr)
      _shgainRefMap[t][1] = damp[t];
    }   
    pthread_mutex_unlock( &mutex );  
}

void XBot::HALThread::set_robot_jnt_ref(const double* jntref)
{
   if (jntref != nullptr)
    for (int t=0; t<num_joint; t++){
      _shjointRefMap[t] = jntref[t];
      _jointRefMap[t] = jntref[t];
    } 
}

void XBot::HALThread::get_robot_state(float* jnt, float* torq, float* vel, float* stiff, float* damp)
{
    pthread_mutex_lock( &w_mutex );
    for( int n=0; n<num_joint; n++){
      if(jnt != nullptr)
        jnt[n] = _shjointRefMap[n];
      if(torq != nullptr)
        torq[n] = _shtorqueRefMap[n];
      if(vel != nullptr)
        vel[n] = _shvelRefMap[n];
      if(stiff != nullptr)
        stiff[n] = _shgainMap[n][0];
      if(damp != nullptr)
        damp[n] = _shgainMap[n][1];
    }
    pthread_mutex_unlock( & w_mutex );  
}

void XBot::HALThread::set_robot_state(const float* jnt, const float* torq, const float* vel, const float* stiff, const float* damp)
{
    pthread_mutex_lock( &mutex );    
    for (int t=0; t<num_joint; t++){
      if(jnt != nullptr)
        _shjointValMap[t] = jnt[t];
      if(torq != nullptr)
        _shtorqueValMap[t] = torq[t];
      if( vel != nullptr)
        _shvelValMap[t] = vel[t];
      if(stiff!= nullptr)
      _shgainRefMap[t][0] = stiff[t];
      if(damp!= nullptr)
      _shgainRefMap[t][1] = damp[t];
    }   
    pthread_mutex_unlock( &mutex );  
}

void XBot::HALThread::set_robot_jnt_ref(const float* jntref)
{
    if (jntref != nullptr)
      for (int t=0; t<num_joint; t++){
        _shjointRefMap[t] = jntref[t];
        _jointRefMap[t] = jntref[t];
      } 
}
////////////////////////////////////
////////////////////////////////////
// SINGLE JOINT PRIVATE FUNCTIONS //
////////////////////////////////////
////////////////////////////////////

bool XBot::HALThread::get_link_pos(int joint_id, double& link_pos)
{
    link_pos = _jointValMap[joint_id];
    return true;   
}

bool XBot::HALThread::get_motor_pos(int joint_id, double& motor_pos)
{
    motor_pos = _jointValMap[joint_id];
    return true; 
}

bool XBot::HALThread::get_link_vel(int joint_id, double& link_vel)
{
    link_vel = _jointVelMap[joint_id];
    return true;  
}

bool XBot::HALThread::get_motor_vel(int joint_id, double& motor_vel)
{
    motor_vel = _jointVelMap[joint_id];
    return true;  
}

bool XBot::HALThread::get_torque(int joint_id, double& torque)
{
    torque = _jointTorqMap[joint_id];    
    return true; 
}

bool XBot::HALThread::get_temperature(int joint_id, double& temperature)
{
    return false; 
}

bool XBot::HALThread::get_fault(int joint_id, double& fault)
{
    return false; 
}

bool XBot::HALThread::get_rtt(int joint_id, double& rtt)
{
    return false;   
}

bool XBot::HALThread::get_op_idx_ack(int joint_id, double& op_idx_ack)
{
    return false;   
}

bool XBot::HALThread::get_aux(int joint_id, double& aux)
{
    return false;
}

bool XBot::HALThread::get_gains(int joint_id, std::vector< double >& gain_vector)
{
    gain_vector = _gainMap[joint_id];            
    return true; 
}
   
bool XBot::HALThread::get_vel_ref(int joint_id, double& vel_ref) {
  
    vel_ref = _jointVelRefMap[joint_id];
    return true;
}
    
bool XBot::HALThread::get_tor_ref(int joint_id, double& tor_ref) {
  
    tor_ref = _jointTorqRefMap[joint_id];
    return true;
}

bool XBot::HALThread::set_pos_ref(int joint_id, const double& pos_ref)
{
    _jointRefMap[joint_id] = pos_ref;
    return true; 
}

bool XBot::HALThread::set_vel_ref(int joint_id, const double& vel_ref)
{
    _jointVelRefMap[joint_id] = vel_ref;
    return true; 
}

bool XBot::HALThread::set_tor_ref(int joint_id, const double& tor_ref)
{
    _jointTorqRefMap[joint_id] = tor_ref;    
    return true; 
}

bool XBot::HALThread::set_gains(int joint_id, const std::vector<double>& gains)
{
    _gainRefMap[joint_id] = gains;
    return true; 
}
 
bool XBot::HALThread::set_fault_ack(int joint_id, const double& fault_ack)
{       
    return false; 
}

bool XBot::HALThread::set_ts(int joint_id, const double& ts)
{    
    return false; 
}

bool XBot::HALThread::set_op_idx_aux(int joint_id, const double& op_idx_aux)
{
    return false; 
}

bool XBot::HALThread::set_aux(int joint_id, const double& aux)
{    
    return false; 
}

bool XBot::HALThread::get_ft(int ft_id, std::vector< double >& ft, int channels)
{    
    return false;   
}

bool XBot::HALThread::get_ft_fault(int ft_id, double& fault)
{   
    return false;  
}

bool XBot::HALThread::get_ft_rtt(int ft_id, double& rtt)
{
    return false;   
}

bool XBot::HALThread::get_imu(int imu_id, 
                             std::vector< double >& lin_acc, 
                             std::vector< double >& ang_vel, 
                             std::vector< double >& quaternion)
{
    return false;   
}

bool XBot::HALThread::get_imu_fault(int imu_id, double& fault)
{
    return false; 
}

bool XBot::HALThread::get_imu_rtt(int imu_id, double& rtt)
{
    return false; 
}

bool XBot::HALThread::grasp(int hand_id, double grasp_percentage)
{    
    return false; 
}

double XBot::HALThread::get_grasp_state(int hand_id)
{    
    return -1;   
}