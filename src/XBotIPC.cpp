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

#include <XCM/XBotIPC.h>

#include <XBotInterface/Utils.h>

#include <sys/stat.h>
#include <fcntl.h>

XBot::XBotIPC::XBotIPC(std::string config_file) : _config_file(config_file)
{
    std::ifstream fin(config_file);
    if ( fin.fail() ) {
        DPRINTF("Can not open %s\n", config_file.c_str());
    }

    const YAML::Node& x_bot_core_config = YAML::LoadFile(config_file)["XBotInterface"];

    urdf_path = x_bot_core_config["urdf_path"].as<std::string>();
    urdf_path = XBot::Utils::computeAbsolutePath(urdf_path);

    srdf_path = x_bot_core_config["srdf_path"].as<std::string>();
    srdf_path = XBot::Utils::computeAbsolutePath(srdf_path);

    joint_map_config = x_bot_core_config["joint_map_path"].as<std::string>();
    joint_map_config = XBot::Utils::computeAbsolutePath(joint_map_config);

    // initialize the model
    if( !model.init( urdf_path, srdf_path, joint_map_config ) ) {
        DPRINTF("ERROR: model initialization failed, please check the urdf_path and srdf_path in your YAML config file.\n");
    }

    // generate the robot
    model.generate_robot();
    // get the robot
    robot = model.get_robot();
    // get the ft
    ft = model.get_ft_sensors();
    // get the imu
    imu = model.get_imu_sensors();
    // get the hand
    hand = model.get_hands();
}

std::map< std::string, std::vector< int > > XBot::XBotIPC::get_robot_map()
{
    return model.get_robot();
}

std::map< std::string, int > XBot::XBotIPC::get_ft_sensors_map()
{
    return model.get_ft_sensors();
}


XBot::XBotCoreModel XBot::XBotIPC::get_robot_model()
{
    return model;
}

bool XBot::XBotIPC::get_link_pos(int joint_id, double& link_pos)
{
    link_pos = pdo_motor.at(joint_id)->RobotStateRX.link_pos;
    return true;
}

bool XBot::XBotIPC::get_motor_pos(int joint_id, double& motor_pos)
{
    motor_pos = pdo_motor.at(joint_id)->RobotStateRX.motor_pos;
    return true;
}

bool XBot::XBotIPC::get_link_vel(int joint_id, double& link_vel)
{
    link_vel = pdo_motor.at(joint_id)->RobotStateRX.link_vel;
    return true;
}

bool XBot::XBotIPC::get_motor_vel(int joint_id, double& motor_vel)
{
    motor_vel = pdo_motor.at(joint_id)->RobotStateRX.motor_vel;
    return true;
}

bool XBot::XBotIPC::get_torque(int joint_id, double& torque)
{
    torque = pdo_motor.at(joint_id)->RobotStateRX.torque;
    return true;
}

bool XBot::XBotIPC::get_temperature(int joint_id, double& temperature)
{
    temperature = pdo_motor.at(joint_id)->RobotStateRX.temperature;
    return true;
}

bool XBot::XBotIPC::get_fault(int joint_id, double& fault)
{
    fault = pdo_motor.at(joint_id)->RobotStateRX.fault;
    return true;
}

bool XBot::XBotIPC::get_rtt(int joint_id, double& rtt)
{
    rtt = pdo_motor.at(joint_id)->RobotStateRX.rtt;
    return true;
}

bool XBot::XBotIPC::get_op_idx_ack(int joint_id, double& op_idx_ack)
{
    op_idx_ack = pdo_motor.at(joint_id)->RobotStateRX.op_idx_ack;
    return true;
}

bool XBot::XBotIPC::get_aux(int joint_id, double& aux)
{
    aux = pdo_motor.at(joint_id)->RobotStateRX.aux;
    return true;
}

bool XBot::XBotIPC::get_gains(int joint_id, std::vector< double >& gain_vector)
{
    // resize the gain vector
    gain_vector.resize(5);
    gain_vector[0] = pdo_motor.at(joint_id)->RobotStateTX.gain_0;
    gain_vector[1] = pdo_motor.at(joint_id)->RobotStateTX.gain_1;
    gain_vector[2] = pdo_motor.at(joint_id)->RobotStateTX.gain_2;
    gain_vector[3] = pdo_motor.at(joint_id)->RobotStateTX.gain_3;
    gain_vector[4] = pdo_motor.at(joint_id)->RobotStateTX.gain_4;
    return true;
}

bool XBot::XBotIPC::get_pos_ref(int joint_id, double& pos_ref)
{
    pos_ref = pdo_motor.at(joint_id)->RobotStateTX.pos_ref;
    return true;
}

bool XBot::XBotIPC::get_vel_ref(int joint_id, double& vel_ref)
{
    vel_ref = pdo_motor.at(joint_id)->RobotStateTX.vel_ref;
    return true;
}

bool XBot::XBotIPC::get_tor_ref(int joint_id, double& tor_ref)
{
    tor_ref = pdo_motor.at(joint_id)->RobotStateTX.tor_ref;
    return true;
}


bool XBot::XBotIPC::set_pos_ref(int joint_id, const double& pos_ref)
{
    pdo_motor.at(joint_id)->RobotStateTX.pos_ref = pos_ref;
    //DPRINTF("joint : %d - set pos ref : %f\n", joint_id, pdo_motor.at(joint_id)->RobotStateTX.pos_ref);
    return true;
}

bool XBot::XBotIPC::set_vel_ref(int joint_id, const double& vel_ref)
{
    pdo_motor.at(joint_id)->RobotStateTX.vel_ref = vel_ref;
    return true;
}

bool XBot::XBotIPC::set_tor_ref(int joint_id, const double& tor_ref)
{
    pdo_motor.at(joint_id)->RobotStateTX.tor_ref = tor_ref;
    return true;
}

bool XBot::XBotIPC::set_gains(int joint_id, const std::vector< double >& gains)
{
    if(gains.size() == 5) {
        pdo_motor.at(joint_id)->RobotStateTX.gain_0 = gains[0];
        pdo_motor.at(joint_id)->RobotStateTX.gain_1 = gains[1];
        pdo_motor.at(joint_id)->RobotStateTX.gain_2 = gains[2];
        pdo_motor.at(joint_id)->RobotStateTX.gain_3 = gains[3];
        pdo_motor.at(joint_id)->RobotStateTX.gain_4 = gains[4];
    }
    return true;
}

bool XBot::XBotIPC::set_fault_ack(int joint_id, const double& fault_ack)
{
    pdo_motor.at(joint_id)->RobotStateTX.fault_ack = fault_ack;
    return true;
}

bool XBot::XBotIPC::set_ts(int joint_id, const double& ts)
{
    pdo_motor.at(joint_id)->RobotStateTX.ts = ts;
    return true;
}

bool XBot::XBotIPC::set_op_idx_aux(int joint_id, const double& op_idx_aux)
{
    pdo_motor.at(joint_id)->RobotStateTX.op_idx_aux = op_idx_aux;
    return true;
}

bool XBot::XBotIPC::set_aux(int joint_id, const double& aux)
{
    pdo_motor.at(joint_id)->RobotStateTX.aux = aux;
    return true;
}

bool XBot::XBotIPC::get_ft(int ft_id, std::vector< double >& ft, int channels)
{
    ft.resize(channels);
    std::memcpy(ft.data(), &(pdo_FT.at(ft_id)->force_X), channels*sizeof(double));

    return true;
}

bool XBot::XBotIPC::get_ft_fault(int ft_id, double& fault)
{
    fault = pdo_FT.at(ft_id)->fault;
    return true;
}

bool XBot::XBotIPC::get_ft_rtt(int ft_id, double& rtt)
{
    rtt =  pdo_FT.at(ft_id)->rtt;
    return true;
}

bool XBot::XBotIPC::get_imu(int imu_id,
                             std::vector< double >& lin_acc,
                             std::vector< double >& ang_vel,
                             std::vector< double >& quaternion)
{
    
    if(quaternion.size() == 4) {
        quaternion.at(0) = pdo_IMU.at(imu_id)->quat_X;
        quaternion.at(1) = pdo_IMU.at(imu_id)->quat_Y;
        quaternion.at(2) = pdo_IMU.at(imu_id)->quat_Z;
        quaternion.at(3) = pdo_IMU.at(imu_id)->quat_W;
    }

    if(ang_vel.size() == 3 ) {
        ang_vel.at(0) = pdo_IMU.at(imu_id)->ang_vel_X;
        ang_vel.at(1) = pdo_IMU.at(imu_id)->ang_vel_Y;
        ang_vel.at(2) = pdo_IMU.at(imu_id)->ang_vel_Z;
    }

    if(lin_acc.size() == 3 ) {
        lin_acc.at(0) = pdo_IMU.at(imu_id)->lin_acc_X;
        lin_acc.at(1) = pdo_IMU.at(imu_id)->lin_acc_Y;
        lin_acc.at(2) = pdo_IMU.at(imu_id)->lin_acc_Z;
    }

    return true;

}

bool XBot::XBotIPC::get_imu_fault(int imu_id, double& fault)
{
    fault = pdo_IMU.at(imu_id)->fault;
    return true;
}

bool XBot::XBotIPC::get_imu_rtt(int imu_id, double& rtt)
{
    rtt = pdo_IMU.at(imu_id)->rtt;
    return true;
}

bool XBot::XBotIPC::grasp(int hand_id, double grasp_percentage)
{
    // HACK 12.0 is assumed as maximum position range
    pdo_motor.at(hand_id)->RobotStateTX.pos_ref = -1 + grasp_percentage * 12.0;
    return true;
}

double XBot::XBotIPC::get_grasp_state(int hand_id)
{
    double grasp_state = 0.0;
    double link_pos = pdo_motor.at(hand_id)->RobotStateRX.link_pos;

    if( link_pos != 0.0) {
        // HACK 12.0 is assumed as maximum position range
        grasp_state = (1 + link_pos) / 12.0;
    }
    return  grasp_state;
}


XBot::XBotIPC::~XBotIPC()
{
//     Logger::info() << "~XBotIPC()" << Logger::endl();
}

