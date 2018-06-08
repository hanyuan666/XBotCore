/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Alessio Margan, Arturo Laurenzi, Luca Muratore
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

#ifndef __X_BOT_ESC_H__
#define __X_BOT_ESC_H__

#include <stdint.h>

namespace XBot {

struct McEscPdoTypes {
                    
    // TX  slave_input -- master output
    struct pdo_tx {
        float       pos_ref;      
        int16_t     vel_ref;    
        int16_t     tor_ref;    
        uint16_t    gain_0;      
        uint16_t    gain_1;     
        uint16_t    gain_2;     
        uint16_t    gain_3;     
        uint16_t    gain_4;     
        uint16_t    fault_ack;
        uint16_t    ts;
        uint16_t    op_idx_aux;  
        float       aux;         

    } pdo_data_tx __attribute__ ( ( __packed__ ) ); // 28 bytes

    // RX  slave_output -- master input
    struct pdo_rx {
        float        link_pos;          
        float        motor_pos;         
        int16_t      link_vel;          
        int16_t      motor_vel;         
        float        torque;            
        uint16_t     temperature;       
        uint16_t     fault;
        uint16_t     rtt;               
        uint16_t     op_idx_ack;        
        float        aux;               

    } pdo_data_rx __attribute__ ( ( __packed__ ) ); // 28 bytes

}; // 56 bytes

struct Ft6EscPdoTypes {
    
    // TX  slave_input -- master output
    struct pdo_tx {
        uint16_t    ts;
        
    }  __attribute__ ( ( __packed__ ) );

    // RX  slave_output -- master input
    struct pdo_rx {
        float       force_X;            // N
        float       force_Y;            // N
        float       force_Z;            // N
        float       torque_X;           // Nm
        float       torque_Y;           // Nm
        float       torque_Z;           // Nm
        uint16_t    fault;
        uint16_t    rtt;                // ns
        
    }  __attribute__ ( ( __packed__ ) );

};

struct ImuEscPdoTypes {

    // TX  slave_input -- master output
    struct pdo_tx {
        uint16_t    fault_ack;
        uint16_t    digital_out;
        uint16_t    ts;
    } __attribute__ ( ( __packed__ ) );

    // RX  slave_output -- master input
    struct pdo_rx {
        float       x_rate;       
        float       y_rate;       
        float       z_rate;       
        float       x_acc;       
        float       y_acc;       
        float       z_acc;       
        float       x_quat;       
        float       y_quat;       
        float       z_quat;       
        float       w_quat;       
        uint32_t    imu_ts;          
        uint16_t    temperature;          
        uint16_t    digital_in;          
        uint16_t    fault;
        uint16_t    rtt;                // us
        
     }  __attribute__ ( ( __packed__ ) );
     
};

struct HeriHandEscPdoTypes {
                    
    // TX  slave_input -- master output
    struct pdo_tx {
        float       pos_ref;    // rad   
        float       pos_ref_2;  // rad   
        int16_t     vel_ref;    // mrad/s 
        int16_t     tor_ref;    // mNm
        uint16_t    gain_0;      
        uint16_t    gain_1;     
        uint16_t    gain_2;     
        uint16_t    gain_3;     
        uint16_t    gain_4;     
        uint16_t    fault_ack;
        uint16_t    ts;
        uint16_t    op_idx_aux;  // op [get/set] , idx
        float       rx_aux;         // set value
        
    }  __attribute__ ( ( __packed__ ) ); // 32 bytes

    // RX  slave_output -- master input
    struct pdo_rx {
        float        motor_pos;     // rad
        float        motor_pos_2;   // rad
        int16_t      motor_vel;     // mrad/s
        int16_t      motor_vel_2;   // mrad/s
        uint16_t     m1_an_1;       // motor 1 analog 1
        uint16_t     m1_an_2;       // motor 1 analog 2
        uint16_t     m1_an_3;       // motor 1 analog 3
        uint16_t     m2_an_1;       // motor 2 analog 1
        uint16_t     m2_an_2;       // motor 2 analog 2
        uint16_t     m2_an_3;       // motor 2 analog 3
        int16_t      m1_curr;       // mA ?
        int16_t      m2_curr;       // mA ?
        uint16_t     fault;
        uint16_t     rtt;           // us
        uint16_t     op_idx_ack;    // op [ack/nack] , idx
        float        tx_aux;        // get value or nack erro code
        
    }  __attribute__ ( ( __packed__ ) ); // 38 bytes

}; // ... 70 bytes




struct sdo_info {
    float min_pos;
    float max_pos;
    uint16_t ctrl_status_cmd;
};


struct RobotState {
                    
    // TX  slave_input -- master output
    struct pdo_tx {
        double    pos_ref;      
        double    vel_ref;    
        double    tor_ref;    
        double    gain_0;      
        double    gain_1;     
        double    gain_2;     
        double    gain_3;     
        double    gain_4;     
        double    fault_ack;
        double    ts;
        double    op_idx_aux;  
        double    aux;         

    } RobotStateTX __attribute__ ( ( __packed__ ) ); 

    // RX  slave_output -- master input
    struct pdo_rx {
        double    link_pos;          
        double    motor_pos;         
        double    link_vel;          
        double    motor_vel;         
        double    torque;            
        double    temperature;       
        double    fault;
        double    rtt;               
        double    op_idx_ack;        
        double    aux;               

    } RobotStateRX __attribute__ ( ( __packed__ ) ); 

};

struct RobotFT {
    
    // TX  slave_input -- master output
    struct pdo_tx {
        double    ts;
        
    } FTTX __attribute__ ( ( __packed__ ) );

    // RX  slave_output -- master input
    struct pdo_rx {
        double    force_X;            // N
        double    force_Y;            // N
        double    force_Z;            // N
        double    torque_X;           // Nm
        double    torque_Y;           // Nm
        double    torque_Z;           // Nm
        double    fault;
        double    rtt;                // ns
        
    } FTRX __attribute__ ( ( __packed__ ) );

};

struct RobotIMU {
    
    // TX  slave_input -- master output
    struct pdo_tx {
        double    fault_ack;
        double    digital_out;
        double    ts;
    } IMUTX __attribute__ ( ( __packed__ ) );

    // RX  slave_output -- master input
    struct pdo_rx {
        double       ang_vel_X;       
        double       ang_vel_Y;       
        double       ang_vel_Z;       
        double       lin_acc_X;       
        double       lin_acc_Y;       
        double       lin_acc_Z;       
        double       quat_X;       
        double       quat_Y;       
        double       quat_Z;       
        double       quat_W;       
        double       imu_ts;          
        double       temperature;          
        double       digital_in;          
        double       fault;
        double       rtt;                // us
        
    } IMURX __attribute__ ( ( __packed__ ) );

};

struct RobotHERIHand {
    
    // TX  slave_input -- master output
    struct pdo_tx {
        double    pos_ref;    // rad   
        double    pos_ref_2;  // rad   
        double    vel_ref;    // mrad/s 
        double    tor_ref;    // mNm
        double    gain_0;      
        double    gain_1;     
        double    gain_2;     
        double    gain_3;     
        double    gain_4;     
        double    fault_ack;
        double    ts;
        double    op_idx_aux;  // op [get/set] , idx
        double    rx_aux;         // set value
    } HERIHandTX __attribute__ ( ( __packed__ ) );

    // RX  slave_output -- master input
    struct pdo_rx {
        double     motor_pos;     // rad
        double     motor_pos_2;   // rad
        double     motor_vel;     // mrad/s
        double     motor_vel_2;   // mrad/s
        double     m1_an_1;       // motor 1 analog 1
        double     m1_an_2;       // motor 1 analog 2
        double     m1_an_3;       // motor 1 analog 3
        double     m2_an_1;       // motor 2 analog 1
        double     m2_an_2;       // motor 2 analog 2
        double     m2_an_3;       // motor 2 analog 3
        double     m1_curr;       // mA ?
        double     m2_curr;       // mA ?
        double     fault;
        double     rtt;           // us
        double     op_idx_ack;    // op [ack/nack] , idx
        double     tx_aux;        // get value or nack erro code
        
    } HERIHandRX __attribute__ ( ( __packed__ ) );

};

}


#endif /* __X_BOT_ESC_H__ */
