#include <XBotCore-interfaces/XDomainCommunication.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <string>

void fs(const std::string& s);

void fc(const XBot::Command& c);

int main(int argc, char **argv){


    
    XBot::SubscriberIDDP<XBot::Command> sub_str("rt1_to_rt2_string");
    XBot::SubscriberIDDP<Eigen::Vector3d> sub_vec("rt1_to_rt2_vec3d");
    
    
    XBot::PublisherIDDP<Eigen::Vector3d> pub_vec("rt2_to_rt1_vec3d");
    XBot::PublisherIDDP<XBot::Command> pub_str("rt2_to_rt1_string");

    Eigen::Vector3d vec1, vec2;
    std::vector<std::string> strings_to_send = {"HELLO", "I AM", "A HARD-REALTIME 2", "PROCESS", "HOW", "ARE YOU?"};


    for( int it = 0; it < 1e10; it++ ){

        std::cout << "RT 2 looping..." << std::endl;

        vec1.setConstant(it);
        pub_vec.write(vec1);

        pub_str.write(strings_to_send[it%strings_to_send.size()]);

        if(sub_vec.read(vec2)){
            std::cout << "RT 2 process: received vector " << vec2.transpose() << std::endl;
        }


        XBot::Command str;
        if(sub_str.read(str)){
            std::cout << "RT 2 process: received string " << str.str() << std::endl;
        }

        usleep(100000);


    }



}