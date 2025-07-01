#include <assert.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <string>
#include <sys/mman.h>
#include <sys/timerfd.h>
#include <thread>
#include <unistd.h>
#include <mutex>

#include <ros/ros.h>
#include <ros/duration.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include "motor_cmd_reset.hpp"
#include "motor_state_reset.hpp"

struct njust_MotorState
{
    unsigned int mode;
    float q;
    float dq;
    float ddq;
    float tauEst;

    njust_MotorState()
    {
        q = 0;
        dq = 0;
        ddq = 0;
        tauEst = 0;
    }
};

struct njustLowlevelState
{
    njust_MotorState motorState[20];
};

class NjustLCMReceiver
{
private:
    std::mutex mtx;

public:
    lcm::LCM lcmSub;
    njustLowlevelState lowState_set_offset;
    remake_lcm::motor_state_reset user_motor_state;

    ros::NodeHandle nh;
    ros::Publisher pub_SelfDog_JointState;

    // Motors Zero Bais
    float position_offset[12] =
    {
         0, -0.6109,  1.4835,
        -0,  0.6109, -1.4835,
        -0,  0.6109, -1.4835,
         0, -0.6109,  1.4835
    };


    NjustLCMReceiver() : lcmSub("udpm://239.255.76.67:7667?ttl=1")
    {
        lcmSub.subscribe("motor_state_second_dog", &NjustLCMReceiver::handle_lcmSub, this);
        pub_SelfDog_JointState = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

        motorState_Init();
    }

    ~NjustLCMReceiver() {};

    // LCM CallBack Funcation
    void handle_lcmSub(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const remake_lcm::motor_state_reset *msg)
    {
        mtx.lock();

        user_motor_state = *msg;
        LCM_Sub_Process(&lowState_set_offset);
        motorState_Limitation();
        lcmMsg2JointMsg(&lowState_set_offset);

        // for(size_t i = 0; i < 4; i++)
        // {
        //     std::cout << "motorState[" << 3 * i + 1 << "].q="<< lowState_set_offset.motorState[ 3 * i + 1 ].q << std::endl;
        // }
        // std::cout << "---------------------" << std::endl;

        mtx.unlock();
    }

    // motor state Init
    void motorState_Init()
    {
        //user_motor_state置0
        for (int i = 0; i < 16; i++)
        {
            
            user_motor_state.p[i] =0;
            user_motor_state.v[i] =0;
            user_motor_state.Estau[i] =0;
            user_motor_state.Temperture[i] =0;
        }
    }

    // Limit motor angle
    void motorState_Limitation()
    {
        for (size_t i = 0; i < 12; i++)
        {
            if (fabs(lowState_set_offset.motorState[i].q) > 3.1415926)
            {
                position_offset[i] -= (2 * 3.1415926);
            }
        }
    }

    // Process LCM data from MCU motors
    void LCM_Sub_Process(njustLowlevelState *LowState)
    {
        // leg0
        //(-)
        LowState->motorState[0].q = -(user_motor_state.p[0] + position_offset[0]);
        LowState->motorState[0].dq = -user_motor_state.v[0];
        LowState->motorState[0].tauEst = -user_motor_state.Estau[0];
        //(+)
        LowState->motorState[1].q = (user_motor_state.p[1] + position_offset[1]);
        LowState->motorState[1].dq = user_motor_state.v[1];
        LowState->motorState[1].tauEst = user_motor_state.Estau[1];
        //(+)
        LowState->motorState[2].q = (user_motor_state.p[2] + position_offset[2]);
        LowState->motorState[2].dq = user_motor_state.v[2];
        LowState->motorState[2].tauEst = user_motor_state.Estau[2];

        // leg1
        //(-)
        LowState->motorState[3].q = -(user_motor_state.p[3] + position_offset[3]);
        LowState->motorState[3].dq = -user_motor_state.v[3];
        LowState->motorState[3].tauEst = -user_motor_state.Estau[3];
        //(-)
        LowState->motorState[4].q = -(user_motor_state.p[4] + position_offset[4]);
        LowState->motorState[4].dq = -user_motor_state.v[4];
        LowState->motorState[4].tauEst = -user_motor_state.Estau[4];
        //(-)
        LowState->motorState[5].q = -(user_motor_state.p[5] + position_offset[5]);
        LowState->motorState[5].dq = -user_motor_state.v[5];
        LowState->motorState[5].tauEst = -user_motor_state.Estau[5];

        // leg2
        //(+)
        LowState->motorState[6].q = user_motor_state.p[6] + position_offset[6];
        LowState->motorState[6].dq = user_motor_state.v[6];
        LowState->motorState[6].tauEst = user_motor_state.Estau[6];
        //(+)
        LowState->motorState[7].q = (user_motor_state.p[7] + position_offset[7]);
        LowState->motorState[7].dq = user_motor_state.v[7];
        LowState->motorState[7].tauEst = user_motor_state.Estau[7];
        //(+)
        LowState->motorState[8].q = (user_motor_state.p[8] + position_offset[8]);
        LowState->motorState[8].dq = user_motor_state.v[8];
        LowState->motorState[8].tauEst = user_motor_state.Estau[8];

        // leg3
        //(+)
        LowState->motorState[9].q = user_motor_state.p[9] + position_offset[9];
        LowState->motorState[9].dq = user_motor_state.v[9];
        LowState->motorState[9].tauEst = user_motor_state.Estau[9];
        //(-)
        LowState->motorState[10].q = -(user_motor_state.p[10] + position_offset[10]);
        LowState->motorState[10].dq = -user_motor_state.v[10];
        LowState->motorState[10].tauEst = -user_motor_state.Estau[10];
        //(-)
        LowState->motorState[11].q = -(user_motor_state.p[11] + position_offset[11]);
        LowState->motorState[11].dq = -user_motor_state.v[11];
        LowState->motorState[11].tauEst = -user_motor_state.Estau[11];

        // wheel0(+)
        LowState->motorState[12].q = user_motor_state.p[12];
        LowState->motorState[12].dq = user_motor_state.v[12];
        LowState->motorState[12].tauEst = user_motor_state.Estau[12];
        // wheel1(-)
        LowState->motorState[13].q = user_motor_state.p[13];
        LowState->motorState[13].dq = user_motor_state.v[13];
        LowState->motorState[13].tauEst = user_motor_state.Estau[13];
        // wheel2(+)
        LowState->motorState[14].q = user_motor_state.p[14];
        LowState->motorState[14].dq = user_motor_state.v[14];
        LowState->motorState[14].tauEst = user_motor_state.Estau[14];
        // wheel3(-)
        LowState->motorState[15].q = user_motor_state.p[15];
        LowState->motorState[15].dq = user_motor_state.v[15];
        LowState->motorState[15].tauEst = user_motor_state.Estau[15];

    }

    void lcmMsg2JointMsg(njustLowlevelState *LowState)
    {
        sensor_msgs::JointState joint_data;
        joint_data.header.stamp = ros::Time::now();
        joint_data.name.resize(12);
        joint_data.position.resize(12);
        joint_data.velocity.resize(12);
        joint_data.effort.resize(12);

        // leg0  -- RF
        joint_data.name[0] = "FR_hip_joint";
        joint_data.name[1] = "FR_thigh_joint";
        joint_data.name[2] = "FR_calf_joint";

        // leg1  -- LF
        joint_data.name[3] = "FL_hip_joint";
        joint_data.name[4] = "FL_thigh_joint";
        joint_data.name[5] = "FL_calf_joint";

        // leg2  -- RR
        joint_data.name[6] = "RR_hip_joint";
        joint_data.name[7] = "RR_thigh_joint";
        joint_data.name[8] = "RR_calf_joint";

        // leg3  -- RL
        joint_data.name[9] = "RL_hip_joint";
        joint_data.name[10] = "RL_thigh_joint";
        joint_data.name[11] = "RL_calf_joint";

        for (int i = 0; i < 12; i++)
        {
            joint_data.position[i] = LowState->motorState[i].q;
            joint_data.velocity[i] = LowState->motorState[i].dq;
            joint_data.effort[i]   = LowState->motorState[i].tauEst;
        }

        pub_SelfDog_JointState.publish(joint_data);
    }


    // Thread of LCM Data Process
    void processDataReceiver()
    {
        ros::Rate loop_rate(500);

        while (ros::ok())
        {
            lcmSub.handle();
            loop_rate.sleep();
        }
    }

};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "lcmDataReceiver");

    ROS_INFO("\033[1;32m----> Njust Lcm is Started.\033[0m");
    
    NjustLCMReceiver lcmRec;

    std::thread lcmThread(&NjustLCMReceiver::processDataReceiver, &lcmRec);

    lcmThread.join();

    return 0;
}
