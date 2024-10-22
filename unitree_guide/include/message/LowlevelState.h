/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef LOWLEVELSTATE_HPP
#define LOWLEVELSTATE_HPP

#include <iostream>
#include "common/mathTypes.h"
#include "common/mathTools.h"
#include "interface/CmdPanel.h"
#include "common/enumClass.h"

struct MotorState
{
	unsigned int mode;
    float q;
    float dq;
    float ddq;
    float tauEst;

    MotorState(){
        q = 0;
        dq = 0;
        ddq = 0;
        tauEst = 0;
    }
};
// 惯性测量单元
struct IMU
{
    float quaternion[4];    // 四元数 q = w + xi + yj + zk
    float gyroscope[3];     // 陀螺仪
    float accelerometer[3]; // 加速度计

    IMU(){ // 构造函数，全部初始化为0
        for(int i = 0; i < 3; i++){
            quaternion[i] = 0;
            gyroscope[i] = 0;
            accelerometer[i] = 0;
        }
        quaternion[3] = 0;
    }
    // 将四元数转换为旋转矩阵
    RotMat getRotMat(){
        Quat quat; // 四元数，4维向量
        quat << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return quatToRotMat(quat);
    }

    Vec3 getAcc(){
        Vec3 acc;
        acc << accelerometer[0], accelerometer[1], accelerometer[2];
        return acc;
    }

    Vec3 getGyro(){
        Vec3 gyro;
        gyro << gyroscope[0], gyroscope[1], gyroscope[2];
        return gyro;
    }

    Quat getQuat(){
        Quat q;
        q << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return q;
    }
};

struct LowlevelState
{
    IMU imu;
    MotorState motorState[12];
    UserCommand userCmd;
    UserValue userValue;

    Vec34 getQ(){
        Vec34 qLegs;
        for(int i(0); i < 4; ++i){
            qLegs.col(i)(0) = motorState[3*i    ].q;
            qLegs.col(i)(1) = motorState[3*i + 1].q;
            qLegs.col(i)(2) = motorState[3*i + 2].q;
        }
        return qLegs;
    }

    Vec34 getQd(){
        Vec34 qdLegs;
        for(int i(0); i < 4; ++i){
            qdLegs.col(i)(0) = motorState[3*i    ].dq;
            qdLegs.col(i)(1) = motorState[3*i + 1].dq;
            qdLegs.col(i)(2) = motorState[3*i + 2].dq;
        }
        return qdLegs;
    }

    RotMat getRotMat(){
        return imu.getRotMat();
    }

    Vec3 getAcc(){
        return imu.getAcc();
    }

    Vec3 getGyro(){
        return imu.getGyro();
    }

    Vec3 getAccGlobal(){
        return getRotMat() * getAcc();
    }

    Vec3 getGyroGlobal(){
        return getRotMat() * getGyro();
    }

    double getYaw(){
        return rotMatToRPY(getRotMat())(2);
    }

    double getDYaw(){
        return getGyroGlobal()(2);
    }

    void setQ(Vec12 q){
        for(int i(0); i<12; ++i){
            motorState[i].q = q(i);
        }
    }
};

#endif  //LOWLEVELSTATE_HPP