/*
 * thumb.h
 *
 *  Created on: Jun 23, 2015
 *  Updated on: Jul 12, 2018
 *      Author: Ashish.Lab
 */

#ifndef THUMB_H
#define THUMB_H

#include <iostream>
#include <string>
#include <fstream>
#include <stdio.h>
#include <math.h>

#include "digit.h"
#include "hw_interface.h"
#include "exojointanglesensor.h"
#include "exomotor.h"
#include "exosea.h"

using namespace std;

class Thumb : public Digit{

protected:

    double exo_desired_torque_CMC_fe;
    double exo_desired_torque_CMC_abad;
    double exo_desired_torque_MCP;
    double exo_desired_torque_IP;

    double ang_sensor_CMC_abad;
    double ang_sensor_CMC_fe;
    double ang_sensor_CMC_MCP;
    double ang_sensor_MCP;
    double ang_sensor_MCP_IP;
    double ang_sensor_IP;

    ExoJointAngleSensor exo_joint_angle_sensor_CMC_abad;
    ExoJointAngleSensor exo_joint_angle_sensor_CMC_fe;
    ExoJointAngleSensor exo_joint_angle_sensor_CMC_MCP;
    ExoJointAngleSensor exo_joint_angle_sensor_MCP;
    ExoJointAngleSensor exo_joint_angle_sensor_MCP_IP;
    ExoJointAngleSensor exo_joint_angle_sensor_IP;
    ExoSEA exo_sea_CMC_fe;
    ExoSEA exo_sea_CMC_abad;
    ExoSEA exo_sea_MCP;
    ExoSEA exo_sea_IP;
    int exit_angle_change;
    ofstream fileout;
    char filename[200];
    double frequency;
    double tau_A[4];
    double tau_sf[4];
    double tau_phi[4];
    double theta_A[4];
    double theta_sf[4];
    double theta_phi[4];
    int task_id;

public:
    Thumb(const char* s, HW_Interface* pHW_interface_);
    void AssignIndexOfExoJointAngleSensorArrayHWInterface(int thumb_exo_joint_angle_sensor_CMC_abd,
              int thumb_exo_joint_angle_sensor_CMC_fe,
              int thumb_exo_joint_angle_sensor_CMC_MCP,
              int thumb_exo_joint_angle_sensor_MCP,
              int thumb_exo_joint_angle_sensor_MCP_IP,
              int thumb_exo_joint_angle_sensor_IP);
    void AssignIndexOfExoMotorArrayHWInterface(int thumb_exo_motor_for_CMC_fe_SEA,
              int thumb_exo_motor_for_CMC_abad_SEA,
              int thumb_exo_motor_for_MCP_SEA,
              int thumb_exo_motor_for_IP_SEA);

    void SetTorqueTaskParameters(char* subject_name,double frequency_,double tau_A_[4], double tau_sf_[4], double tau_phi_[4]);
    void SetMotorPositionTaskParameters(char* subject_name, double frequency_, double theta_A_[4], double theta_sf_[4], double theta_phi_[4]);
    void SetCalibrationTaskParameters(char* subject_name, double frequency_, double theta_A_[4], double theta_sf_[4], double theta_phi_[4]);
    virtual ~Thumb();
    virtual bool task();
    bool task_sensing();
    bool task_torque();
    bool task_sensor_calibration();
    void setDesiredTorque(double* desired_torque_array);
    void getExoJointAngles(double* exo_joint_angles); // Retrieve thumb exoskeleton joint angles in radians
    bool ExitThumb();
};

#endif // THUMB_H
