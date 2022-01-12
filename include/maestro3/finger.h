/*
 * finger.h
 *
 *  Created on: Jun 23, 2015
 *  Updated on: Jul 12, 2018
 *      Author: Ashish.Lab
 */

#ifndef FINGER_H
#define FINGER_H

#include <iostream>
#include <string>
#include <fstream>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <iostream>

#include "digit.h"
#include "hw_interface.h"
#include "exojointanglesensor.h"
#include "exomotor.h"
#include "exosea.h"

//#ifndef UDP_H
//#include "CUDP.h"
//#endif

//#define VF_SR 5

using namespace std;

class Finger : public Digit
{

protected:

    // set the desired torque
    double exo_desired_torque_MCP;
    double exo_desired_torque_PIP;
    double exo_actual_torque_MCP;
    double exo_actual_torque_PIP;

    // angles of interest
    double ang_sensor_MCP_abad;
    double ang_sensor_MCP_fe;
    double ang_sensor_MCP_PIP;
    double ang_sensor_PIP;
    double ang_sensor_DIP;

    ExoJointAngleSensor exo_joint_angle_sensor_MCP_abad;
    ExoJointAngleSensor exo_joint_angle_sensor_MCP_fe;
    ExoJointAngleSensor exo_joint_angle_sensor_MCP_PIP;
    ExoJointAngleSensor exo_joint_angle_sensor_PIP;
    ExoJointAngleSensor exo_joint_angle_sensor_DIP;
    ExoSEA exo_sea_MCP;
    ExoSEA exo_sea_PIP;
    int exit_angle_change;
    ofstream fileout;
    char filename[200];
    double frequency;
    double tau_A[2];
    double tau_sf[2];
    double tau_phi[2];
    double theta_A[2];
    double theta_sf[2];
    double theta_phi[2];
    int task_id;
//    UDP visual_feedback_channel; // UDP communication channel for visual feedback
//    struct custom_data visual_feedback_data; // Feedback data for visual feedback
//    uint visual_feedback_counter;
//    uint8_t trajectory_transparency;
//    bool visual_feedback_flag;

public:
    Finger(const char* s, HW_Interface* pHW_interface_, const char* filename);
    void AssignIndexOfExoJointAngleSensorArrayHWInterface(int index_exo_joint_angle_sensor_MCP_abd, int index_exo_joint_angle_sensor_MCP_SEA, int index_exo_joint_angle_sensor_btw_SEAs, int index_exo_joint_angle_sensor_PIP_SEA, int index_exo_joint_angle_sensor_DIP_flex);
    void AssignIndexOfExoMotorArrayHWInterface(int index_exo_motor_for_MCP_SEA, int index_exo_motor_for_PIP_SEA);
    //void GetAllJointAngleSensorData(double SensorData[]);
    virtual ~Finger();
    virtual bool task();
    void SetTorqueTaskParameters(const char* subject_name,const char* digit_name,double frequency_,double tau_A_[2], double tau_sf_[2], double tau_phi_[2]);
    void SetMotorPositionTaskParameters(const char* subject_name,const char* digit_name, double frequency, double theta_A_[2], double theta_sf_[2], double theta_phi_[2], bool visual_feedback_flag_index_);
    void SetCalibrationTaskParameters(const char* subject_name, const char* digit_name, double frequency_, double theta_A_[2], double theta_sf_[2], double theta_phi_[2], bool visual_feedback_flag_);

    bool task_sensing();
    bool task_torque();
    bool task_sensor_calibration();
    void setDesiredTorque(double* desired_torque_array);
    void getExoJointAngles(double* joint_angles);       // Retrieve current finger exoskeleton joint angles in radians
    bool ExitFinger();

    //  Output data


};

#endif // FINGER_H
