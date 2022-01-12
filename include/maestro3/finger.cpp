/*
 * finger.cpp
 *
 *  Created on: Jun 23, 2015
 *  Updated on: Jul 12, 2018
 *      Author: Ashish.Lab
 */

#include "finger.h"

using namespace std;

//#ifndef VISUAL_FEEDBACK_COMPUTER_ADDRESS
//#define VISUAL_FEEDBACK_COMPUTER_ADDRESS "146.6.84.231"
//#endif

Finger::Finger(const char *s, HW_Interface *pHW_interface_, const char *filename) : Digit(s,pHW_interface_){

    exo_joint_angle_sensor_MCP_abad.SetHWInterface(pHW_interface_);
    exo_joint_angle_sensor_MCP_fe.SetHWInterface(pHW_interface_);
    exo_joint_angle_sensor_MCP_PIP.SetHWInterface(pHW_interface_);
    exo_joint_angle_sensor_PIP.SetHWInterface(pHW_interface_);
    exo_joint_angle_sensor_DIP.SetHWInterface(pHW_interface_);

    exo_desired_torque_MCP = 0;
    exo_desired_torque_PIP = 0;

    // LFFC
    exo_sea_MCP.SetSEAParameters(5630*0.3,0.15,0,0,-M_PI/6,M_PI/5,0.025/2); //N/m (Mc master: 9435K84) 800 // 11.4 mm initial length between clamps - compressed to 7.5 mm
    exo_sea_PIP.SetSEAParameters(5630*0.5*0.5,0.05,0,0,-M_PI/6,M_PI/5,0.0234/2); // N/m (Mc master: 9435K78) Full ROM Limits: -M_PI/4,M_PI/5

    // Default SEA Gains: MCP - 0.1,0,0; PIP - 0.05,0,0
    exit_angle_change = 3;
    frequency = 0.25;

    // set task id to sens
    //task_id = SENSE;
    task_id = TORQUE;

    cout << s << " Finger is constructed" << endl;

}

Finger::~Finger() {
    // TODO Auto-generated destructor stub
    fileout.close();
}

bool Finger::task(){

    bool flag;
    switch(task_id)
    {

    case SENSE:{
        flag = task_sensing();
        break;
    }
    case TORQUE:	{
        flag = task_torque();
        break;
    }
    case CALIBRATION: {
        flag =task_sensor_calibration();
        break;
    }
    }
    return flag;
}

// Update finger exoskeleton joint angles in degrees
bool Finger::task_sensing(){

    // Values in degrees (Dec 3 2021)
    ang_sensor_MCP_abad = exo_joint_angle_sensor_MCP_abad.GetExoJointAngleRad();
    ang_sensor_MCP_fe = exo_joint_angle_sensor_MCP_fe.GetExoJointAngleRad();
    ang_sensor_MCP_PIP = exo_joint_angle_sensor_MCP_PIP.GetExoJointAngleRad();
    ang_sensor_PIP = exo_joint_angle_sensor_PIP.GetExoJointAngleRad();
    ang_sensor_DIP = exo_joint_angle_sensor_DIP.GetExoJointAngleRad();

    /*
    std::cout << ang_sensor_MCP_fe << "," << ang_sensor_MCP_abad << "," << ang_sensor_MCP_PIP << endl;
    std::cout.flush();
    */

//    cout<<showpos<<"MCP: "<<exo_joint_angle_sensor_MCP_fe.GetExoJointAngleRad()*180/M_PI<<"\t"
//            <<"PIP: "<<exo_joint_angle_sensor_PIP.GetExoJointAngleRad()*180/M_PI<<endl;

    return true;
}

bool Finger::task_torque(){

    /*
    double desired_motor_angle_rad_MCP, desired_motor_angle_rad_PIP;

    double tau_A_MCP=tau_A[0];
    double tau_A_PIP=tau_A[1];
    double tau_MCP_sf = tau_sf[0];
    double tau_PIP_sf = tau_sf[1];
    double tau_phi_MCP=tau_phi[0];
    double tau_phi_PIP=tau_phi[1];

    // Desired exoskeleton joint torque

    if(!exo_sea_MCP.TorqueControl(exo_desired_torque_MCP, 0, desired_motor_angle_rad_MCP, exo_actual_torque_MCP,1))
        return false;

    if(!exo_sea_PIP.TorqueControl(exo_desired_torque_PIP, 0, desired_motor_angle_rad_PIP, exo_actual_torque_PIP,0))
        return false;
    */
    // Values in degrees (Dec 3 2021)
    ang_sensor_MCP_abad = exo_joint_angle_sensor_MCP_abad.GetExoJointAngleRad();
    ang_sensor_MCP_fe = exo_joint_angle_sensor_MCP_fe.GetExoJointAngleRad();
    ang_sensor_MCP_PIP = exo_joint_angle_sensor_MCP_PIP.GetExoJointAngleRad();
    ang_sensor_PIP = exo_joint_angle_sensor_PIP.GetExoJointAngleRad();
    ang_sensor_DIP = exo_joint_angle_sensor_DIP.GetExoJointAngleRad();

    double desired_motor_angle_rad_MCP, desired_motor_angle_rad_PIP;

    //std::cout << exo_desired_torque_MCP << " & " << exo_desired_torque_PIP << endl;
    //std::cout.flush();

    if(!exo_sea_MCP.TorqueControl(exo_desired_torque_MCP, 0, desired_motor_angle_rad_MCP, exo_actual_torque_MCP,1))
        return false;

    if(!exo_sea_PIP.TorqueControl(exo_desired_torque_PIP, 0, desired_motor_angle_rad_PIP, exo_actual_torque_PIP,0))
        return false;


    return true;
}

bool Finger::task_sensor_calibration()
{

    // commands for moving joints

    return true;
}

void Finger::AssignIndexOfExoJointAngleSensorArrayHWInterface(int index_exo_joint_angle_sensor_MCP_abd,
        int index_exo_joint_angle_sensor_MCP_SEA,
        int index_exo_joint_angle_sensor_btw_SEAs,
        int index_exo_joint_angle_sensor_PIP_SEA,
        int index_exo_joint_angle_sensor_DIP_flex){
    exo_joint_angle_sensor_MCP_abad.AssignIndexOfExoJointAngleSensorInHWInterface(index_exo_joint_angle_sensor_MCP_abd);
    exo_joint_angle_sensor_MCP_fe.AssignIndexOfExoJointAngleSensorInHWInterface(index_exo_joint_angle_sensor_MCP_SEA);
    exo_joint_angle_sensor_MCP_PIP.AssignIndexOfExoJointAngleSensorInHWInterface(index_exo_joint_angle_sensor_btw_SEAs);
    exo_joint_angle_sensor_PIP.AssignIndexOfExoJointAngleSensorInHWInterface(index_exo_joint_angle_sensor_PIP_SEA);
    exo_joint_angle_sensor_DIP.AssignIndexOfExoJointAngleSensorInHWInterface(index_exo_joint_angle_sensor_DIP_flex);

    exo_sea_MCP.SetJointAngleSensorSEA(&exo_joint_angle_sensor_MCP_fe);
    exo_sea_PIP.SetJointAngleSensorSEA(&exo_joint_angle_sensor_PIP);
}

void Finger::AssignIndexOfExoMotorArrayHWInterface(int index_exo_motor_for_MCP_SEA, int index_exo_motor_for_PIP_SEA){
    exo_sea_MCP.SetMotorIndexSEA(pHW_interface, index_exo_motor_for_MCP_SEA);
    exo_sea_PIP.SetMotorIndexSEA(pHW_interface, index_exo_motor_for_PIP_SEA);
}

bool Finger::ExitFinger(){
    if (exo_sea_MCP.ExitSEA(exit_angle_change) && exo_sea_PIP.ExitSEA(exit_angle_change))
        return true;
    else
        return false;
}

// Set the desired torque to drive to the motors
void Finger::setDesiredTorque(double *desired_torque_array)
{
    exo_desired_torque_MCP = desired_torque_array[0];
    exo_desired_torque_PIP = desired_torque_array[1];
    //exo_actual_torque_MCP;
    //exo_actual_torque_PIP;
     std::cout << exo_desired_torque_MCP<<','<< exo_desired_torque_PIP << endl;
     std::cout.flush();

}

// Retrieve current finger exoskeleton joint angles in radians
void Finger::getExoJointAngles(double *exo_joint_angles)
{
    exo_joint_angles[0] = ang_sensor_MCP_abad;  // [rad]
    exo_joint_angles[1] = ang_sensor_MCP_fe;    // [rad]
    exo_joint_angles[2] = ang_sensor_MCP_PIP;   // [rad]
    exo_joint_angles[3] = ang_sensor_PIP;       // [rad]
    exo_joint_angles[4] = ang_sensor_DIP;       // [rad]

    // Debug showing in degrees
    //std::cout << ang_sensor_MCP_abad*180/M_PI << "," << ang_sensor_MCP_fe*180/M_PI << ","<< ang_sensor_MCP_PIP*180/M_PI << endl;
    //std::cout.flush();

}


