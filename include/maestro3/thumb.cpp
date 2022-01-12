/*
 * thumb.cpp
 *
 *  Created on: Aug 5, 2015
 *  Updated on: Jul 12, 2018
 *      Author: Ashish.Lab
 */

#include "thumb.h"

using namespace std;

Thumb::Thumb(const char* s, HW_Interface* pHW_interface_):Digit(s,pHW_interface_){
    exo_joint_angle_sensor_CMC_abad.SetHWInterface(pHW_interface_);
    exo_joint_angle_sensor_CMC_fe.SetHWInterface(pHW_interface_);
    exo_joint_angle_sensor_CMC_MCP.SetHWInterface(pHW_interface_);
    exo_joint_angle_sensor_MCP.SetHWInterface(pHW_interface_);
    exo_joint_angle_sensor_MCP_IP.SetHWInterface(pHW_interface_);
    exo_joint_angle_sensor_IP.SetHWInterface(pHW_interface_);

    //	exo_sea_CMC_fe.SetSEAParameters(1160,0.1,0,0,-M_PI/6,M_PI/5); // N/m (Mc master: 9435K84) // 11.4 mm initial length between clamps - compressed to 7.5 mm
    //	exo_sea_CMC_abad.SetSEAParameters(1160,0.1,0,0,-M_PI/5,M_PI/5); // N/m (Mc master: 9435K84)
    //	exo_sea_MCP.SetSEAParameters(1160,0.5,0,0,-M_PI/6,M_PI/4); // N/m (Mc master: 9435K84) 1.2
    //	exo_sea_IP.SetSEAParameters(1160*0.751,0.5,0,0,-M_PI/5,M_PI/5); // N/m (Mc master: 9435K78) Change the stiffness

    exo_sea_CMC_fe.SetSEAParameters(5630*0.3,0.3,0,0,-M_PI/6,M_PI/6,0.025/2); // N/m (Mc master: 9435K84) // 11.4 mm initial length between clamps - compressed to 7.5 mm
    exo_sea_CMC_abad.SetSEAParameters(5630*0.3,0.2,0,0,-M_PI/5,M_PI/5,0.025/2); // N/m (Mc master: 9435K84)
    exo_sea_MCP.SetSEAParameters(5630*0.3,0.3,0,0,-M_PI/4,M_PI/4,0.024/2); // N/m (Mc master: 9435K84) 1.2
    exo_sea_IP.SetSEAParameters(5630*0.5*0.51,0.5,0,0,-M_PI/5,M_PI/5,0.024/2); // N/m (Mc master: 9435K78) Change the stiffness

    //	exo_sea_CMC_fe.SetSEAParameters(8936*0.3,0.3,0,0,-M_PI/6,M_PI/6,0.025/2); // N/m (Mc master: 94125K614) // 11.4 mm initial length between clamps - compressed to 7.5 mm
    //	exo_sea_CMC_abad.SetSEAParameters(5630*0.3,0.2,0,0,-M_PI/5,M_PI/5,0.025/2); // N/m (Mc master: 9435K84)
    //	exo_sea_MCP.SetSEAParameters(5630*0.3,0.3,0,0,-M_PI/4,M_PI/4,0.024/2); // N/m (Mc master: 9435K84) 1.2
    //	exo_sea_IP.SetSEAParameters(5630*0.5*0.51,0.5,0,0,-M_PI/5,M_PI/5,0.024/2); // N/m (Mc master: 9435K78) Change the stiffness

    // Best zero torque
    //	exo_sea_CMC_fe.SetSEAParameters(5630*0.3,0.3,0,0,-M_PI/6,M_PI/6,0.025/2); // N/m (Mc master: 9435K84) // 11.4 mm initial length between clamps - compressed to 7.5 mm
    //	exo_sea_CMC_abad.SetSEAParameters(5630*0.3,0.2,0,0,-M_PI/5,M_PI/5,0.025/2); // N/m (Mc master: 9435K84)
    //	exo_sea_MCP.SetSEAParameters(5630*0.3,0.3,0,0,-M_PI/4,M_PI/4,0.024/2); // N/m (Mc master: 9435K84) 1.2
    //	exo_sea_IP.SetSEAParameters(5630*0.5*0.51,0.5,0,0,-M_PI/5,M_PI/5,0.024/2); // N/m (Mc master: 9435K78) Change the stiffness

    exit_angle_change = 3;
    cout << s << " Thumb is constructed" << endl;
    //	filename=(char *)(s);
    frequency = 0.25;
}


Thumb::~Thumb() {
    // TODO Auto-generated destructor stub

    fileout.close();
}
// Exoskeleton joint sensor data in degrees
bool Thumb::task_sensing(){

    ang_sensor_CMC_abad = exo_joint_angle_sensor_CMC_abad.GetExoJointAngleRad()*180/M_PI; // [deg]
    ang_sensor_CMC_fe = exo_joint_angle_sensor_CMC_fe.GetExoJointAngleRad()*180/M_PI;     // [deg]
    ang_sensor_CMC_MCP = exo_joint_angle_sensor_CMC_MCP.GetExoJointAngleRad()*180/M_PI;   // [deg]
    ang_sensor_MCP = exo_joint_angle_sensor_MCP.GetExoJointAngleRad()*180/M_PI;           // [deg]
    ang_sensor_MCP_IP = exo_joint_angle_sensor_MCP_IP.GetExoJointAngleRad()*180/M_PI;     // [deg]
    ang_sensor_IP = exo_joint_angle_sensor_IP.GetExoJointAngleRad()*180/M_PI;             // [deg]

    return true;
}



bool Thumb::task_torque(){
    // Torque control task
    // Torque Command
    double exo_desired_torque_CMC_fe, exo_desired_torque_CMC_abad, exo_desired_torque_MCP, exo_desired_torque_IP;
    double exo_actual_torque_CMC_fe, exo_actual_torque_CMC_abad, exo_actual_torque_MCP, exo_actual_torque_IP;
    double desired_motor_angle_rad_CMC_fe, desired_motor_angle_rad_CMC_abad, desired_motor_angle_rad_MCP, desired_motor_angle_rad_IP;

    // Sinusoidal Torque Trajectory

    // Exoskeleton CMC F/E Joint Feed-forward PID Torque control
    double tau_A_CMC_fe=tau_A[0]; // 0.2   	// A is amplitude
    double tau_A_CMC_abad=tau_A[1]; // 0.2
    double tau_A_MCP=tau_A[2]; // 0.2
    double tau_A_IP=tau_A[3]; // 0.1
    double tau_sf_CMC_fe=tau_sf[0], tau_sf_CMC_abad=tau_sf[1], tau_sf_MCP=tau_sf[2], tau_sf_IP=tau_sf[3];
    double tau_phi_CMC_fe=tau_phi[0], tau_phi_CMC_abad=tau_phi[1], tau_phi_MCP=tau_phi[2], tau_phi_IP=tau_phi[3]; // phi: phase

    return true;
}
bool Thumb::task_sensor_calibration(){

    return true;
}

bool Thumb::task(){

    bool flag;
    switch(task_id)
    {

    case SENSE: {
        flag = task_sensing();
        break;
    }

    case TORQUE:	{
        flag = task_torque();
        break;
    }
    case CALIBRATION: {
        flag = task_sensor_calibration();
    }
    }
    return flag;
}


void Thumb::AssignIndexOfExoJointAngleSensorArrayHWInterface(int thumb_exo_joint_angle_sensor_CMC_fe,
        int thumb_exo_joint_angle_sensor_CMC_abad,
        int thumb_exo_joint_angle_sensor_CMC_MCP,
        int thumb_exo_joint_angle_sensor_MCP,
        int thumb_exo_joint_angle_sensor_MCP_IP,
        int thumb_exo_joint_angle_sensor_IP){
    exo_joint_angle_sensor_CMC_fe.AssignIndexOfExoJointAngleSensorInHWInterface(thumb_exo_joint_angle_sensor_CMC_fe);
    exo_joint_angle_sensor_CMC_abad.AssignIndexOfExoJointAngleSensorInHWInterface(thumb_exo_joint_angle_sensor_CMC_abad);
    exo_joint_angle_sensor_CMC_MCP.AssignIndexOfExoJointAngleSensorInHWInterface(thumb_exo_joint_angle_sensor_CMC_MCP);
    exo_joint_angle_sensor_MCP.AssignIndexOfExoJointAngleSensorInHWInterface(thumb_exo_joint_angle_sensor_MCP);
    exo_joint_angle_sensor_MCP_IP.AssignIndexOfExoJointAngleSensorInHWInterface(thumb_exo_joint_angle_sensor_MCP_IP);
    exo_joint_angle_sensor_IP.AssignIndexOfExoJointAngleSensorInHWInterface(thumb_exo_joint_angle_sensor_IP);

    exo_sea_CMC_fe.SetJointAngleSensorSEA(&exo_joint_angle_sensor_CMC_fe);
    exo_sea_CMC_abad.SetJointAngleSensorSEA(&exo_joint_angle_sensor_CMC_abad);
    exo_sea_MCP.SetJointAngleSensorSEA(&exo_joint_angle_sensor_MCP);
    exo_sea_IP.SetJointAngleSensorSEA(&exo_joint_angle_sensor_IP);
}


void Thumb::AssignIndexOfExoMotorArrayHWInterface(int thumb_exo_motor_for_CMC_fe_SEA,
        int thumb_exo_motor_for_CMC_abad_SEA,
        int thumb_exo_motor_for_MCP_SEA,
        int thumb_exo_motor_for_IP_SEA){
    exo_sea_CMC_fe.SetMotorIndexSEA(pHW_interface, thumb_exo_motor_for_CMC_fe_SEA);
    exo_sea_CMC_abad.SetMotorIndexSEA(pHW_interface, thumb_exo_motor_for_CMC_abad_SEA);
    exo_sea_MCP.SetMotorIndexSEA(pHW_interface, thumb_exo_motor_for_MCP_SEA);
    exo_sea_IP.SetMotorIndexSEA(pHW_interface, thumb_exo_motor_for_IP_SEA);
}


bool Thumb::ExitThumb(){
    if (exo_sea_CMC_fe.ExitSEA(exit_angle_change) && exo_sea_CMC_abad.ExitSEA(exit_angle_change) && exo_sea_MCP.ExitSEA(exit_angle_change) && exo_sea_IP.ExitSEA(exit_angle_change))
        return true;
    else
        return false;
}


void Thumb::SetTorqueTaskParameters(char* subject_name,double frequency_,double tau_A_[4], double tau_sf_[4], double tau_phi_[4]){
    task_id = TORQUE;
    frequency = frequency_;
    for(int i=0;i<4;i++)
    {
        tau_A[i] = tau_A_[i];
        tau_sf[i] = tau_sf_[i];
        tau_phi[i] = tau_phi_[i];
    }
    sprintf(filename,"thumb_%s_%d_f_%lf_tau_A_%lf_%lf_%lf_%lf_tau_sf_%lf_%lf_%lf_%lf_tau_phi_%lf_%lf_%lf_%lf.csv",subject_name,task_id,frequency,
            tau_A[0],tau_A[1],tau_A[2],tau_A[3],tau_sf[0],tau_sf[1],tau_sf[2],tau_sf[3],
            tau_phi[0],tau_phi[1],tau_phi[2],tau_phi[3]);
    fileout.open((const char*)(filename), ios::out);
}


void Thumb::SetMotorPositionTaskParameters(char* subject_name, double frequency_, double theta_A_[4], double theta_sf_[4], double theta_phi_[4]){
    task_id = MOTOR_POSITION;
    frequency = frequency_;
    for(int i=0;i<4;i++)
    {
        theta_A[i] = theta_A_[i];
        theta_sf[i] = theta_sf_[i];
        theta_phi[i] = theta_phi_[i];
    }
    sprintf(filename,"thumb_%s_%d_f_%lf.csv",subject_name,task_id,frequency);
    fileout.open((const char*)(filename), ios::out);
}

void Thumb::SetCalibrationTaskParameters(char* subject_name, double frequency_, double theta_A_[4], double theta_sf_[4], double theta_phi_[4]){
    task_id = CALIBRATION;
    frequency = frequency_;
    for(int i=0;i<4;i++)
    {
        theta_A[i] = theta_A_[i];
        theta_sf[i] = theta_sf_[i];
        theta_phi[i] = theta_phi_[i];
    }
    sprintf(filename,"calib_thumb_%s_%d_f_%lf.csv",subject_name,task_id,frequency);
    fileout.open((const char*)(filename), ios::out);
}

// Set the desired torque to drive to the motors
void Thumb::setDesiredTorque(double *desired_torque_array)
{
    exo_desired_torque_CMC_fe = desired_torque_array[0];
    exo_desired_torque_CMC_abad = desired_torque_array[1];
    exo_desired_torque_MCP = desired_torque_array[2];
    exo_desired_torque_IP = desired_torque_array[3];
}

// Retrieve thumb exoskeleton joint angles in radians
void Thumb::getExoJointAngles(double *exo_joint_angles)
{
    exo_joint_angles[0] = ang_sensor_CMC_abad;
    exo_joint_angles[1] = ang_sensor_CMC_fe;
    exo_joint_angles[1] = ang_sensor_MCP;
    exo_joint_angles[3] = ang_sensor_MCP_IP;
    exo_joint_angles[4] = ang_sensor_CMC_MCP;
    exo_joint_angles[5] = ang_sensor_IP;
}
