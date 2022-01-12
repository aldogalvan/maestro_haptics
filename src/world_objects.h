//
// Created by aldo on 11/24/21.
//

#include <Eigen/Dense>
#include <igl/readOBJ.h>
#include "../include/maestro3/finger.h"
#include "../include/maestro3/thumb.h"

#ifndef MAESTRO_HAPTICS_WORLD_OBJECTS_H
#define MAESTRO_HAPTICS_WORLD_OBJECTS_H

class finger_haptics {

public:
    //! Constructor for finger
    finger_haptics(){};

    //! Destructor for finger
    ~finger_haptics(){};

public:
    //! This function updates the sensor values from the finger class
    void updateJointSensorAngles(void)
    {
        double sensorDataRad[5];
        pFinger->GetAllJointAngleSensorData(sensorDataRad);
        jointAngles(0) = sensorDataRad[0];
        jointAngles(1) = sensorDataRad[1];
        jointAngles(2) = sensorDataRad[2];
        jointAngles(3) = sensorDataRad[3];
        jointAngles(4) = sensorDataRad[4];
    }

    //! This function computes the forward kinematics from the base to the fingertip
    Eigen::Vector3d computeForwardKinematics(void){

    }

    //! This function computes the inverse kinematics from the fingertip to the base
    Eigen::Vector3d computeInverseKinematics(void){

    }

    //! This function computes whether this object is in collision with another object

    bool computeCollision(void){

    }

    //! This function computes any haptic feedback
    Eigen::Vector3d computeHapticFeedback(){

    }

    //! This function sends any computed force data to Maestro
    void commandJointTorque()
    {

    }

public:

    // State Values

    Eigen::Vector3d hand_pos;
    Eigen::Vector3d last_hand_pos;
    Eigen::Vector3d hand_vel;
    Eigen::Vector3d last_hand_vel;
    Eigen::Vector3d finger_pos;
    Eigen::Vector3d last_finger_pos;
    Eigen::Vector3d finger_vel;
    Eigen::Vector3d last_finger_vel;
    Eigen::Matrix3Xf mesh;

    Eigen::Vector3d fingerDimensions;
    Eigen::VectorXd jointAngles;
    double radius;


    // pointer to maestro finger object
    Finger* pFinger;
    const int NUM_SENSORS = 5;

};

class thumb_haptics {
    thumb_haptics(){}

    ~thumb_haptics(){}

    // pointer to maestro thumb object
    Thumb* pThumb;

};

class object {

public:
    //constructor for world object
    object (){

    }

    //destructor for object
    ~object(){};

public:

    //object physical parameters
    double radius;
    double stiffness;
    double damping;

    Eigen::Matrix3Xd V_;
    Eigen::Matrix3Xi F_;

};

#endif //MAESTRO_HAPTICS_WORLD_OBJECTS_H
