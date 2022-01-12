//
// Created by aldo on 12/1/21.
//

#include <Eigen/Dense>
#include "hGenericObject.h"
#include "finger.h"
#include "thumb.h"

#ifndef MAESTRO_HAPTICS_HMAESTROHAND_H
#define MAESTRO_HAPTICS_HMAESTROHAND_H

class hMaestroHand : hGenericObject
{

public:

    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESRUCTOR
    //--------------------------------------------------------------------------

    //! Constructor for hMaestroHand
    hMaestroHand(bool a_useIdxFinger, bool a_useMidFinger, bool a_useThumb);

    //! Destructor for hMaestroHand
    virtual ~hMaestroHand();

public:

    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GENERAL
    //--------------------------------------------------------------------------

    //! This method sets the approximate finger dimensions
    // void setFingerDimensions();

    //! This method sets the objects from Maestro for communication
    void setMaestroObjects(Finger* aIdxFinger);

    //! This method sets a mesh for each finger
    void loadFingerMeshFromFile(std::string a_fileName);

    //! This method updates the data from the Maestro class
    bool updateJointAngles(void);

    // compute fake angles for finger exoskeleton (use for debugin)
    Eigen::Vector4d pseudoComputeJointAnglesFinger(double joint_angle_sensor_MCP, double joint_angle_sensor_PIP);
    //! This method computes the actual joint angles from finger exoskeleton angles
    Eigen::Vector4d computeJointAnglesFinger(double joint_angle_sensor_MCP_abad, double joint_angle_sensor_MCP_fe, double joint_angle_sensor_MCP_PIP);

    /*
    //! This method computes the actual joint angles from thumb exoskeleton angles
    Eigen::Vector2d computeJointAnglesThumb(double joint_angle_sensor_MCP, double joint_angle_sensor_CMC_MCP);
    */

    //! This method computes the fingertip position from joint angles
    void computeForwardKinematicsFinger(void);

    //! This method computes the forward kinematics for the thumb
    void computeForwardKinematicsThumb(void);

    //! This method computes the force feedback vector at the fingertip
    Eigen::Vector3d computeReactionForceVector();

    //! This method computes the inverse dynamics for the finger
    Eigen::Vector2d computeInverseDynamics_Finger(Eigen::Vector3d force);

    //! This method commands the desired torque back to esmacat finger class
    void commandExoTorqueFinger(double exo_torque_MCP, double exo_torque_PIP);

    /*
    //! This method commands the desired torque back to esmacat thumb class
    void commandMotorDriverTorqueThumb(double joint_torque_MCP,
                                        double joint_torque_PIP);
                                        */

public:

    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS
    //--------------------------------------------------------------------------

    //! Use fingers?
    bool useIdxFinger;
    bool useMidFinger;
    bool useThumb;

    //! Position of Index Finger
    Eigen::Vector3d hIdxMCPpos;        // MCP joint location
    Eigen::Vector3d hIdxPIPpos;        // PIP joint position
    Eigen::Vector3d hIdxDIPpos;        // DIP joint position
    Eigen::Vector3d hIdxFingerPos;     // Finger-tip position

    /*
    //! Position of Middle Finger
    Eigen::Vector3d hMidPIPpos;        // PIP joint position
    Eigen::Vector3d hMidDIPpos;        // DIP joint position
    Eigen::Vector3d hMidFingerPos;     // Finger-tip position

    //! Position of Thumb
    Eigen::Vector3d hThumbMCPpos;        // PIP joint position
    Eigen::Vector3d hThumbDIPpos;        // DIP joint position
    Eigen::Vector3d hThumbPos;     // Finger-tip position
    */

    //! Last Positions
    Eigen::Vector3d hIdxLastPos;
    Eigen::Vector3d hMidLastPos;
    Eigen::Vector3d hThumbLastPos;


    //! Velocities of the finger
    Eigen::Vector3d hIdxFingerVel;
    Eigen::Vector3d hMidFingerVel;
    Eigen::Vector3d hThumbVel;

    //! Joint angles for each finger
    Eigen::Vector4d hIdxJointAngles;
    Eigen::Vector4d hMidJointAngles;
    Eigen::Vector4d hThumbJointAngles;

    //! Segment lengths for each finger
    Eigen::Vector3d hIdxSegLengths;
    Eigen::Vector3d hMidSegLengths;
    Eigen::Vector2d hThumbSegLengths;

    //! Virtual Joint torques for each digit
    Eigen::Vector3d hIdxJointTorques;
    Eigen::Vector3d hMidJointTorques;
    Eigen::Vector2d hThumbJointTorques;

    /*
    //! Mesh for each finger
    Eigen::MatrixXd* hIdxFingerVertices;
    Eigen::MatrixXd* hMidFingerVertices;
    Eigen::MatrixXd* hThumbVertices;
    Eigen::MatrixXi* hIdxFingerFaces;
    Eigen::MatrixXi* hMidFingerFaces;
    Eigen::MatrixXi* hThumbFaces;
     */

private:

    //--------------------------------------------------------------------------
    // PRIVATE MEMBERS
    //--------------------------------------------------------------------------

    //! Index finger from Maestro library
    Finger* hIdxFinger;

    //! Middle finger from Maestro Library
    Finger* hMidFinger;

    //! Thumb from Maestro Library
    Thumb* hThumb;
};

#endif //MAESTRO_HAPTICS_HMAESTROHAND_H
