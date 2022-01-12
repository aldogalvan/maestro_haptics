//
// Created by aldo on 12/1/21.
//

#include <Eigen/Dense>
#include <igl/readOBJ.h>

#ifndef MAESTRO_HAPTICS_HGENERICOBJECT_H
#define MAESTRO_HAPTICS_HGENERICOBJECT_H

class hGenericObject
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor for hGenericObject
    hGenericObject();

    //! Destructor for hGenericObject
    ~hGenericObject();

    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GENERAL
    //--------------------------------------------------------------------------

public:

    //! This method adds a child to the world
    bool addChild(hGenericObject* a_object);

    //! Set the position of this object
    void setGlobalPosition(Eigen::Vector3d a_pos);

    //! This method scales the object
    void scaleGenericObject(double a_scale);

    //! This method translates the object by a vector
    void translateObject(Eigen::Vector3d a_vec);

    //! Computes the centroid position from mesh
    void computeCentroidPosition(void);

    //! This function loads a mesh for this object (.obj currently only supported)
    void loadMeshFromFile(std::string a_fileName);

    //! This function sets the stiffness and damping values
    void setStiffnessandDamping(double a_stiffness, double a_damping);

    //! This method computes the force feedback vector
    Eigen::Vector3d computeReactionForceVector();

    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS - GENERAL
    //--------------------------------------------------------------------------

public:

    //! The position of the centroid of this object
    Eigen::Vector3d h_Pos;

    //! The velocity of this object
    Eigen::Vector3d h_Vel;

    //! The orientation of this object
    Eigen::Vector3d h_Ang;

    //! The angular velocity of this object
    Eigen::Matrix3d h_AngVel;

    //! The Vertices of this object mesh
    Eigen::MatrixXd h_Vertices;

    //! The Faces of this object mesh
    Eigen::MatrixXi h_Faces;

    //! The stiffness of this object
    double h_stiffness;

    //! The damping of this object
    double h_damping;

};


#endif //MAESTRO_HAPTICS_HGENERICOBJECT_H
