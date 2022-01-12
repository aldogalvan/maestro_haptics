//
// Created by aldo on 12/1/21.
//

#include "hGenericObject.h"

//! Constructor for hGenericObject
hGenericObject::hGenericObject()
{
    h_stiffness = 1;
    h_damping = 1;

    h_Pos.setZero();
}

//! Destructor for this object
hGenericObject::~hGenericObject()
{

}

//! Set global position of this object
void hGenericObject::setGlobalPosition(Eigen::Vector3d a_pos)
{
    // sets position
    h_Pos = a_pos;
}

//! This method scales the object by a set amount
void hGenericObject::scaleGenericObject(double a_scale)
{
    h_Vertices *= a_scale;
    computeCentroidPosition();
}

//! Translate the object
void hGenericObject::translateObject(Eigen::Vector3d a_vec)
{
    for (int i = 0; i < h_Vertices.rows(); i++)
    {
        h_Vertices.row(i) += a_vec.transpose();
    }

    computeCentroidPosition();
}

//! Compute Centroid position of this object
void hGenericObject::computeCentroidPosition()
{

    // sets position
    Eigen::Vector3d pos(0,0,0);

    for (int i = 0; i < h_Vertices.rows(); i++)
    {
        pos += h_Vertices.row(i);
    }

    pos /= h_Vertices.rows();

    h_Pos = pos;
}

//! Load the mesh using libIGL (just .obj)
void hGenericObject::loadMeshFromFile(std::string a_fileName)
{
    // Currently just obj
    igl::readOBJ(a_fileName, h_Vertices, h_Faces);

    // compute the center position
    this->computeCentroidPosition();
}

//! Set the stiffness and damping
void hGenericObject::setStiffnessandDamping(double a_stiffness, double a_damping)
{
    h_stiffness = a_stiffness;
    h_damping = a_damping;
}