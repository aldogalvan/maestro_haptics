//
// Created by aldo on 12/1/21.
//

#include "hGenericObject.h"

#ifndef MAESTRO_HAPTICS_HCOLLISIONDETECTOR_H
#define MAESTRO_HAPTICS_HCOLLISIONDETECTOR_H

class hCollisionDetector
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of hCollisionDetector
    hCollisionDetector(){};

    //! Destructor of hCollisionDetector
    ~hCollisionDetector(){};

public:

    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

    //! This method computes a collision between two objects
    bool computeCollisionDetection(hGenericObject* object1, hGenericObject* object2);


public:

    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------
};

#endif //MAESTRO_HAPTICS_HCOLLISIONDETECTOR_H
