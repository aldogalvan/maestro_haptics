//
// Created by aldo on 12/1/21.
//

#include "hGenericObject.h"

#ifndef MAESTRO_HAPTICS_HWORLD_H
#define MAESTRO_HAPTICS_HWORLD_H

class hWorld : hGenericObject{

public:

    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

    //! Constructor for hWorld
    hWorld();

    //! Destructor for hWorld
    virtual ~hWorld();

public:

    //--------------------------------------------------------------------------
    // PUBLIC METHODS - BACKGROUND PROPERTIES
    //--------------------------------------------------------------------------


    //! This method computes collisions between all objects in this world
    bool computeCollisions();


};

#endif //MAESTRO_HAPTICS_HWORLD_H
