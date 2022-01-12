//
// Created by aldo on 12/1/21.
//
#include <Eigen/Dense>
#include "hGenericObject.h"

#ifndef MAESTRO_HAPTICS_HUSEROBJECT_H
#define MAESTRO_HAPTICS_HUSEROBJECT_H

class hHapticObject : hGenericObject
{

    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------
    //! Constructor for the hHapticObject
    hHapticObject();

    //! Destructor for hHapticObject
    virtual ~hHapticObject();

    //--------------------------------------------------------------------------
    // PUBLIC METHODS - GENERAL
    //--------------------------------------------------------------------------

    //! This function computes the haptic feedback for hHapticObject
    void computeHapticFeedback();

};
#endif //MAESTRO_HAPTICS_HUSEROBJECT_H
