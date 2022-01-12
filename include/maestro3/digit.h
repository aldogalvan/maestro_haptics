/*
 * CDigit.h
 *
 *  Created on: Jul 2, 2015
 *      Author: Ashish.Lab
 */

#ifndef DIGIT_H
#define DIGIT_H

#include <iostream>
#include <string>
#include "hw_interface.h"

#define SENSE -1// Exoskeleton Joint Sensing
#define MOTOR_POSITION 0 // Exoskeleton Motor Position Control
#define TORQUE 1 // Exoskeleton Joint Torque Controller
#define CALIBRATION 2 //Exoskeleton Motor Position Control while writing All sensor voltage values

using namespace std;

class Digit
{

protected:
    HW_Interface* pHW_interface;
    string digit_name;
//	int* kb_input;

public:
//    Digit();
    Digit(string s, HW_Interface* pHW_interface_);
    string GetDigitName(){return digit_name;};
    virtual ~Digit();
    virtual bool task()=0;
//	void SetKbInputPointer(int* kb_input_);
};

#endif // DIGIT_H
