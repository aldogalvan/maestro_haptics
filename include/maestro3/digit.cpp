/*
 * CDigit.cpp
 *
 *  Created on: Jul 2, 2015
 *      Author: Ashish.Lab
 */

#include "digit.h"

Digit::Digit(string s, HW_Interface* pHW_interface_)
{
    // TODO Auto-generated constructor stub
    pHW_interface = pHW_interface_;
    digit_name = s;
}

Digit::~Digit() {
    // TODO Auto-generated destructor stub
}

//void Digit::SetKbInputPointer(int* kb_input_){
//	kb_input = kb_input_;
//}
