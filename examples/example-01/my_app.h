//
// Created by aldo on 11/23/21.
//

#ifndef MAESTRO_HAPTICS_MY_APP_H
#define MAESTRO_HAPTICS_MY_APP_H

#include <iostream>
#include <fstream>
#include <math.h>
#include <ctime>
#include <string.h>
#include <fcntl.h>
#include "hMaestroHand.h"
#include "hwinterface_esmacat.h"
#include "esmacat_application.h"
#include "finger.h"
#include "digit.h"
#include "thumb.h"


using namespace std;

class my_app : public esmacat_application
{
private:

    // * Communication Setup *//
    void connect();
    void setup();
    void loop();

    // Pointers to the Esmacat drivers
    esmacat_motor_driver* ecat_md1;
    esmacat_motor_driver* ecat_md2;
    esmacat_motor_driver* ecat_md3;
    esmacat_motor_driver* ecat_md4;
    esmacat_motor_driver* ecat_md5;
    esmacat_motor_driver* ecat_md6;
    esmacat_motor_driver* ecat_md7;
    esmacat_motor_driver* ecat_md8;
    esmacat_motor_driver* ecat_array[NUMBER_OF_DRIVERS];

    ofstream myfile;

    // * Hand Exo Structure Setup * //
    int running_cnt;
    double startTime;
    bool run_flag; //false if failed
    HWInterface_Esmacat* pHW_interface;
    Finger* pIndex_finger;
    Finger* pMiddle_finger;
    Thumb* pThumb;

    bool motorsHomed;

public:
    my_app(hMaestroHand* a_hand);
    virtual ~my_app();
    bool run(double time);
    bool ExitManager();
    int kbhit();

    // hMaestroHand pointer
    hMaestroHand* hand;

};
#endif //MAESTRO_HAPTICS_MY_APP_H
