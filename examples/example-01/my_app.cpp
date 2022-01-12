//
// Created by aldo on 11/23/21.
//

#include "my_app.h"

my_app::my_app(hMaestroHand* aHand)
{
    // Creates a pointer to hMaestroHand
    hand = aHand;

    cout<< "Constructing my project..." << endl;
    cout<< "My project constructed!!!" <<endl;
}

my_app::~my_app(){

    // Deletes the interface
    delete pHW_interface;

    // Deleting basic finger objects
    if(pIndex_finger)
        delete pIndex_finger;
    if(pMiddle_finger)
        delete pMiddle_finger;
    if(pThumb)
        delete pThumb;

    cout << "My project Destructed!!!" <<endl;
}

void my_app::connect(){

    cout <<"Connecting to EtherCAT..." <<endl;

    // Assigns the slaves
    assign_esmacat_slave_index(ecat_md1,0);
    assign_esmacat_slave_index(ecat_md2,1);
    assign_esmacat_slave_index(ecat_md3,2);
    assign_esmacat_slave_index(ecat_md4,3);
    assign_esmacat_slave_index(ecat_md5,4);
    assign_esmacat_slave_index(ecat_md6,5);
    assign_esmacat_slave_index(ecat_md7,6);
    assign_esmacat_slave_index(ecat_md8,7);

    // Stores slaves in an array
    ecat_array[0] = ecat_md1;
    ecat_array[1] = ecat_md2;
    ecat_array[2] = ecat_md3;
    ecat_array[3] = ecat_md4;
    ecat_array[4] = ecat_md5;
    ecat_array[5] = ecat_md6;
    ecat_array[6] = ecat_md7;
    ecat_array[7] = ecat_md8;

    // Initializes the interface
    pHW_interface = new HWInterface_Esmacat();
    pHW_interface->esmacatInterfaceConnect(ecat_array);
}

void my_app::setup(){

    cout <<"Setting up EtherCAT..."<<endl;

    running_cnt = 0;
    startTime = 0.0;
    run_flag = true;
    motorsHomed = false; //false;

    pHW_interface->esmacatInterfaceSetup();
    pHW_interface->UpdateAllSensorData();

    cout<< "Sensor Data Updated..."<<endl;

    // Basic Finger assignment and initialization
    /// Initializing index finger
    if (hand->useIdxFinger)
    {
        pIndex_finger = new Finger("index_finger", pHW_interface, "index_finger_torque.csv");
        pIndex_finger->AssignIndexOfExoJointAngleSensorArrayHWInterface(7, 6, 8, 9,
                                                                        10); //ABAD, MCP FE, MCP-PIP, PIP FE, DIP
        pIndex_finger->AssignIndexOfExoMotorArrayHWInterface(4, 5); //MCP FE, PIP FE
    }
    /*
    /// Initializing middle finger
    if (hand->useMidFinger) {
        pMiddle_finger = new Finger("middle_finger", pHW_interface, "middle_finger_torque.csv");
        pMiddle_finger->AssignIndexOfExoJointAngleSensorArrayHWInterface(12, 11, 13, 14, 15);
        pMiddle_finger->AssignIndexOfExoMotorArrayHWInterface(6, 7);
    }

    /// Initializing thumb
    if (hand->useThumb) {
        pThumb = new Thumb("thumb", pHW_interface);
        pThumb->AssignIndexOfExoJointAngleSensorArrayHWInterface(0, 1, 2, 3, 4, 5);
        pThumb->AssignIndexOfExoMotorArrayHWInterface(0, 1, 2, 3);
    }
     */

    // Pass values to hMaestroHaptics
    hand->setMaestroObjects(pIndex_finger);

    cout <<"Hand Structure Assembled" << endl;
}


void my_app::loop(){

    //TODO: Implement high level control (e.g. AAN, Impedance)
    run_flag = run_flag && !kbhit(); //check for keystroke or other stop condition triggered by program


    //
    if(!motorsHomed && run_flag){
        // motorsHomed = pHW_interface->homeMotors();
        // paria's addition to not home motor 8
        motorsHomed = pHW_interface->homeMotors_but_eight(); // modified to only home 5&6
        startTime = elapsed_time_ms;
    }
    else if(run_flag){

        running_cnt++;
        //	cout << "manager running..." << endl;
        pHW_interface->UpdateAllSensorData();

        // Running basic finger task
        if (pIndex_finger)
        {
            pIndex_finger->task();
        }
        /*
        if (pMiddle_finger)
        {
            pMiddle_finger->task();
        }
        if (pThumb)
        {
            pThumb->task();
        }
         */

        pHW_interface->UpdateAllOutputData();

    }
    else{
        ExitManager();
    }

}


bool my_app::ExitManager(){

    // Exit fingers
    if (pIndex_finger)
    {
        pIndex_finger->ExitFinger();
    }
    if (pMiddle_finger)
    {
        pMiddle_finger->ExitFinger();
    }
    if (pThumb)
    {
        pThumb->ExitThumb();
    }

    stop();

    return true;
}

// keyboard input reading
int my_app::kbhit(){
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    return select(1, &fds, NULL, NULL, &tv);
}