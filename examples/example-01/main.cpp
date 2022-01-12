#include <iostream>
#include "my_app.h"
#include "hMaestroHand.h"
#include "hGenericObject.h"
#include "hThread.h"

// INCLUDE EIGEN
#include <Eigen/Dense>

// INCLUDE IGL
#include <igl/opengl/glfw/Viewer.h>

// The initialized objects
hMaestroHand* hand;
hGenericObject* sphere;

// Basic variables
double stiffness = 0.22; // 0.22 [N/mm] Foam (220 N/m)
double damping = 0.1;  //
double r_sphere = 20;
double r_hand = 0.25;
Eigen::Vector3d force(0,0,0);

/*
// Inline mesh of a cube
const Eigen::MatrixXd V = (Eigen::MatrixXd(8,3)<<
                                              0.0,0.0,0.0,
        0.0,0.0,1.0,
        0.0,1.0,0.0,
        0.0,1.0,1.0,
        1.0,0.0,0.0,
        1.0,0.0,1.0,
        1.0,1.0,0.0,
        1.0,1.0,1.0).finished();
const Eigen::MatrixXi F = (Eigen::MatrixXi(12,3)<<
                                                1,7,5,
        1,3,7,
        1,4,3,
        1,2,4,
        3,8,7,
        3,4,8,
        5,7,8,
        5,8,6,
        1,5,6,
        1,6,2,
        2,6,8,
        2,8,4).finished().array()-1;
*/

// Main haptics thread thread
hThread* hapticsThread;

// IGL viewer
igl::opengl::glfw::Viewer* viewer;

// Esmacat application
my_app* application;

// a flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// a flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// Function that updates the application haptics
void updateHaptics(void);

// close the simulation
void close(void);

int main() {

    // Initialize the objects in the world
    hand = new hMaestroHand(true,false,false);
    sphere = new hGenericObject();

    // Initialize esmacat application first
    application = new my_app(hand);
    application->set_ethercat_adapter_name_through_terminal();
    application->start();

    // Load the mesh
    string mesh_filename = "/home/aldo/CLionProjects/maestro-haptics/examples/example-01/assets/sphere.obj";
    sphere->loadMeshFromFile(mesh_filename);
    // Will just render a point
    // hand->loadFingerMeshFromFile(mesh_filename);

    // Set variable values for object
    sphere->setStiffnessandDamping(stiffness, damping);
    sphere->scaleGenericObject(r_sphere);
    Eigen::Vector3d vec(50,25,0);
    sphere->translateObject(vec);


    // create a thread which starts the main haptics rendering loop
    hapticsThread = new hThread();
    hapticsThread->start(updateHaptics, THREAD_PRIORITY_HAPTICS);

    // Closing function
    atexit(close);

    // intialize the viewer
    viewer = new igl::opengl::glfw::Viewer;
    viewer->core().is_animating = true;

    // callback to update the graphics
    viewer->callback_pre_draw = [&](igl::opengl::glfw::Viewer & )->bool
    {
        // Clears the current meshes
        viewer->data().clear();

        // Adds the mesh for the object
        viewer->data().set_mesh(sphere->h_Vertices,sphere->h_Faces);

        // Adds a point for the finger
        Eigen::MatrixXd points(4,3);
        points.row(0) = hand->hIdxFingerPos.transpose();
        points.row(1) = hand->hIdxDIPpos.transpose();
        points.row(2) = hand->hIdxPIPpos.transpose();
        points.row(3) = hand->hIdxMCPpos.transpose();
        viewer->data().set_points(points,Eigen::RowVector3d(1,0,0.5));

        // Adds a line for the virtual force
        viewer->data().add_edges(hand->hIdxFingerPos.transpose(),
                                 hand->hIdxFingerPos.transpose() + 5*force.transpose(),Eigen::RowVector3d(1,0,0));


        // Continue
        return false;
    };

    // Adds the mesh for the object
    viewer->data().set_mesh(sphere->h_Vertices,sphere->h_Faces);

    // launches the viewer
    viewer->launch();

    return(0);
}

//! Main haptic rendering loop
void updateHaptics(void)
{
    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    while(simulationRunning)
    {

        //finger_index->updateJointSensorAngles();
        //cout << finger_index->jointAngles << endl;

        // print hand values
        if(hand->updateJointAngles()) {

            //! Very basic collision detector
            // find distance between hand and object
            double dist = (hand->hIdxFingerPos - sphere->h_Pos).norm();

            // find the unit vector connecting them
            Eigen::Vector3d uVec = (hand->hIdxFingerPos - sphere->h_Pos) / dist;

            // find point on finger object closest to sphere
            //Eigen::Vector3d proxyPos = 0.1*hand->hIdxFingerPos - uVec * r_hand;

           // std::cout << sphere->h_Pos.transpose() << endl;
           force.setZero();

            // check whether this point lies within the sphere
            if ((hand->hIdxFingerPos-sphere->h_Pos).norm()<r_sphere)
            {
                Eigen::Vector3d godPos = hand->hIdxFingerPos;
                Eigen::Vector3d proxyPos = uVec*r_sphere;
                double penetrationDepth = (godPos - proxyPos).norm();
                force += uVec*penetrationDepth*sphere->h_stiffness;
                //force += uVec*hand->hIdxFingerVel.norm()*sphere->h_damping;
            }

            // Compute Exo Torques
            Eigen::Vector2d exoTorques = hand->computeInverseDynamics_Finger(-1*force);

            // Send Desired Exo Torque commands
             hand->commandExoTorqueFinger(exoTorques(0), exoTorques(1));

        }
    }

    // exit haptics thread
    simulationFinished = true;
}

//! Key callback function
// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // option - exit
    if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }
}

//! Exit handler
void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) {  sleep(2); }

    // Application closing
    std::cout << "Application closing!" << endl;
    std::cout.flush();

    // Delete all threads
    if (hapticsThread)
        delete hapticsThread;
    if(application)
        delete application;
    if(viewer)
        delete viewer;
    if(hand)
        delete hand;
    if (sphere)
        delete sphere;

}


//! ==================================================================
//! MAESTRO INVERSE/FORWARD KINEMATICS/DYNAMICS FUNCTIONS
//! ==================================================================