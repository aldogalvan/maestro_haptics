//
// Created by aldo on 12/1/21.
//

#include "hMaestroHand.h"

/*constants used across multiple functions
 * Finger Proximal Phalanx Length:  double ppl= 45;
 *
 * */

// Constructor for hMaestroHand
hMaestroHand::hMaestroHand(bool a_useIdxFinger, bool a_useMidFinger, bool a_useThumb)
{
    // Set values
    useIdxFinger = a_useIdxFinger;
    useMidFinger = a_useMidFinger;
    useThumb = a_useThumb;

    // Set approximate finger length values
    hIdxSegLengths = Eigen::Vector3d(45,25,20);
    hMidSegLengths = Eigen::Vector3d(1,1,1);
    hThumbSegLengths = Eigen::Vector2d(1,1);


    // Set baseline position
    hIdxMCPpos = Eigen::Vector3d(0,0,0);
    //hMidMCPPos = Eigen::Vector3d(0,0,0);
    //hThumbMCPPos = Eigen::Vector3d(0,0,0);
}

// Destructor for hMaestroHand
hMaestroHand::~hMaestroHand()
{

}

// Function to set the maestro objects (OPTIONAL)
void hMaestroHand::setMaestroObjects(Finger* aIdxFinger)
{
    // Set object values
    hIdxFinger = aIdxFinger;
    /*
    hMidFinger = aMidFinger;
    hThumb = aThumb;
     */
}

// Function to set the mesh
void hMaestroHand::loadFingerMeshFromFile(std::string a_fileName)
{
    // Currently just obj
    Eigen::MatrixX3d V;
    Eigen::MatrixX3i F;
    igl::readOBJ(a_fileName, V, F);

    /*

    // Sets same values for each (will need to compute new vertices)
    if (hIdxFinger)
    {
        *hIdxFingerVertices = V;
        *hIdxFingerFaces = F;
    }

    if (hMidFinger)
    {
        *hMidFingerVertices = V;
        *hMidFingerFaces = F;
    }

    if (hThumb)
    {
        *hMidFingerFaces = F;
        *hThumbFaces = F;
    }

     */
}
Eigen::Vector4d hMaestroHand::pseudoComputeJointAnglesFinger( double joint_angle_sensor_MCP, double joint_angle_sensor_MCP_PIP)
{
    double offset_MCP  = 0*M_PI/180; // [rad]
    double offset_PIP  = 0*M_PI/180; // [rad]
    double DIP_PIP_ratio = 0.77;

    double MCP = 2*(joint_angle_sensor_MCP + offset_MCP);
    double PIP = joint_angle_sensor_MCP_PIP + offset_PIP;
    double DIP = DIP_PIP_ratio*PIP;

    Eigen::Vector4d joint_angles;
    joint_angles = Eigen::Vector4d(0,MCP,PIP,DIP);

     std::cout << MCP*180/M_PI <<','<< PIP*180/M_PI <<','<< DIP*180/M_PI << endl;
     //std::cout << joint_angles << endl;
     std::cout.flush();


    return joint_angles;
}

// Function to compute finger joint angles, inputs are in radians
Eigen::Vector4d hMaestroHand::computeJointAnglesFinger(double joint_angle_sensor_MCP_abad, double joint_angle_sensor_MCP_fe, double joint_angle_sensor_MCP_PIP)
{

    // These values vary depending on the finger and the adjustment of sensor
    double offset_MCP_fe  = -90*M_PI/180; // [rad]
    double offset_MCP_PIP =  45*M_PI/180; // [rad]


    // This value is constant but dependent on the finger
    double DIP_PIP_ratio = 0.77; // coupling factor between PIP  (index finger 0.77, middle finger 0.75) []


    // User Finger Constants, depend on the subject and finger (inter-subject & intra-subject variability)

        // Subject dependent constants for first kinematic loop (crank-slider 4-bar)
        double theta_MGB= 120*M_PI/180; // metacarpal ground to base link a1 angle  [rad]

        double a1 = 40; // Crank-Slider Link a length [mm]
        double b1 = 73; // Crank-Slider Link b length [mm]
        double c1 = 15; // Crank-Slider Link c length [mm]

        double l1= 45; // Proximal Phalanx Length [mm]

        // Subject dependent constants for second kinematic loop (standard 4-bar)
        double a2 = 47; // 4-bar Link a length [mm]
        double b2 = 42; // 4-bar Link b length [mm]
        double c2 = 20; // 4-bar Link c length [mm]

        //! I don't know why but the equations below kind of work but they are definitely wrong lol (to be used as last resource)
        /*
        double theta_3_cs = asin((a1*( M_PI - joint_angle_sensor_MCP_fe - offset_MCP_fe - theta_MGB) - c1)/(a1 - b1))  ;
        double theta_2_cs = asin( ( (b1*sin(theta_3_cs ) + c1) / a1 ) ) ; // [rad] cross circuit
        double d1 =  -(a1*cos(theta_2_cs) - b1*cos(theta_3_cs));
        double joint_angle_FE_MCP =  theta_MGB - theta_2_cs;
        */
    // First kinematic loop  Variables (Cross Circuit Crank Slider)

    double theta_in = joint_angle_sensor_MCP_fe + offset_MCP_fe - M_PI;

        double Z1 = (-b1 + (a1/cos(theta_in)) );
        double Z2 = (tan(theta_in)*a1)/Z1;
        double Z3 = c1/Z1;

        double A1 = pow(a1,2)/pow(Z2,2) + pow(b1,2);
        double B1 = 2*b1*c1 -2*pow(a1,2)*Z3/pow(Z2,2) ;
        double C1 = pow(a1,2)*pow(Z3,2)/pow(Z2,2) - pow(a1,2) + pow(c1,2);

        //! Crank-slider angle 3 [rad]
        // double theta_3_cs = asin(- B1 - sqrt(pow(B1,2)-4*A1*C1) / (2*A1));
        // double theta_3_cs = asin(- B1 + sqrt(pow(B1,2)-4*A1*C1) / (2*A1)); // other solution
        double theta_3_cs = (- B1 - sqrt(pow(B1,2)-4*A1*C1) / (2*A1));

        // Crank-slider angle 2 [rad]
        // double theta_2_cs = asin(-1*((b1*sin(theta_3_cs-M_PI)+c1)/a1)); // [rad] crossed circuit
        double theta_2_cs = asin(-1*((b1*sin(theta_3_cs-M_PI)+c1)/a1));

        // Slider position [mm]
        double d1 =  a1*cos(theta_2_cs) - b1*cos(theta_3_cs);


    // Second Kinematic loop
        // Variable ground offset angle [rad]
        double theta_1_std = atan(c1/(l1-d1));

        // Variable ground Linkage Length [mm]
        double d2 = (l1 - d1)/cos(theta_1_std);

        // angle 2 of standard 4-bar
        double theta_2_std =  theta_1_std + theta_3_cs + joint_angle_sensor_MCP_PIP + offset_MCP_PIP;

        //  Open circuit equations for standard
        double k1 = d2/a2; double k2 = d2/c2; double k3 = (pow(a2,2) - pow(b2,2) + pow(c2,2) + pow(d2,2))/(2*a2*c2);

        double A2 = cos(theta_2_std) - k1 - k2*cos(theta_2_std) + k3;
        double B2 = -2*sin(theta_2_std);
        double C2 = k1 - (k2 + 1)*cos(theta_2_std) + k3;

        double theta_4_std = 2*atan((- B2 - sqrt(pow(B2,2)-4*A2*C2) ) / (2*A2));


    // Compute Joint Angles
    double joint_angle_FE_MCP =  theta_MGB - theta_2_cs;
    double joint_angle_FE_PIP = theta_4_std - theta_1_std;
    double joint_angle_FE_DIP = DIP_PIP_ratio*joint_angle_FE_PIP ;

    Eigen::Vector4d joint_angles;
    joint_angles = Eigen::Vector4d(0,joint_angle_FE_MCP,joint_angle_FE_PIP,joint_angle_FE_DIP);

    // std::cout << (joint_angle_sensor_MCP_fe + offset_MCP_fe)*180/M_PI << endl; // Value should be zero when proximal and metacarpal interfaces align
    // std::cout << theta_in*180/M_PI<< endl; // should read -180 when finger extended
    // std::cout << Z1 << ',' << Z2 << ','<< Z3 << ',' << endl;
    // std::cout << theta_3_cs << endl;  // value becomes nan only when crossing singularity
    // std::cout << theta_3_cs*180/M_PI << ',' << theta_2_cs*180/M_PI << ','<< d1 << ',' << joint_angle_FE_MCP*180/M_PI << endl;
    // std::cout << joint_angle_FE_MCP*180/M_PI<< endl;
    // std::cout << joint_angles << endl;
    //
    // std::cout.flush();

    return joint_angles;
}

/*
// Function to compute actual joint angles
Eigen::Vector4d hMaestroHand::computeJointAnglesThumb(double joint_angle_sensor_MCP, double joint_angle_sensor_CMC_MCP)
{
    Eigen::Vector4d joint_angles;

    return joint_angles;
}
 */

// Function to update the hand joint angles
bool hMaestroHand::updateJointAngles()
{
    bool flag = false;

    if (hIdxFinger)
    {

        double values_idx[5];
        // update the robot sensor values

        hIdxFinger->getExoJointAngles(values_idx); // [rad]
        // compute the finger joint values, inputs in radians
        // hIdxJointAngles = computeJointAnglesFinger(values_idx[0],values_idx[1],values_idx[2]);
        hIdxJointAngles = pseudoComputeJointAnglesFinger(values_idx[1],values_idx[2]);

        computeForwardKinematicsFinger();

        flag = true;
        // Index FInger Exoskeleton Joint Angles [deg]

        //std::cout << hIdxJointAngles << endl;
        //std::cout.flush();

    }

    /*
    if (hMidFinger)
    {
        double values_mid[5];
        hMidFinger->getExoJointAngles(values_mid);
        hMidJointAngles = computeJointAnglesFinger(values_mid[0],values_mid[1],values_mid[2]);
    }

    if (hThumb)
    {
        double values_thumb[6];
        hThumb->getExoJointAngles(values_thumb); // [rad]
        hThumbJointAngles = this->computeJointAnglesThumb(values_thumb[0],values_thumb[1]);
    }
     */
    return flag;

}


// Function to compute forward kinematics (finger joints to finger tip)
void hMaestroHand::computeForwardKinematicsFinger(void)
{
    hIdxLastPos = hIdxFingerPos;

    // User finger Parameters
    double l1 = hIdxSegLengths(0); // 45 [mm]
    double l2 = hIdxSegLengths(1); // 25 [mm]
    double l3 = hIdxSegLengths(2); // 20 [mm]

    // Finger joint angles
    double joint_angle_abad_MCP = hIdxJointAngles(0);
    double joint_angle_fe_MCP   = hIdxJointAngles(1);
    double joint_angle_PIP      = hIdxJointAngles(2);
    double joint_angle_DIP      = hIdxJointAngles(3);

    //! change to matrix operations when integrating MCP abduction and adduction
    // P0: MCP coordinates w.r.t world frame
    double xMCP = hIdxMCPpos(0);
    double yMCP = hIdxMCPpos(1);
    double zMCP = hIdxMCPpos(2);

    // P1: PIP coordinates w.r.t world frame
    double xPIP = xMCP + l1*sin(joint_angle_fe_MCP) ;
    double yPIP = yMCP + l1*cos(joint_angle_fe_MCP) ;
    double zPIP = zMCP ;

    // P2: DIP coordinates with respect to  P0
    double xDIP = xPIP + l2*sin( joint_angle_fe_MCP + joint_angle_PIP) ;
    double yDIP = yPIP + l2*cos( joint_angle_fe_MCP + joint_angle_PIP) ;
    double zDIP = zPIP + 0 ;

    // P3: Finger tip coordinates with respect to  P0
    double xft = xDIP +l3*sin(joint_angle_fe_MCP + joint_angle_PIP + joint_angle_DIP ) ;
    double yft = yDIP +l3*cos(joint_angle_fe_MCP + joint_angle_PIP + joint_angle_DIP ) ;
    double zft = zDIP + 0 ;


    hIdxPIPpos = Eigen::Vector3d(xPIP,yPIP,zPIP) ; // PIP joint position w.r.t World Frame

    hIdxDIPpos = Eigen::Vector3d(xDIP,yDIP,zDIP) ; // DIP joint position w.r.t World Frame

    hIdxFingerPos = Eigen::Vector3d(xft,yft,zft) ; // Finger-tip position w.r.t World Frame

    //hIdxFingerVel = (hIdxFingerPos + hIdxLastPos);

    // std::cout << hIdxMCPpos <<endl; std::cout <<'-'<< endl;
    // std::cout << joint_angle_fe_MCP <<','<<joint_angle_PIP<<','<< joint_angle_DIP << endl;
    // std::cout << xPIP <<','<< yPIP<<','<< zPIP << endl;
    // std::cout << xDIP <<','<< yDIP<<','<< zDIP << endl;
    // std::cout << xft <<','<< yft<<','<< zft << endl;
    // std::cout.flush();
}

// Function to compute forward kinematics
void hMaestroHand::computeForwardKinematicsThumb(void)
{

}

// Function to computer inverse dynamics
Eigen::Vector2d hMaestroHand::computeInverseDynamics_Finger(Eigen::Vector3d force)
{
    Eigen::Vector3d P0 = hIdxMCPpos;
    Eigen::Vector3d P1 = hIdxPIPpos ;
    Eigen::Vector3d P2 = hIdxDIPpos ;
    Eigen::Vector3d P3 = hIdxFingerPos ;

    Eigen::Vector3d L3 = P3 - P2 ;
    Eigen::Vector3d L2 = P2 - P1 ;
    Eigen::Vector3d L1 = P1 - P0 ;

    // to do: dot product to extract torques only in the z axis
    Eigen::Vector3d torques_P2 = L3.cross(force) ;  // + torques_P3 when generalizing
    Eigen::Vector3d torques_P1 = L2.cross(force) + torques_P2 ;
    Eigen::Vector3d torques_P0 = L1.cross(force) + torques_P1 ;

    //
    double joint_torque_MCP = 0.001*torques_P0.dot(Eigen::Vector3d  (0,0,1)); //   [N m]
    double joint_torque_PIP = 0.001*torques_P1.dot(Eigen::Vector3d  (0,0,1)); //   [N m]
    double joint_torque_DIP = 0.001*torques_P2.dot(Eigen::Vector3d  (0,0,1)); //   [N m]

    // std::cout << force(0)<<','<<force(1)<<','<<force(2)<< endl;
    // std::cout << L3(0)<<','<< L3(1)<<','<< L3(2) << endl;
    // std::cout << joint_torque_DIP(0)<<','<<joint_torque_DIP(1)<<','<<joint_torque_DIP(2)<< endl;
    // std::cout << joint_torque_DIP << endl;
    // std::cout << joint_torque_PIP << endl;
    // std::cout << joint_torque_MCP << endl;
    //std::cout << joint_torque_MCP<<','<< joint_torque_PIP << endl;
    //std::cout.flush();



    // compute exo joint torques
    double exo_torque_MCP = joint_torque_MCP;
    double exo_torque_PIP = -joint_torque_PIP;



    return Eigen::Vector2d (exo_torque_MCP, exo_torque_PIP);

}




//! Command the desired exo torque back to esmacat finger class
void hMaestroHand::commandExoTorqueFinger(double exo_torque_MCP, double exo_torque_PIP)
{
     //std::cout << exo_torque_MCP <<','<< exo_torque_PIP << endl;
    // std::cout.flush();

    // Initialize desired torque array
    double desired_torque_array[2];

    // Fill with desired torques
    desired_torque_array[0] = exo_torque_MCP;
    desired_torque_array[1] = exo_torque_PIP;


    // Sets desired torque
    hIdxFinger->setDesiredTorque(desired_torque_array);

}