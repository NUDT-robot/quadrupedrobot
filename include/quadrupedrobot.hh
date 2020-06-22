/*
 * Author : Chang Xu
 * QuadRobot class models the quadrupled robot. Physics vectors like position, velocity represented in body frame i.e. OXYZ.
 * Dimension : SI
 * Associated document :  Model Construction for Quadrupled Robot .doc
*/
#ifndef QUADRUPLEDROBOT_HH_
#define QUADRUPLEDROBOT_HH_

#include <iostream>
#include <vector>
#include "gazebo/sensor/sensor.hh"
#include "gazebo/message/jointsensors.pb.h"
#include "gazebo/message/contactsensors.pb.h"
#include "gazebo/message/jointsposition.pb.h"
#include "gazebo/message/control.pb.h"
#include "matlab/message/matlabidentification.pb.h"
#include "ds/traprocgen.hh"
#include "limbModel.hh"
#include "config.hh"
#include "mpc.hh"
#include "gait.hh"
#include "gnuplot.hh"

#include <Eigen/Dense>

/*Gazebo data form is different from real sensor, if run simulation in gazebo, we should define USE_GAZEBO_SENSOR macro*/
//#define USE_GAZEBO_SENSOR

/*
 *!!!!!!  Vector represented in body frame i.e. OXYZ !!!!!!
*/
class QuadRobot
{
public:
    //inverse kinetics solution type
    enum INV_KIN_SOLUTION_TYPE
    {
        MINI_DISTANCE = 0,
        MANUAL = 1
    };
    //Robot state
    enum ROBOT_STATE
    {
        STOP = 0,
        WALK = 1,
        TROT = 2,
        PRONK = 3,
        BOUND = 4,
        MPC = 20,
        IDENTIFICATION = 99,
    };
    //Contorl mode : Position and Torque
    enum CONTROL_MODE
    {
        POSITION = 1,
        TORQUE = 2
    };
    //QuadRobot relevant math function
    struct QRMATH
    {
        TransProcGenerator linearTPG[12];
    }qrmath;

    //four limbs
    LimbModel limb_frontleft;
    LimbModel limb_frontright;
    LimbModel limb_backleft;
    LimbModel limb_backright;
    std::vector<LimbModel*> limbVector;
private:
/*State parameters*/
    bool m_ready =false;
    bool m_SyncLock = false;
    //step time for calculating
    double m_steptime;
    double m_factorTime;
    //Expected height from ground to the position of joint 1
    double m_expheight = 0;
    //expected velocity of body
    double m_exp_vel = 0;
    //Robot state
    ROBOT_STATE robot_state = ROBOT_STATE::STOP;
    /*
     * m_TimeCounter is a internal time counter that synchronize the instance of class QuadRobot with real/simulation robot.
     * Hence, this time is not the real time in simulation and it actually indicates number of update steps. Simulation time = m_steptime*m_TimeCounter .
    */
    ulong m_TimeCounter = 0;
/*Kinetics parameters*/
    // Initial attitude and position.
    Eigen::Vector3d initialposition;
    //body attitude, velocity and relative vector or matrix
    double m_roll = 0;
    double m_pitch =0;
    double m_yaw = 0;
    // Four foot points position in ***world frame***
    Eigen::Vector3d r[4];
    //angle velocity of body
    Eigen::Vector3d m_angleVel = Eigen::Vector3d::Zero();
    //accelerated velocity of body; INS fill it
    Eigen::Vector3d m_accVel = Eigen::Vector3d::Zero();
    //velocity of body (actually it is the velocity of INS mesurement point); In navigation ,INS and body together fill it
    Eigen::Vector3d m_velocity = Eigen::Vector3d::Zero();
        // Expected velocity and attitude
    Eigen::Vector3d m_velocity_exp = Eigen::Vector3d::Zero();
    Eigen::Vector3d m_attitude_exp = Eigen::Vector3d::Zero();
    //position of body ; In navigation ,INS and body together fill it
    Eigen::Vector3d m_position = Eigen::Vector3d::Zero();
public:
    // Body reference position for MPC controller, in body frame.
    Eigen::Vector3d r_bodypos[4];
private:
    //origin position of limb
    Eigen::Vector3d limborg_frontleft;
    Eigen::Vector3d limborg_frontright;
    Eigen::Vector3d limborg_backleft;
    Eigen::Vector3d limborg_backright;
    std::vector<Eigen::Vector3d*> limborginVector;
    // Rotation Matrix Limb frame to Body frame
    Eigen::Matrix3d fl_BtoBody;
    Eigen::Matrix3d fr_BtoBody;
    Eigen::Matrix3d bl_BtoBody;
    Eigen::Matrix3d br_BtoBody;
    // Rotation Matrix Body frame to World frame, DCM means Direction Cosine Matrix
    Eigen::Matrix3d BodytoWorldDCM;

/*Dynamics parameters*/
    //center of mass of the body
    bool m_bodyCOMValid = 0;
    Eigen::Vector3d m_bodyCOM;
public:
    class Wrench
    {
    public:
        Eigen::Vector3d force;
        Eigen::Vector3d torque;
    };
    //Joint wrench of four limbs in childframe and parent to child direction. f: front, b: back, l: left, r: right.
    std::vector<Wrench> fl_Jwrench;
    std::vector<Wrench> fr_Jwrench;
    std::vector<Wrench> bl_Jwrench;
    std::vector<Wrench> br_Jwrench;
    //Wrench of contact limb. f: front, b: back, l: left, r: right. xx_Cwrenchraw means these contact force is calculated based on contact sensor (from gazebo).
    //In gazebo contact sensor return the force in frame which the sensor is in. xx_Cwrench is the force in body frame.
    //These variables store the sensors' result.
    Wrench  fl_Cwrenchraw, fr_Cwrenchraw, bl_Cwrenchraw, br_Cwrenchraw;
    Wrench  fl_Cwrench, fr_Cwrench, bl_Cwrench, br_Cwrench;
private:
    //low pass filter for four feet contact force/torque. If we want to attenuate old value to 1% of its own, set m_filter 0.631
    //we can use 0.631 in motion mode, and 0.99 in static mode;
    double m_cffilter = 0.631;
#define QRCFFB 5
    Eigen::Vector3d cffilter[8][QRCFFB];     //In first index, 4 size for force and 4 size for torque
    ulong index_QRCFFB = 0;
/*Control parameters*/
    short ctrl_modesel[12];
    //Target positions of PID, indexes order is fl, fr, bl, br. Torque applied to joints, indexes order is fl, fr, bl, br.
    double ctrl_jointsPos[12], ctrl_jointsTor[12];
/*Controller and Gait generator*/
    MPCController mpc;
    GaitGenerator gaig;
    Eigen::Vector4i supporting;
/*Other members*/
    //Inertial Navigation System sensor
    Sensor_INS m_sensorINS;
    //All joint force/torque sensors comunication node, contact points  information node and joints' positions node
    ignition::transport::Node jointsPositionNode;
    ignition::transport::Node jointSensorNode;
    ignition::transport::Node contactFLSensorNode;
    ignition::transport::Node contactFRSensorNode;
    ignition::transport::Node contactBLSensorNode;
    ignition::transport::Node contactBRSensorNode;
    //Matlab communication
    ignition::transport::Node matlabNode;
    ignition::transport::Node::Publisher pubMatlab;
    ::matlab::msgs::Identification matlabIdentMsg;
    //Command transport
    ignition::transport::Node controlNode;
    ignition::transport::Node::Publisher pubControl;
    ::QRcommand::msgs::Control controlMsg;
#ifdef GRAPH_ANALYSIS
    // Graphical analysis
    Gnuplot gp;
    int gpfileindex;
#endif

public:
    //Matlab data transfer flag
    bool transfer_contact_force = 0;
private:
    /*Private functions only serve for internal implementation*/
    void SetReady(bool ready ){m_ready = ready;}
    //Initialize expected joints' position
    void InitializeETP_TORQUE();
    //Construct model for the robot.
    void ConstructModel();
    //This function should be called when real/simulation update
    void AddTime() { m_TimeCounter++; }
    inline void SetAttitude(Eigen::Vector3d attitude){m_roll = attitude.x(); m_pitch = attitude.y(); m_yaw = attitude.z();}
    void ProcessJointsPositionData(const ::QRsensor::msgs::JointsPosition& positiondata);
    void ProcessJointSensorsData(const ::QRsensor::msgs::AllJointSensors&  jointdata);
    void ProcessFLContactSensorsData(const gazebo::msgs::Contacts&  contactdata);
    void ProcessFRContactSensorsData(const gazebo::msgs::Contacts&  contactdata);
    void ProcessBLContactSensorsData(const gazebo::msgs::Contacts&  contactdata);
    void ProcessBRContactSensorsData(const gazebo::msgs::Contacts&  contactdata);
    void AnalyzeContactsMessage(const gazebo::msgs::Contacts&  contactdata, Eigen::Vector3d& out_totalforce, Eigen::Vector3d& out_totaltorque);
    //Set control mode to "newmode" and change ctrl_modesel to correct value.Every four indexes as a group represents joints1,2,3
    inline void SetControlMode(int limbn, int thetan, CONTROL_MODE newmode)
    {
        if(limbn > 4 || limbn <= 0 || thetan > 3 || thetan <=0)
        {
            std::cout << "Wrong index in function SetControlMode ! Nothing done !" << std::endl;
            return;
        }
        ctrl_modesel[(limbn-1)*3+thetan-1] = newmode;
    }
    void SetJointsPIDTargetPos(int indexlimb, int indextheta,double targetvalue);
private:
//step function calculating all we need using stepsize m_steptime
    void Step();
/*Transmit position and torque control message*/
    void TransPosCtlMessage();
    void TransTorCtlMessage();
// This function actually control gait adpoted depend on some kinds of algorithm
    void GaitAlgorithm();
//Transport commands to quadrobot entity in real or simulation world
    void ControlTransport();
//This function simply return xx_invans members and  should be called after function SetFootPointPos called
    const std::vector<Eigen::Vector3d>& GetInvKineticsAnswer(int indexlimb);
    /*Identify inertial parameters, this function is called every update step. When the function finish its job, it will set robot state to STOP.
        We can call the function GetState to check whether identification process has been finished.*/
    void IdentifyInertialParameters();
    void IdentifyInertialParametersUsingNoAccDynamics();

// MPC Controller
    void Mpc(Eigen::VectorXd X0, const Eigen::VectorXd& Xref, const Eigen::Vector4i& advisesupport);
public:
    QuadRobot();
//property functions
    void SyncStepTime(double steptime = 0.001);
    double GetStepTime() { return m_steptime; }
    ulong GetTimeCounter() { return m_TimeCounter; }
    //Get running time of real or simulation environment
    double GetRunTime() { return m_steptime*m_TimeCounter;}
//Kinetics  functions , index from 1 to 4 represent limb_frontleft, limb_frontright, limb_backleft, limb_backright
     Eigen::Vector3d GetFootPos(u_char index);
     Eigen::Vector3d GetFootVel (u_char index);
//Dynamics functions , index from 1 to 4 represent limb_frontleft, limb_frontright, limb_backleft, limb_backright
    inline void SetLowPassFilter(double e){m_cffilter = e;}
    inline double  GetLowPassFilter(){return m_cffilter;}
public:
/*Debug funtions*/
#ifdef QUAD_DEBUG_OUTPUT
    void Debug_AllJointsPositionsOutput()
    {
        std::cout << "limb_frontleft joint theta1 : " << limb_frontleft.GetThetaPos(1) <<std::endl;
        std::cout << "limb_frontleft joint theta2 : " << limb_frontleft.GetThetaPos(2) << std::endl;
        std::cout << "limb_frontleft joint theta3 : " << limb_frontleft.GetThetaPos(3) << std::endl;
        std::cout << "limb_frontleft joint theta4 : " << limb_frontleft.GetThetaPos(4) << std::endl;
        std::cout << "limb_frontright joint theta1 : " << limb_frontright.GetThetaPos(1) << std::endl;
        std::cout << "limb_frontright joint theta2 : " << limb_frontright.GetThetaPos(2) << std::endl;
        std::cout << "limb_frontright joint theta3 : " << limb_frontright.GetThetaPos(3) << std::endl;
        std::cout << "limb_frontright joint theta4 : " << limb_frontright.GetThetaPos(4) << std::endl;
        std::cout << "limb_backleft joint theta1 : " << limb_backleft.GetThetaPos(1) << std::endl;
        std::cout << "limb_backleft joint theta2 : " << limb_backleft.GetThetaPos(2) << std::endl;
        std::cout << "limb_backleft joint theta3 : " << limb_backleft.GetThetaPos(3) << std::endl;
        std::cout << "limb_backleft joint theta4 : " << limb_backleft.GetThetaPos(4) << std::endl;
        std::cout << "limb_backright  joint theta1 : " << limb_backright.GetThetaPos(1) << std::endl;
        std::cout << "limb_backright joint theta2 : " << limb_backright.GetThetaPos(2) << std::endl;
        std::cout << "limb_backright joint theta3 : " << limb_backright.GetThetaPos(3) << std::endl;
        std::cout << "limb_backright joint theta4 : " << limb_backright.GetThetaPos(4) << std::endl;
    }
   /* void Debug_AllCtrlETPOutput(int index)
    {
        for(int i = 0; i < 200; i++)
        {
            Debug_AllCtrlETPOutput
        }
    }*/
#endif
public:
    bool IsReady() {return m_ready;}
    //GetLock and ReleaseLock functions assure multi-thread safe.
    void GetLock() { if(m_SyncLock) { std::cout << "GetLock Work!!!" << std::endl; return;} m_SyncLock = true; }
    void ReleaseLock() { m_SyncLock = false; }
    //Get Robot_STATE
    inline void SetState(ROBOT_STATE state)
    {
        robot_state = state; std::cout << "set robot state to " << state << std::endl;
    }
    ROBOT_STATE GetState() { return robot_state; }
    inline Eigen::Vector3d GetFootPointInWorldFrame(int index) { return r[index]; } // Order: r1,r2,r3,r4
    //This expected height influence what height should the bottom of robot's body approximately trot on.
     void SetExpectedHeight(double height) { m_expheight = height; }
    /*Control functions, indexlimb from 1 to 4 and indextheta from 1 to 3, angularVel means transitional process velocity, i.e. rad/s (default value is 200*0.005 = 1rad/s).
        angularVel below 2rad/s have no overhead now.*/
    void SetJointsTargetPos(int indexlimb, int indextheta,double targetvalue, double angularVel = 1);
    //Set the limb foot point to the given position, return true if success, otherwise flase if the position that the limb can't reach. Using minimize distance solution by default.
       //execute_time means how long the limb reach the postion in the second parameter given.
    bool SetFootPointPos(int indexlimb, const Eigen::Vector3d& position, double execute_time = 0.5,  INV_KIN_SOLUTION_TYPE  type = MINI_DISTANCE);
    //This function change control mode to torque control and apply a torque to the one we need.
    void SetJointsTorque(int indexlimb, int indextheta,double torque);
    //Calling this function simply change robot_state to IDENTIFICATION. After that, Function IdentifyInertialParameters will called every update step.
    inline void IdentificationMode() { SetState(ROBOT_STATE::IDENTIFICATION); }
    void MpcMode(Eigen::VectorXd X_ref);
    //void IdentificationCOMPosition(); See TODO below

    //This function return the center of mass of the body. Return true if success, otherwise false meaning that the function 'IdentificationCOMPosition' has not been called.
    inline bool GetBodyCOM(Eigen::Vector3d& com)
    {
        if(m_bodyCOMValid) { com = m_bodyCOM; return true;}
        else{ return false; }
    }
    void IdentificationCOMPosition();
    Eigen::Vector3d GetVelocity(const Eigen::Vector4i& support);
    void SetInitialPos(Eigen::Vector3d pos);
    void WalkGait(const Eigen::Vector3d& attitude_direction, const Eigen::Vector3d& velocity);
    void TrotGait(const Eigen::Vector3d& attitude_direction, const Eigen::Vector3d& velocity);
    void PronkGait(const Eigen::Vector3d& attitude_direction, const Eigen::Vector3d& velocity);
    void BoundGait(const Eigen::Vector3d& attitude_direction, const Eigen::Vector3d& velocity);
    //Now, this function should be only called when the body is in level.

friend class Sensor_INS;
friend class GaitGenerator;
};

#endif

