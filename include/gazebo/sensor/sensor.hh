/*
 * Author : ChangXu
 * Sensor_INS class represents Inertial Navigation System(INS)
 * Dimension : SI
 * Associated document :  Model Construction for Quadrupled Robot .doc
*/

#ifndef SENSOR_H
#define SENSOR_H

#include <iostream>
#include <gazebo/gazebo.hh>
#include <ignition/transport.hh>
#include <Eigen/Dense>

class QuadRobot;
class Sensor_INS
{
    //Euler angle and angular velocity.
    double preroll = 0 ;
    double prepitch = 0;
    double preyaw = 0;
    double yawVel = 0;
    double pitchVel = 0;
    double rollVel = 0;
    //indicate data is valid or not
    bool m_ready = 0;
    ignition::transport::Node node;
    QuadRobot* parent = 0;
    double m_steptime = 0.005;
    double m_factorTime = 200;
private:
    //callback function for node subscribe;
    void ProcessGazeboINSData(const gazebo::msgs::IMU& imu_msg, const ignition::transport:: MessageInfo &_info);
    /*In gazebo, Euler angle represented three angles around X axis, Y axis and Z axis of parent frame in turn,
     *which is not coincidence with real Ekinox INS. So we should tackle this differece in simulation,
     * and that what GetAttitudeFromGazebo function does
    */
    inline bool GetINSFromGazebo(double& roll, double& pitch, double& yaw, Eigen::Vector3d& linearAcc);

public:
    /*Initialize INS : Connect to the INS and get valid data then set "m_ready" true*/
    Sensor_INS(QuadRobot* _parent = 0) ;
    inline  void SetSteptime(double steptime) {assert(steptime > 0); m_steptime = steptime;m_factorTime= 1.0/m_steptime;}
    //this should be modified in the future. Better build a base of all robots' class.
    void SetParent(QuadRobot* _parent){parent = _parent;}
/*In simulation with gazebo, we should call ConnectGazeboIMU instead of ConnectINS, this is not a good way and will be modified in the future*/
    bool ConnectINS();
    bool ConnectGazeboIMU();
    //Get the attitude angle
    bool GetAttitude(double& yaw, double& pitch, double& roll);

};



#endif // SENSOR_H
