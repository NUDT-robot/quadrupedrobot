/*
 * Author : ChangXu
*/
#ifndef QRMODELPLUGIN_H
#define QRMODELPLUGIN_H
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/transport.hh>
#include "include/gazebo/message/jointsensors.pb.h"
#include "include/gazebo/message/contactsensors.pb.h"
#include "include/gazebo/message/jointsposition.pb.h"
#include "include/gazebo/message/control.pb.h"
#include "config.hh"

namespace gazebo
{
    class QRModelPlugin : public ModelPlugin
    {
#define PRECISION 0.00001
    private:
        // indicate whether the plugin ready
        bool m_ready = true;
        // Pointer to the model
        physics::ModelPtr model;
        //Joints position relative members, fl: frontleft, fr: frontright, bl: backleft, br: backright
        physics::JointControllerPtr joint_controller;
        physics::Joint_V fl_jointsV, fr_jointsV, bl_jointsV, br_jointsV;
        std::vector<double> fl_jointsPosition_V, fr_jointsPosition_V, bl_jointsPosition_V, br_jointsPosition_V;
        std::vector<std::string> jointNames_V;
        common::PID pidArray[12];
        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;
       //SensorManager manage all sensors
        sensors::SensorManager* pSensorManager = 0;
       //Node for communication
        ignition::transport::Node node_JointsPosition;
        ignition::transport::Node node_AllJointSensors;
        ignition::transport::Node node_FLContactSensors;
        ignition::transport::Node node_FRContactSensors;
        ignition::transport::Node node_BLContactSensors;
        ignition::transport::Node node_BRContactSensors;
        ignition::transport::Node node_Control;
        ignition::transport::Node::Publisher pubJointsPosition;
        ignition::transport::Node::Publisher pubAllJointSensors;
        ignition::transport::Node::Publisher pubFLContactSensors;
        ignition::transport::Node::Publisher pubFRContactSensors;
        ignition::transport::Node::Publisher pubBLContactSensors;
        ignition::transport::Node::Publisher pubBRContactSensors;
        //QuadRobot messages about joint force/torque sensors
        ::QRsensor::msgs::AllJointSensors forceTorqueMsg;
        ::QRsensor::msgs::JointsPosition jointsPositionMsg;
        //vector for all QuadRobot Joint Sensors
        std::vector<sensors::ForceTorqueSensorPtr> allJointSensors_V;
        std::vector<sensors::ContactSensorPtr> contactSensors_V;
        //Update Rate, gazebo9 support max 200 Hz for the force/toque joint sensor
        uint m_update_rate = 200;
        //offset angel, gazebo model joints' angle is zero on the time when the model completed
        double offset_theta2 =  2.35619+3.1415926/2, offset_theta3 = 1.5708+3.1415926;
        //control mode
        int32_t ctrl_mode[12];
        //This array store the target PID position in order to avoid no meaning call, i.e., setting the same target position.
        double ctrl_position[12];
        //Save torque data from message for calling Joint::SetForce() every update step.
        double ctrl_torque[12];
     public:
         QRModelPlugin();
         void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
     private:
         // Save all joints' names
         void SaveJointName();
         //Get all quadrobot joint and contact sensors from sensorManager, get all joints' pointer.
         bool GetQRWrenchSensors();
         bool GetQRContactSensors();
         bool GetQRJoints();
         // Publish all force_torque sensors information
         void OnSensorUpdate();
         //Only called in OnSensorUpdate function to fill member "jointPositionMsg"
         void FillJointPositionMsg();
         //Only called in OnSensorUpdate function to fill member "forceTorqueMsg"
         void FillSensorMsg();
         //Process control message to get target joints positions, torques or others in the future.
         void ProcessControl(const ::QRcommand::msgs::Control& control);
         //Initialize PID
         void InitializePID();
         //Set PID controller parameters
          inline void SetPIDControllerParams(int index, double p, double i, double d);
    };
    GZ_REGISTER_MODEL_PLUGIN(QRModelPlugin)
}


#endif // QRMODELPLUGIN_H
