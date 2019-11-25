/*
 * Author : ChangXu
*/
#include "qrmodelplugin.h"
#include <iostream>

using namespace std;
namespace gazebo
{
    QRModelPlugin::QRModelPlugin()
    {

    }

    void QRModelPlugin ::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // Store the pointer to the model
        model = _parent;
        //Allocate memory for vectors
        fl_jointsPosition_V.resize(3);
        fr_jointsPosition_V.resize(3);
        bl_jointsPosition_V.resize(3);
        br_jointsPosition_V.resize(3);

        //Store the joint controller, save joints' names and get joints pointers.
        joint_controller = model->GetJointController();
        SaveJointName();
        InitializePID();
        GetQRJoints();
        //Initialize control mode, default position control. 1: position, 2: torque.
        memset(ctrl_mode, 2, sizeof(ctrl_mode));
        memset(ctrl_torque, 0, sizeof(ctrl_torque));
        // Listen to the update begin event. This event is broadcast every simulation iteration. Do not use ConnectWorldUpdateEnd to add force, it will behave abnormally.
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&QRModelPlugin::OnSensorUpdate, this));
        pSensorManager = sensors::SensorManager::Instance();
        //Get joint and contact sensors from SensorManager
        GetQRWrenchSensors();
        GetQRContactSensors();
        //Setup communication, all jiont sensors infomation transport to topic "/QuadRobot/12joints_sensors". Default rate 200 Hz.
        pubAllJointSensors = node_AllJointSensors.Advertise<::QRsensor::msgs::AllJointSensors>("/QuadRobot/12joints_sensors");
        pubFLContactSensors = node_FLContactSensors.Advertise<gazebo::msgs::Contacts>("/QuadRobot/contact_flsensors");
        pubFRContactSensors = node_FRContactSensors.Advertise<gazebo::msgs::Contacts>("/QuadRobot/contact_frsensors");
        pubBLContactSensors = node_BLContactSensors.Advertise<gazebo::msgs::Contacts>("/QuadRobot/contact_blsensors");
        pubBRContactSensors = node_BRContactSensors.Advertise<gazebo::msgs::Contacts>("/QuadRobot/contact_brsensors");
             //Default rate 1000 Hz for transporting joints' positions
        pubJointsPosition = node_JointsPosition.Advertise<::QRsensor::msgs::JointsPosition>("/QuadRobot/12joints_positions");
            //Receive control command and update PID target value.
        node_Control.Subscribe("/QuadRobot/control", &QRModelPlugin::ProcessControl, this);
        std::cout << " qrModelPlugin 1.0 for QuadrupedRobot3" << std::endl;
    }
   void QRModelPlugin::SaveJointName()
   {
       /*If we get joints' names from a model, no model prefix appear.
        *If we get joints' names from JointController, model prefix appears because only one JointController in the world ,
        *and it need to know which model the name belongs to.
      */
       jointNames_V.push_back(QRFL1);
       jointNames_V.push_back(QRFL2);
       jointNames_V.push_back(QRFL3);

       jointNames_V.push_back(QRFR1);
       jointNames_V.push_back(QRFR2);
       jointNames_V.push_back(QRFR3);

       jointNames_V.push_back(QRBL1);
       jointNames_V.push_back(QRBL2);
       jointNames_V.push_back(QRBL3);

       jointNames_V.push_back(QRBR1);
       jointNames_V.push_back(QRBR2);
       jointNames_V.push_back(QRBR3);
   }

    void QRModelPlugin::OnSensorUpdate()
    {

        if(!m_ready)
        {
            m_ready = true;
            //If fail at initialized stage, get joint and contact sensors from SensorManager and all joints pointer again
            GetQRJoints();
            GetQRWrenchSensors();
            GetQRContactSensors();
            if(!m_ready)
            {
                gzerr << "Get SensorPtr failed !";
                return;
            }
        }
        //Apply torque to joints
        for(ulong i = 0; i < 12; i++)
        {
            if(ctrl_mode[i] == 2)
            {
                if(i < 3)
                {
                    fl_jointsV[i]->SetForce(0, ctrl_torque[i]);
                }
                else if(i < 6)
                {
                    fr_jointsV[i-3]->SetForce(0, ctrl_torque[i]);
                }
                else if(i < 9)
                {
                    bl_jointsV[i-6]->SetForce(0, ctrl_torque[i]);
                }
                else
                {
                    br_jointsV[i-9]->SetForce(0, ctrl_torque[i]);
                }
            }
        }
    /*Fill and publish messages*/
        //Publish jointPositionMsg in world step update rate
        FillJointPositionMsg();
        pubJointsPosition.Publish(jointsPositionMsg);
        //Default 200 Hz to publish joint sensor and contact sensor messages
//        static ulong counter = 0,  stepsUpdate = 1000/m_update_rate;
//        if(counter%stepsUpdate == 0)
//        {
            FillSensorMsg();
            pubAllJointSensors.Publish(forceTorqueMsg);
            pubFLContactSensors.Publish(contactSensors_V[0]->Contacts());
            pubFRContactSensors.Publish(contactSensors_V[1]->Contacts());
            pubBLContactSensors.Publish(contactSensors_V[2]->Contacts());
            pubBRContactSensors.Publish(contactSensors_V[3]->Contacts());
//        }
//        counter++;
    }

    bool QRModelPlugin::GetQRJoints()
    {
        physics::JointPtr ptr;
        ptr = model->GetJoint(FL1);
        if(ptr)
            fl_jointsV.push_back(ptr);
        else
        {
            m_ready = false;
            gzerr << "Joint pointer is zero in GetQRJoints function";
            return false;
        }
        ptr = model->GetJoint(FL2);
        if(ptr)
            fl_jointsV.push_back(ptr);
        else
        {
            m_ready = false;
            gzerr << "Joint pointer is zero in GetQRJoints function";
            return false;
        }
        ptr = model->GetJoint(FL3);
        if(ptr)
            fl_jointsV.push_back(ptr);
        else
        {
            m_ready = false;
            gzerr << "Joint pointer is zero in GetQRJoints function";
            return false;
        }

        ptr = model->GetJoint(FR1);
        if(ptr)
            fr_jointsV.push_back(ptr);
        else
        {
            m_ready = false;
            gzerr << "Joint pointer is zero in GetQRJoints function";
            return false;
        }
        ptr = model->GetJoint(FR2);
        if(ptr)
            fr_jointsV.push_back(ptr);
        else
        {
            m_ready = false;
            gzerr << "Joint pointer is zero in GetQRJoints function";
            return false;
        }
        ptr = model->GetJoint(FR3);
        if(ptr)
            fr_jointsV.push_back(ptr);
        else
        {
            m_ready = false;
            gzerr << "Joint pointer is zero in GetQRJoints function";
            return false;
        }

        ptr = model->GetJoint(BL1);
        if(ptr)
           bl_jointsV.push_back(ptr);
        else
        {
            m_ready = false;
            gzerr << "Joint pointer is zero in GetQRJoints function";
            return false;
        }
        ptr = model->GetJoint(BL2);
        if(ptr)
           bl_jointsV.push_back(ptr);
        else
        {
            m_ready = false;
            gzerr << "Joint pointer is zero in GetQRJoints function";
            return false;
        }
        ptr = model->GetJoint(BL3);
        if(ptr)
           bl_jointsV.push_back(ptr);
        else
        {
            m_ready = false;
            gzerr << "Joint pointer is zero in GetQRJoints function";
            return false;
        }

        ptr = model->GetJoint(BR1);
        if(ptr)
           br_jointsV.push_back(ptr);
        else
        {
            m_ready = false;
            gzerr << "Joint pointer is zero in GetQRJoints function";
            return false;
        }
        ptr = model->GetJoint(BR2);
        if(ptr)
           br_jointsV.push_back(ptr);
        else
        {
            m_ready = false;
            gzerr << "Joint pointer is zero in GetQRJoints function";
            return false;
        }
        ptr = model->GetJoint(BR3);
        if(ptr)
           br_jointsV.push_back(ptr);
        else
        {
            m_ready = false;
            gzerr << "Joint pointer is zero in GetQRJoints function";
            return false;
        }

        return true;
    }

    bool QRModelPlugin::GetQRWrenchSensors()
    {
         if(std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(FLWRENCH1)))
            allJointSensors_V.push_back(std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(FLWRENCH1)));
         else
         {
             m_ready = false;
             gzerr << "fl_theta1 force/torque sensor pointers is null, See QRModelPlugin::GetQRJointSensors" << std::endl;
             return false;
         }
         if(std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(FLWRENCH2)))
            allJointSensors_V.push_back(std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(FLWRENCH2)));
         else
         {
             m_ready = false;
             gzerr << "fl_theta2 force/torque sensor pointers is null, See QRModelPlugin::GetQRJointSensors" << std::endl;
             return false;
         }
         if(std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(FLWRENCH3)))
            allJointSensors_V.push_back(std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(FLWRENCH3)));
         else
         {
             m_ready = false;
             gzerr << "fl_theta3 force/torque sensor pointers is null, See QRModelPlugin::GetQRJointSensors" << std::endl;
             return false;
         }

         if(std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(FRWRENCH1)))
            allJointSensors_V.push_back( std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(FRWRENCH1)));
         else
         {
             m_ready = false;
             gzerr << "fr_theta1 force/torque sensor pointers is null, See QRModelPlugin::GetQRJointSensors" << std::endl;
             return false;
         }
         if(std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(FRWRENCH2)))
            allJointSensors_V.push_back(std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(FRWRENCH2)));
         else
         {
             m_ready = false;
             gzerr << "fr_theta2 force/torque sensor pointers is null, See QRModelPlugin::GetQRJointSensors" << std::endl;
             return false;
         }
         if(std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(FRWRENCH3)))
            allJointSensors_V.push_back(std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(FRWRENCH3)));
         else
         {
             m_ready = false;
             gzerr << "fr_theta3 force/torque sensor pointers is null, See QRModelPlugin::GetQRJointSensors" << std::endl;
             return false;
         }

         if(std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(BLWRENCH1)))
             allJointSensors_V.push_back(std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(BLWRENCH1)));
         else
         {
             m_ready = false;
             gzerr << "bl_theta1 force/torque sensor pointers is null, See QRModelPlugin::GetQRJointSensors" << std::endl;
             return false;
         }
         if(std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(BLWRENCH2)))
            allJointSensors_V.push_back(std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(BLWRENCH2)));
         else
         {
             m_ready = false;
             gzerr << "bl_theta2 force/torque sensor pointers is null, See QRModelPlugin::GetQRJointSensors" << std::endl;
             return false;
         }
         if(std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(BLWRENCH3)))
             allJointSensors_V.push_back(std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(BLWRENCH3)));
         else
         {
             m_ready = false;
             gzerr << "bl_theta3 force/torque sensor pointers is null, See QRModelPlugin::GetQRJointSensors" << std::endl;
             return false;
         }

         if(std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(BRWRENCH1)))
            allJointSensors_V.push_back(std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(BRWRENCH1)));
         else
         {
             m_ready = false;
             gzerr << "br_theta1 force/torque sensor pointers is null, See QRModelPlugin::GetQRJointSensors" << std::endl;
             return false;
         }
         if(std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(BRWRENCH2)))
             allJointSensors_V.push_back(std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(BRWRENCH2)));
         else
         {
             m_ready = false;
             gzerr << "br_theta2 force/torque sensor pointers is null, See QRModelPlugin::GetQRJointSensors" << std::endl;
             return false;
         }
         if(std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(BRWRENCH3)))
             allJointSensors_V.push_back(std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(pSensorManager->GetSensor(BRWRENCH3)));
         else
         {
             m_ready = false;
             gzerr << "br_theta3 force/torque sensor pointers is null, See QRModelPlugin::GetQRJointSensors" << std::endl;
             return false;
         }

    }

    bool QRModelPlugin::GetQRContactSensors()
    {
        if( std::dynamic_pointer_cast<sensors::ContactSensor>(pSensorManager->GetSensor(FLCONTACT)))
           contactSensors_V.push_back(std::dynamic_pointer_cast<sensors::ContactSensor>(pSensorManager->GetSensor(FLCONTACT)));
        else
        {
            m_ready = false;
            gzerr << "fl_contact sensor pointers is null, See QRModelPlugin::GetQRContactSensors" << std::endl;
            return false;
        }
        if( std::dynamic_pointer_cast<sensors::ContactSensor>(pSensorManager->GetSensor(FRCONTACT)))
           contactSensors_V.push_back(std::dynamic_pointer_cast<sensors::ContactSensor>(pSensorManager->GetSensor(FRCONTACT)));
        else
        {
            m_ready = false;
            gzerr << "fr_contact sensor pointers is null, See QRModelPlugin::GetQRContactSensors" << std::endl;
            return false;
        }
        if( std::dynamic_pointer_cast<sensors::ContactSensor>(pSensorManager->GetSensor(BLCONTACT)))
           contactSensors_V.push_back(std::dynamic_pointer_cast<sensors::ContactSensor>(pSensorManager->GetSensor(BLCONTACT)));
        else
        {
            m_ready = false;
            gzerr << "bl_contact sensor pointers is null, See QRModelPlugin::GetQRContactSensors" << std::endl;
            return false;
        }
        if( std::dynamic_pointer_cast<sensors::ContactSensor>(pSensorManager->GetSensor(BRCONTACT)))
           contactSensors_V.push_back(std::dynamic_pointer_cast<sensors::ContactSensor>(pSensorManager->GetSensor(BRCONTACT)));
        else
        {
            m_ready = false;
            gzerr << "br_contact sensor pointers is null, See QRModelPlugin::GetQRContactSensors" << std::endl;
            return false;
        }
    }

    void QRModelPlugin::FillJointPositionMsg()
    {
         jointsPositionMsg.Clear();
        // Store all joints position
        for(ulong i = 0; i < fl_jointsV.size(); i++)
        {
            fl_jointsPosition_V[i] = fl_jointsV[i]->Position();
            fr_jointsPosition_V[i] = fr_jointsV[i]->Position();
            bl_jointsPosition_V[i] = bl_jointsV[i]->Position();
            br_jointsPosition_V[i] = br_jointsV[i]->Position();
        }
        //Fill the jointsPositionMsg with joints' positions. Gazebo all joints position is zero when model has been constructed, so joint offset should be considered

        ::QRsensor::msgs::LimbJoints* pLimbJoint = jointsPositionMsg.add_limbjoints();
        pLimbJoint->set_theta1(fl_jointsPosition_V[0]);
        pLimbJoint->set_theta2( - fl_jointsPosition_V[1] + offset_theta2);
        pLimbJoint->set_theta3(fl_jointsPosition_V[2] + offset_theta3);

        pLimbJoint = jointsPositionMsg.add_limbjoints();
        pLimbJoint->set_theta1(fr_jointsPosition_V[0]);
        pLimbJoint->set_theta2(- fr_jointsPosition_V[1] + offset_theta2);
        pLimbJoint->set_theta3(fr_jointsPosition_V[2] + offset_theta3);

        pLimbJoint = jointsPositionMsg.add_limbjoints();
        pLimbJoint->set_theta1(bl_jointsPosition_V[0]);
        pLimbJoint->set_theta2(- bl_jointsPosition_V[1] + offset_theta2);
        pLimbJoint->set_theta3(bl_jointsPosition_V[2] + offset_theta3);

        pLimbJoint = jointsPositionMsg.add_limbjoints();
        pLimbJoint->set_theta1(br_jointsPosition_V[0]);
        pLimbJoint->set_theta2(- br_jointsPosition_V[1] + offset_theta2);
        pLimbJoint->set_theta3(br_jointsPosition_V[2] + offset_theta3);

    }

    void QRModelPlugin::FillSensorMsg()
    {
        //Fill message ForceTorqueMsg to transport to QuadRobot
        ::QRsensor::msgs::Wrench* pwrench;
        forceTorqueMsg.Clear();
        for(ulong i = 0; i < allJointSensors_V.size(); i++)
        {
            pwrench = forceTorqueMsg.add_jointwrench()->mutable_wrench();
            pwrench->mutable_force()->set_x(allJointSensors_V[i]->Force().X());
            pwrench->mutable_force()->set_y(allJointSensors_V[i]->Force().Y());
            pwrench->mutable_force()->set_z(allJointSensors_V[i]->Force().Z());
            pwrench->mutable_torque()->set_x(allJointSensors_V[i]->Torque().X());
            pwrench->mutable_torque()->set_y(allJointSensors_V[i]->Torque().Y());
            pwrench->mutable_torque()->set_z(allJointSensors_V[i]->Torque().Z());
       }
    }

    void QRModelPlugin::ProcessControl(const ::QRcommand::msgs::Control& control)
    {
        //Take actions according to control message.1: position target, 2: torque
        int index = 0;
        switch(control.type())
        {
        case 1:     //position
            for(ulong i = 0; i < 12; i++)
            {
                switch(control.mode(i))
                {
                case 1:
                    //if control mode change to position, reset PID parameters
                    if(ctrl_mode[i] != 1)
                    {
                        ctrl_mode[i] = 1;
                        SetPIDControllerParams(i+1,2000,0,150);
                    }
                    switch( i%3 )
                    {
                    case 0:
                        ctrl_position[i] = control.position(index);
                        break;
                    case 1:
                        ctrl_position[i] = -control.position(index) + offset_theta2;
                        break;
                    case 2:
                        ctrl_position[i] = control.position(index) - offset_theta3;
                        break;
                    }
                    //cout << "target position " << i << " is "<< control.position(index) << endl;
                    joint_controller->SetPositionTarget(jointNames_V[i],ctrl_position[i]);

                    index++;
                    break;
                }
            }
            break;
        case 2:
            for(ulong i = 0; i < 12; i++)
            {
                switch(control.mode(i))
                {
                case 2:
                    if(ctrl_mode[i] != 2)       //if control mode change to torque, reset PID parameters to zero
                    {
                        ctrl_mode[i] = 2;
                        SetPIDControllerParams(i+1,0,0,0);
                    }
                    //cout << "target torque " << i << " is "<< control.torque(index) << endl;
                    ctrl_torque[i] = control.torque(index);
                    index++;
                    break;
                }
            }
            break;
         }
    }

    void QRModelPlugin::InitializePID()
    {
        for(int i = 1; i <= 12; i++)
            SetPIDControllerParams(i, 2000, 0, 150); //5000,150
    }

    inline void QRModelPlugin::SetPIDControllerParams(int index, double p, double i, double d)
    {
        pidArray[index-1].Reset();
        pidArray[index-1].Init(p,i,d,500,-500);
        joint_controller->SetPositionPID(jointNames_V[index-1], pidArray[index-1]);
    }

}//namespace gazebo

