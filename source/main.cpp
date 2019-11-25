#define QUAD_DEBUG_OUTPUT

#include "include/quadrupedrobot.hh"
#include <iostream>
#include <ignition/math.hh>
#include <ignition/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include "include/ds/tree.h"
//#include <engine.h>


using namespace std;
using namespace ignition::math;
using namespace ignition::transport;
int main()
{
    QuadRobot robot;
    robot.SyncStepTime(0.001);
    Vector3d initial_position(0,0,0.50);
    for (int i = 1; i <= 4; i++)
    {

        if(robot.SetFootPointPos(i,initial_position,4))
        {
            //this expected height is used to trot.
            robot.SetExpectedHeight(0.50);
        }
        else {
            cout << "Set height failed !" << endl;
        }
    }

    int  command = 0;
    double pos1, pos2, pos3;
    bool exit = false;
    double vel = 0;
    int limbn = 0, jointn = 0;
    Vector3d targetpos(0, 0, 0.5);
    double torque = 0;
    Vector3d position,verify_position;
    std::vector<Vector3d> theta_v;
    ulong counter = 0, totalcounter = 0, solvecounter = 0;
    double time = 0;
    double v1 = 0, v2 = 0, v3 = 0;
    double trotVel = 0;
    Vector3d bodyCOM;
    while(!exit)
    {
        cout << "Enter a command number : "<< endl;
        cin >> command;
        switch(command)
        {      
        case 1:
            cout << "Enter the same four joint positions for every limb in rad:  pos1  pos2  pos3  time "<< endl;
            cin >> pos1 >> pos2 >> pos3 >> time;
            v1 = abs(robot.limb_backleft.GetThetaPos(1) - pos1)/time;
            v2 = abs(robot.limb_backleft.GetThetaPos(2) - pos2)/time;
            v3 = abs(robot.limb_backleft.GetThetaPos(3) - pos3)/time;
            for(int i = 1; i <= 4; i++)
            {
                 for(int j = 1; j <= 3; j++)
                 {
                     switch(j%3)
                     {
                     case 1:
                         robot.SetJointsTargetPos(i, j, pos1,v1);
                         break;
                     case 2:
                         robot.SetJointsTargetPos(i, j, pos2,v2);
                         break;
                     case 0:
                         robot.SetJointsTargetPos(i, j, pos3,v3);
                         break;
                     }
                  }
            }

            break;
        case 2:
            cout << "Enter a target position :" << endl;
            cin >> pos1;
            robot.SetJointsTargetPos(1,1, pos1, 1);
            break;
        case 3:
            cout << "Set foot point. Input: limb, x, y, z "<< endl;
            cin >> limbn >>  targetpos ;
            if(!robot.SetFootPointPos(limbn, targetpos , 2, QuadRobot::MANUAL))
               cout << "No solution exists !" << endl;
            break;
        case 4:
            cout << "Set joints' torque. Input: limbn, jointn, torque "<< endl;
            cin >> limbn >> jointn >>torque ;
            robot.SetJointsTorque(limbn, jointn, torque);
            break;
        case 10:
            cout << "Trot velocity : " << endl;
            cin >> trotVel;
            robot.TrotGait(trotVel);
            break;
        case 11:
            cout << "Jump gait :" << endl;
            robot.JumpGait();
            break;
        case 98:
            //Indentification coenter of mass
            robot.IdentificationCOMPosition();
            robot.GetBodyCOM(bodyCOM);
            cout << "The center of mass position is :"<< bodyCOM << endl;
            break;
        case 99:
                robot.SetJointsTorque(1,1,0);
                robot.SetJointsTorque(1,2,0);
                robot.SetJointsTorque(1,3,0);
            cout << "Preparing for identification, just waiting for 5s ......" << endl;
            sleep(5);
            robot.IdentificationMode();
            break;
        case 100:
            robot.Debug_AllJointsPositionsOutput();
            cout << "limb_frontleft foot position : " << robot.limb_frontleft.GetPointPos(3) << endl;
            cout << "limb_frontright foot position : " << robot.limb_frontright.GetPointPos(3) << endl;
            cout << "limb_backleft foot position : " << robot.limb_backleft.GetPointPos(3) << endl;
            cout << "limb_backright foot position : " << robot.limb_backright.GetPointPos(3) << endl;
            break;
        case 101:   //Verify Invert Kinetics Calculation, we must do it without the simulation running to avoid angle conflict.
            for(int  i = -1000; i < 1000; i+=random()%10 )
                for(int j =-1000; j < 1000; j+=random()%10 )
                    for(int k = 0; k < 1000; k+=random()%10)
                        {
                            position.X() = i/1000.0;
                            position.Y() = j/1000.0;
                            position.Z() = k/1000.0;
                            if(!robot.limb_backleft.CalInvertKinetics(position))
                            {
                                totalcounter++;
                                continue;
                            }
                            solvecounter++;
                            theta_v = robot.limb_backleft.GetInvAnswer();
                            for(ulong i = 0; i < theta_v.size(); i++)
                            {
                                if(theta_v.size() == 0)
                                    break;
                                robot.limb_backleft.SetThetaPos(1,theta_v[i].X());
                                robot.limb_backleft.SetThetaPos(2,theta_v[i].Y());
                                robot.limb_backleft.SetThetaPos(3,theta_v[i].Z());
                                robot.limb_backleft.CalMainPointPos();
                                verify_position = robot.limb_backleft.GetPointPos(3);
                                if(position.Equal(verify_position,0.0001))
                                {
                                }
                                else
                                {
                                    counter++;
                                }
                            }
                            totalcounter++;
                        }
               if(counter == 0)
               {
                     cout << "Verify success ! " << "Total test: " << totalcounter << endl;
                     totalcounter = 0;
               }
               else {
                     cout << "Total test: " << totalcounter << ". Error quantities: " << counter << endl;
                     counter = 0;
                     totalcounter = 0;
               }
               cout << "Total solve number: " << solvecounter << endl;
            break;
        case 102:
            //joint torque output
            for(int i = 0; i< 3; i++)
            {
               cout << "FL torque joint " << i+1 <<" : " << robot.fl_Jwrench[i].torque << endl;
               cout << "FR torque joint " << i+1 <<" : " << robot.fr_Jwrench[i].torque << endl;
               cout << "BL torque joint " << i+1 <<" : " << robot.bl_Jwrench[i].torque << endl;
               cout << "BR torque joint " << i+1 <<" : " << robot.br_Jwrench[i].torque << endl;
            }
            break;
        case 103:
            //contact force output
            cout << "fl contact force :" << robot.limb_frontleft.GetContactForce() << endl;
            cout << "fr contact force :" << robot.limb_frontright.GetContactForce() << endl;
            cout << "bl contact force :" << robot.limb_backleft.GetContactForce() << endl;
            cout << "br contact force :" << robot.limb_backright.GetContactForce() << endl;
            robot.transfer_contact_force = 1;
            break;
            //Joint angles output
        case 104:
            cout << "BL Theta1 : "<< robot.limb_backleft.GetThetaPos(1) <<  "\nBL Theta2 : "<< robot.limb_backleft.GetThetaPos(2) \
                      << "\nBL Theta3 : " << robot.limb_backleft.GetThetaPos(3) << endl;
            break;
        case -1:
             exit = true;
             break;
        default:
            cout << "command :" << command << " is not supported !" << endl;
            break;
        }
    }
    return 0;
}

/*   Quaternion<double> quater;
   Vector3d result;
   double q1,  q2, q3, q4;
   while(1)
  {
       cin >> q1;
       cin >> q2;
       cin >> q3;
       cin >> q4;
       quater.Set(q1,q2,q3,q4);
       result =quater.Euler();
       cout << "Q to Euler result :"<<result <<endl;
   }
*/
