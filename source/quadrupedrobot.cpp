#include "include/quadrupedrobot.hh"

/*
 * QRDEBUG_JOINTPOS : four limbs  joint position debug
*/
//This macro should be modified according with limbModel.hh
#define FILTERSIZE 20

QuadRobot::QuadRobot()
 {
     ConstructModel();
 }

void QuadRobot::SyncStepTime(double steptime)
{
     m_steptime = steptime;
     m_factorTime = 1.0/m_steptime;
     limb_frontleft.SetSteptime(steptime);
     limb_frontright.SetSteptime(steptime);
     limb_backleft.SetSteptime(steptime);
     limb_backright.SetSteptime(steptime);
     //INS_STEPTIME should be configured in config.h. Ekinox max update rate is 200Hz.
     m_sensorINS.SetSteptime(INS_STEPTIME);
 }

void QuadRobot::InitializeETP_TORQUE()
{
    //Store initial height
    limb_frontleft.CalMainPointPos();
    m_expheight = limb_frontleft.GetPointPos(3).Z();
}
void QuadRobot::ConstructModel()
{

    //Synchronize all steptime and calculate multiplication factor time, default step time is 0.001, i.e. 1000Hz
    SyncStepTime(0.001);
    //Configure limb position in body
    limborg_frontleft = Vector3d(0.441, -0.195, -0.18725);
    limborg_frontright = Vector3d(0.441, 0.195, -0.18725);
    limborg_backleft  = Vector3d(-0.441, -0.195, -0.18725);
    limborg_backright = Vector3d(-0.441, 0.195, -0.18725);
    //Configure limbModel parameter, cop[3], mass[3] is actually not used now.
    double L[3] ={LINK1_LEN, LINK2_LEN, LINK3_LEN }, cop[3] = {0.02975, 0.135, 0.1675}, mass[3] = {1.053, 5.245, 3.178};
    Matrix3d inertial[3] = {Matrix3d::Identity, Matrix3d::Identity, Matrix3d::Identity};  //Inertial matrix is actually not used now.
    //Same parameters for four limbs
    for (u_char i = 0; i < 3; i++)
    {
        limb_frontleft.ModifyLinkParam(i+1, L[i], cop[i], mass[i], inertial[i]);
        limb_frontright.ModifyLinkParam(i+1, L[i], cop[i], mass[i], inertial[i]);
        limb_backleft.ModifyLinkParam(i+1, L[i], cop[i], mass[i], inertial[i]);
        limb_backright.ModifyLinkParam(i+1, L[i], cop[i], mass[i], inertial[i]);
    }
    //Configure dynamics parameters
    fl_Jwrench.resize(3);
    fr_Jwrench.resize(3);
    bl_Jwrench.resize(3);
    br_Jwrench.resize(3);
    //Initialize control parameters
    for(int i = 0; i < 12; i++)
    {
        ctrl_modesel[i] = CONTROL_MODE::TORQUE;
    }
    memset(ctrl_jointsPos, 0, sizeof(ctrl_jointsPos));
    memset(ctrl_jointsTor, 0,  sizeof(ctrl_jointsTor));
    //Configure Sensors
    m_sensorINS.SetParent(this);
    //Setup communcation
    jointsPositionNode.Subscribe("/QuadRobot/12joints_positions", &QuadRobot::ProcessJointsPositionData, this);
    jointSensorNode.Subscribe("/QuadRobot/12joints_sensors", &QuadRobot::ProcessJointSensorsData, this);
    contactFLSensorNode.Subscribe("/QuadRobot/contact_flsensors", &QuadRobot::ProcessFLContactSensorsData, this);
    contactFRSensorNode.Subscribe("/QuadRobot/contact_frsensors", &QuadRobot::ProcessFRContactSensorsData, this);
    contactBLSensorNode.Subscribe("/QuadRobot/contact_blsensors", &QuadRobot::ProcessBLContactSensorsData, this);
    contactBRSensorNode.Subscribe("/QuadRobot/contact_brsensors", &QuadRobot::ProcessBRContactSensorsData, this);
    pubControl = controlNode.Advertise<::QRcommand::msgs::Control>("/QuadRobot/control");
    pubMatlab = matlabNode.Advertise<matlab::msgs::Identification>("/QuadRobot/matlab");
    //Initialize expected joints' positions
    while(!IsReady());
    InitializeETP_TORQUE();
}
void QuadRobot::ProcessJointsPositionData(const ::QRsensor::msgs::JointsPosition& positiondata)
{
    /*Calculate joints' positions and velocities every five steps, in gazebo dufault step time is 0.001s
     *f is low pass filter, 0.3^5 = 0.0023
     * This function calculates all joint position, joint angle velocity and joint angle acceleration
    */

    //In torque control mode, every world step, i.e. 0.001s, torque should be applied.
    TransTorCtlMessage();

    //Low pass filter coefficient
    double f_s = 0, f_v = 0, f_a = 0;
    static double fl_pretheta[3] = {0,135,90}, fr_pretheta[3] = {0,135,90}, bl_pretheta[3] = {0,135,90}, br_pretheta[3] = {0,135,90};
    static double fl_prethetaVel[3]={0,0,0}, fr_prethetaVel[3]={0,0,0}, bl_prethetaVel[3]={0,0,0}, br_prethetaVel[3]={0,0,0};
    /*All joint position is rounded to 1/resolution, so velocity resolution is 1/resolution*m_factorTime rad/s*/
    //fl first
    limb_frontleft.m_thetaPos[0] = f_s*limb_frontleft.m_thetaPos[0] + (1-f_s)*positiondata.limbjoints(0).theta1();
    limb_frontleft.m_thetaPos[1] = f_s*limb_frontleft.m_thetaPos[1] + (1-f_s)*positiondata.limbjoints(0).theta2();
    limb_frontleft.m_thetaPos[2] = f_s*limb_frontleft.m_thetaPos[2] + (1-f_s)*positiondata.limbjoints(0).theta3();
    //then fr
    limb_frontright.m_thetaPos[0] = f_s*limb_frontright.m_thetaPos[0] + (1-f_s)*positiondata.limbjoints(1).theta1();
    limb_frontright.m_thetaPos[1] = f_s*limb_frontright.m_thetaPos[1] + (1-f_s)*positiondata.limbjoints(1).theta2();
    limb_frontright.m_thetaPos[2] = f_s*limb_frontright.m_thetaPos[2] + (1-f_s)*positiondata.limbjoints(1).theta3();
    //then bl
    limb_backleft.m_thetaPos[0] = f_s*limb_backleft.m_thetaPos[0] + (1-f_s)*positiondata.limbjoints(2).theta1();
    limb_backleft.m_thetaPos[1] = f_s*limb_backleft.m_thetaPos[1] + (1-f_s)*positiondata.limbjoints(2).theta2();
    limb_backleft.m_thetaPos[2] = f_s*limb_backleft.m_thetaPos[2] + (1-f_s)*positiondata.limbjoints(2).theta3();
    //last br
    limb_backright.m_thetaPos[0] = f_s*limb_backright.m_thetaPos[0] + (1-f_s)*positiondata.limbjoints(3).theta1();
    limb_backright.m_thetaPos[1] = f_s*limb_backright.m_thetaPos[1] + (1-f_s)*positiondata.limbjoints(3).theta2();
    limb_backright.m_thetaPos[2] = f_s*limb_backright.m_thetaPos[2] + (1-f_s)*positiondata.limbjoints(3).theta3();
    static ulong index = 0;
    for(int i = 0; i < 3; i++)
    {
        //Calculate joint angle velocity
        limb_frontleft.m_thetaVelFilterBuffer[i][index%FILTERSIZE] = (limb_frontleft.m_thetaPos[i] - fl_pretheta[i]) * POSITION_STEPFACTOR;
        limb_frontright.m_thetaVelFilterBuffer[i][index%FILTERSIZE] =   (limb_frontright.m_thetaPos[i] - fr_pretheta[i]) * POSITION_STEPFACTOR;
        limb_backleft.m_thetaVelFilterBuffer[i][index%FILTERSIZE] = (limb_backleft.m_thetaPos[i] - bl_pretheta[i]) * POSITION_STEPFACTOR;
        limb_backright.m_thetaVelFilterBuffer[i][index%FILTERSIZE] =  (limb_backright.m_thetaPos[i] - br_pretheta[i]) * POSITION_STEPFACTOR;
        //save current joints' positions as previous.
        fl_pretheta[i] = limb_frontleft.m_thetaPos[i];
        fr_pretheta[i] = limb_frontright.m_thetaPos[i];
        bl_pretheta[i] = limb_backleft.m_thetaPos[i];
        br_pretheta[i] = limb_backright.m_thetaPos[i];
    }
    double velFilter[12],accVelFilter[12];
    memset(velFilter, 0, sizeof(velFilter));
    memset(accVelFilter, 0, sizeof(accVelFilter));
    //Angle velocity average filter
    for(int j = 0; j < FILTERSIZE; j++)
    {
        velFilter[0] +=  limb_frontleft.m_thetaVelFilterBuffer[0][j]/FILTERSIZE;
        velFilter[1] +=  limb_frontleft.m_thetaVelFilterBuffer[1][j]/FILTERSIZE;
        velFilter[2] +=  limb_frontleft.m_thetaVelFilterBuffer[2][j]/FILTERSIZE;

        velFilter[3] +=  limb_frontright.m_thetaVelFilterBuffer[0][j]/FILTERSIZE;
        velFilter[4] +=  limb_frontright.m_thetaVelFilterBuffer[1][j]/FILTERSIZE;
        velFilter[5] +=  limb_frontright.m_thetaVelFilterBuffer[2][j]/FILTERSIZE;

        velFilter[6] +=  limb_backleft.m_thetaVelFilterBuffer[0][j]/FILTERSIZE;
        velFilter[7] +=  limb_backleft.m_thetaVelFilterBuffer[1][j]/FILTERSIZE;
        velFilter[8] +=  limb_backleft.m_thetaVelFilterBuffer[2][j]/FILTERSIZE;

        velFilter[9] +=  limb_backright.m_thetaVelFilterBuffer[0][j]/FILTERSIZE;
        velFilter[10] +=  limb_backright.m_thetaVelFilterBuffer[1][j]/FILTERSIZE;
        velFilter[11] +=  limb_backright.m_thetaVelFilterBuffer[2][j]/FILTERSIZE;
    }
    //angel velocity low pass filter
    for(int i = 0; i < 3; i++)
    {
        limb_frontleft.m_thetaVel[i] = f_v*limb_frontleft.m_thetaVel[i]+(1-f_v)*velFilter[i];
        limb_frontright.m_thetaVel[i] = f_v*limb_frontright.m_thetaVel[i]+(1-f_v)*velFilter[i+3];
        limb_backleft.m_thetaVel[i] = f_v*limb_backleft.m_thetaVel[i]+(1-f_v)*velFilter[i+6];
        limb_backright.m_thetaVel[i] = f_v*limb_backright.m_thetaVel[i]+(1-f_v)*velFilter[i+9];
    }
    for(int i = 0; i < 3; i++)
    {
        //Calculate joint accelerated velocity using average value filter
        limb_frontleft.m_thetaAccVelFilterBuffer[i][index%FILTERSIZE] =  (limb_frontleft.m_thetaVel[i] - fl_prethetaVel[i]) * POSITION_STEPFACTOR;
        limb_frontright.m_thetaAccVelFilterBuffer[i][index%FILTERSIZE] = (limb_frontright.m_thetaVel[i] - fr_prethetaVel[i]) * POSITION_STEPFACTOR;
        limb_backleft.m_thetaAccVelFilterBuffer[i][index%FILTERSIZE] = (limb_backleft.m_thetaVel[i] - bl_prethetaVel[i]) * POSITION_STEPFACTOR;
        limb_backright.m_thetaAccVelFilterBuffer[i][index%FILTERSIZE] = (limb_backright.m_thetaVel[i] - br_prethetaVel[i]) * POSITION_STEPFACTOR;
        //After accelerated velocity calculating finish. Save current joints velocities  as previous.
        fl_prethetaVel[i] = limb_frontleft.m_thetaVel[i];
        fr_prethetaVel[i] = limb_frontright.m_thetaVel[i];
        bl_prethetaVel[i] = limb_backleft.m_thetaVel[i];
        br_prethetaVel[i] = limb_backright.m_thetaVel[i];
    }
    //Angled Accelerated Velocity average filter
    for(int j = 0; j < FILTERSIZE; j++)
    {
        accVelFilter[0] +=  limb_frontleft.m_thetaAccVelFilterBuffer[0][j]/FILTERSIZE;
        accVelFilter[1] +=  limb_frontleft.m_thetaAccVelFilterBuffer[1][j]/FILTERSIZE;
        accVelFilter[2] +=  limb_frontleft.m_thetaAccVelFilterBuffer[2][j]/FILTERSIZE;

        accVelFilter[3] +=  limb_frontright.m_thetaAccVelFilterBuffer[0][j]/FILTERSIZE;
        accVelFilter[4] +=  limb_frontright.m_thetaAccVelFilterBuffer[1][j]/FILTERSIZE;
        accVelFilter[5] +=  limb_frontright.m_thetaAccVelFilterBuffer[2][j]/FILTERSIZE;

        accVelFilter[6] +=  limb_backleft.m_thetaAccVelFilterBuffer[0][j]/FILTERSIZE;
        accVelFilter[7] +=  limb_backleft.m_thetaAccVelFilterBuffer[1][j]/FILTERSIZE;
        accVelFilter[8] +=  limb_backleft.m_thetaAccVelFilterBuffer[2][j]/FILTERSIZE;

        accVelFilter[9] +=  limb_backright.m_thetaAccVelFilterBuffer[0][j]/FILTERSIZE;
        accVelFilter[10] +=  limb_backright.m_thetaAccVelFilterBuffer[1][j]/FILTERSIZE;
        accVelFilter[11] +=  limb_backright.m_thetaAccVelFilterBuffer[2][j]/FILTERSIZE;
    }
    //angel accelerated velocity low pass filter
    for(int i = 0; i < 3; i++)
    {
        limb_frontleft.m_thetaAccVel[i] = f_a*limb_frontleft.m_thetaAccVel[i]+(1-f_a)*accVelFilter[i];
        limb_frontright.m_thetaAccVel[i] = f_a*limb_frontright.m_thetaAccVel[i]+(1-f_a)*accVelFilter[i+3];
        limb_backleft.m_thetaAccVel[i] = f_a*limb_backleft.m_thetaAccVel[i]+(1-f_a)*accVelFilter[i+6];
        limb_backright.m_thetaAccVel[i] = f_a*limb_backright.m_thetaAccVel[i]+(1-f_a)*accVelFilter[i+9];
    }
    index++;
    //Receive joint data means limbs' data is valid and quadrobot is ready.
    limb_frontleft.SetValid(true);
    limb_frontright.SetValid(true);
    limb_backleft.SetValid(true);
    limb_backright.SetValid(true);
    SetReady(true);
}
void QuadRobot::ProcessJointSensorsData(const ::QRsensor::msgs::AllJointSensors&  jointdata)
{
    for(int i = 0; i < jointdata.jointwrench().size(); i++)
   {
       switch(i)
       {
           case 0:
           case 1:
           case 2:
               fl_Jwrench[i].force.X()  = jointdata.jointwrench(i).wrench().force().x();
               fl_Jwrench[i].force.Y()  = jointdata.jointwrench(i).wrench().force().y();
               fl_Jwrench[i].force.Z()  = jointdata.jointwrench(i).wrench().force().z();
               fl_Jwrench[i].torque.X()  = jointdata.jointwrench(i).wrench().torque().x();
               fl_Jwrench[i].torque.Y()  = jointdata.jointwrench(i).wrench().torque().y();
               fl_Jwrench[i].torque.Z()  = jointdata.jointwrench(i).wrench().torque().z();
               break;
           case 3:
           case 4:
           case 5:
               fr_Jwrench[i-3].force.X()  = jointdata.jointwrench(i).wrench().force().x();
               fr_Jwrench[i-3].force.Y()  = jointdata.jointwrench(i).wrench().force().y();
               fr_Jwrench[i-3].force.Z()  = jointdata.jointwrench(i).wrench().force().z();
               fr_Jwrench[i-3].torque.X()  = jointdata.jointwrench(i).wrench().torque().x();
               fr_Jwrench[i-3].torque.Y()  = jointdata.jointwrench(i).wrench().torque().y();
               fr_Jwrench[i-3].torque.Z()  = jointdata.jointwrench(i).wrench().torque().z();
               break;
           case 6:
           case 7:
           case 8:
               bl_Jwrench[i-6].force.X()  = jointdata.jointwrench(i).wrench().force().x();
               bl_Jwrench[i-6].force.Y()  = jointdata.jointwrench(i).wrench().force().y();
               bl_Jwrench[i-6].force.Z()  = jointdata.jointwrench(i).wrench().force().z();
               bl_Jwrench[i-6].torque.X()  = jointdata.jointwrench(i).wrench().torque().x();
               bl_Jwrench[i-6].torque.Y()  = jointdata.jointwrench(i).wrench().torque().y();
               bl_Jwrench[i-6].torque.Z()  = jointdata.jointwrench(i).wrench().torque().z();
               break;
           case 9:
           case 10:
           case 11:
               br_Jwrench[i-9].force.X()  = jointdata.jointwrench(i).wrench().force().x();
               br_Jwrench[i-9].force.Y()  = jointdata.jointwrench(i).wrench().force().y();
               br_Jwrench[i-9].force.Z()  = jointdata.jointwrench(i).wrench().force().z();
               br_Jwrench[i-9].torque.X()  = jointdata.jointwrench(i).wrench().torque().x();
               br_Jwrench[i-9].torque.Y()  = jointdata.jointwrench(i).wrench().torque().y();
               br_Jwrench[i-9].torque.Z()  = jointdata.jointwrench(i).wrench().torque().z();
               break;
       default:
               std::cout << "ProcessJointSensorsData Maybe Wrong !!!" << std::endl;
        }
    }
}
void QuadRobot::AnalyzeContactsMessage(const gazebo::msgs::Contacts&  contactdata, Vector3d& out_totalforce, Vector3d& out_totaltorque)
{
    Vector3d  oneforce, onetorque;
    /*
     * I think gzebo::msgs::Contacts include all contacts in  world step time rate no matter what contact senseor update rate we set.
    * That means if we set contact sensor update_rate = 200, when world step time is 0.001,
    * gzebo::msgs::Contacts may include at most 5 contact in 5 different time.
    * Every step time the contact points are different. Don't accumulate different time contact points force or torque !!!
    */
    //Calculate contact force/torque using low pass filter
    if(contactdata.contact_size() == 0)
    {
        //cout << "No contact points received !" << endl;
        return;
    }
    for(int j= 0; j < contactdata.contact(0).wrench_size(); j++)
    {
        // I think body_1 is the body owning contact sensor, because when quadrobot touchs ground , it is body_1.
       oneforce.X() += contactdata.contact(0).wrench(j).body_1_wrench().force().x();
       oneforce.Y() += contactdata.contact(0).wrench(j).body_1_wrench().force().y();
       oneforce.Z() += contactdata.contact(0).wrench(j).body_1_wrench().force().z();
       onetorque.X() += contactdata.contact(0).wrench(j).body_1_wrench().torque().x();
       onetorque.Y() += contactdata.contact(0).wrench(j).body_1_wrench().torque().y();
       onetorque.Z() += contactdata.contact(0).wrench(j).body_1_wrench().torque().z();
    }
    out_totalforce = oneforce;
    out_totaltorque = onetorque;
}
void QuadRobot::ProcessFLContactSensorsData(const gazebo::msgs::Contacts&  contactdata)
{
    AnalyzeContactsMessage(contactdata, fl_Cwrenchraw.force, fl_Cwrenchraw.torque);
}
void QuadRobot::ProcessFRContactSensorsData(const gazebo::msgs::Contacts&  contactdata)
{
   AnalyzeContactsMessage(contactdata, fr_Cwrenchraw.force, fr_Cwrenchraw.torque);
}
void QuadRobot::ProcessBLContactSensorsData(const gazebo::msgs::Contacts&  contactdata)
{
    AnalyzeContactsMessage(contactdata, bl_Cwrenchraw.force, bl_Cwrenchraw.torque);
}
void QuadRobot::ProcessBRContactSensorsData(const gazebo::msgs::Contacts&  contactdata)
{
    AnalyzeContactsMessage(contactdata, br_Cwrenchraw.force, br_Cwrenchraw.torque);
    /*
     * Make sure that after this function, all valuble data have been received, so we call function "Step()" to update quadrobot state.
    */
    Step();
}
Vector3d QuadRobot::GetFootPos(u_char index)
{

}

/*
 * Calculate body angle velocity,velocities and accelerated velocities of origins of limbs,
*Update four limbs' kinetics and dynamics parameters
*/
void QuadRobot::Step()
{
    //When step function is called, add simulation time by AddTime function
    AddTime();
    //Calculate velocities and accelerated velocities of origins of limbs
    Vector3d O1Vel = m_angleVel.Cross(limborg_frontleft) + m_velocity;
    Vector3d O2Vel = m_angleVel.Cross(limborg_frontright) + m_velocity;
    Vector3d O3Vel = m_angleVel.Cross(limborg_backleft) + m_velocity;
    Vector3d O4Vel = m_angleVel.Cross(limborg_backright) + m_velocity;
    /* TODO, AccVel is inaccurate now */
    Vector3d O1AccVel = m_accVel;// + m_angleVel.Cross(m_angleVel.Cross(limborg_frontleft)) ;
    Vector3d O2AccVel = m_accVel;// + m_angleVel.Cross(m_angleVel.Cross(limborg_frontright)) ;
    Vector3d O3AccVel = m_accVel;// + m_angleVel.Cross(m_angleVel.Cross(limborg_backleft)) ;
    Vector3d O4AccVel = m_accVel;// + m_angleVel.Cross(m_angleVel.Cross(limborg_backright)) ;
    //Update four limbs' kinetics and dynamics parameters before calling LimbModel class's step funtion
            //We need set velocity and accelerated velocity of origins of limbs
    limb_frontleft.SetOriginVel(O1Vel);
    limb_frontleft.SetOriginAccVel(O1AccVel);
    limb_frontright.SetOriginVel(O2Vel);
    limb_frontright.SetOriginAccVel(O2AccVel);
    limb_backleft.SetOriginVel(O3Vel);
    limb_backleft.SetOriginAccVel(O3AccVel);
    limb_backright.SetOriginVel(O4Vel);
    limb_backright.SetOriginAccVel(O4AccVel);
            //Set four limbs generalized force
    Vector3d gf;
    gf.X() = fl_Jwrench[0].torque.X();
    gf.Y() = -fl_Jwrench[1].torque.Y();
    gf.Z() = fl_Jwrench[2].torque.Y();
    limb_frontleft.SetGeneralizedForce(gf);

    gf.X() = fr_Jwrench[0].torque.X();
    gf.Y() = -fr_Jwrench[1].torque.Y();
    gf.Z() = fr_Jwrench[2].torque.Y();
    limb_frontright.SetGeneralizedForce(gf);

    gf.X() = bl_Jwrench[0].torque.X();
    gf.Y() = -bl_Jwrench[1].torque.Y();
    gf.Z() = bl_Jwrench[2].torque.Y();
    limb_backleft.SetGeneralizedForce(gf);

    gf.X() = br_Jwrench[0].torque.X();
    gf.Y() = -br_Jwrench[1].torque.Y();
    gf.Z() = br_Jwrench[2].torque.Y();
    limb_backright.SetGeneralizedForce(gf);
    //When parameters update above have been finished, calling four limbs 'Step' function in turn
    limb_frontleft.Step();
    limb_frontright.Step();
    limb_backleft.Step();
    limb_backright.Step();
    //In gazebo, contact sensor return the force in frame which the sensor is in. Here transform the contact force from frame B to Body
    Matrix3d fl_BtoBody = limb_frontleft.GetCoordTransformMatrix();
    Matrix3d fr_BtoBody = limb_frontright.GetCoordTransformMatrix();
    Matrix3d bl_BtoBody = limb_backleft.GetCoordTransformMatrix();
    Matrix3d br_BtoBody = limb_backright.GetCoordTransformMatrix();
         //Average filter and low pass filter for contact force/torque
    Vector3d cfbuf[8];
    cffilter[0][index_QRCFFB%QRCFFB] = fl_Cwrenchraw.force;
    cffilter[1][index_QRCFFB%QRCFFB] = fr_Cwrenchraw.force;
    cffilter[2][index_QRCFFB%QRCFFB] = bl_Cwrenchraw.force;
    cffilter[3][index_QRCFFB%QRCFFB] = br_Cwrenchraw.force;
    cffilter[4][index_QRCFFB%QRCFFB] = fl_Cwrenchraw.torque;
    cffilter[5][index_QRCFFB%QRCFFB] = fr_Cwrenchraw.torque;
    cffilter[6][index_QRCFFB%QRCFFB] = bl_Cwrenchraw.torque;
    cffilter[7][index_QRCFFB%QRCFFB] = br_Cwrenchraw.torque;
    index_QRCFFB++;
    for(int i = 0; i < 8; i++)
    {
        for(int j = 0; j < QRCFFB; j++)
            cfbuf[i] += cffilter[i][j]/QRCFFB;
    }
    fl_Cwrench.force = m_cffilter*fl_Cwrench.force + fl_BtoBody*cfbuf[0]*(1-m_cffilter);
    fr_Cwrench.force = m_cffilter*fr_Cwrench.force+ fr_BtoBody*cfbuf[1]*(1-m_cffilter);
    bl_Cwrench.force = m_cffilter*bl_Cwrench.force + bl_BtoBody*cfbuf[2]*(1-m_cffilter);
    br_Cwrench.force = m_cffilter*br_Cwrench.force + br_BtoBody*cfbuf[3]*(1-m_cffilter);
    fl_Cwrench.torque = m_cffilter*fl_Cwrench.torque + fl_BtoBody*cfbuf[4]*(1-m_cffilter);
    fr_Cwrench.torque = m_cffilter*fr_Cwrench.torque + fr_BtoBody*cfbuf[5]*(1-m_cffilter);
    bl_Cwrench.torque =m_cffilter*bl_Cwrench.torque + bl_BtoBody*cfbuf[6]*(1-m_cffilter);
    br_Cwrench.torque = m_cffilter*br_Cwrench.torque + br_BtoBody*cfbuf[7]*(1-m_cffilter);

    //Matlab data transfer, macro CFROW should be less than that of matlab_app application programme.
        //The macro here should be according with that of matlab_app
    #define CFROW 30000
    #define CFCOLUMN 33
    static double cf[CFROW][CFCOLUMN];
    static ulong index = 0;
    cf[index][0] = limb_frontleft.GetContactForce().X();
    cf[index][1] = limb_frontleft.GetContactForce().Y();
    cf[index][2] = limb_frontleft.GetContactForce().Z();
    cf[index][3] = limb_frontright.GetContactForce().X();
    cf[index][4] = limb_frontright.GetContactForce().Y();
    cf[index][5] = limb_frontright.GetContactForce().Z();
    cf[index][6] = limb_backleft.GetContactForce().X();
    cf[index][7] = limb_backleft.GetContactForce().Y();
    cf[index][8] = limb_backleft.GetContactForce().Z();
    cf[index][9] = limb_backright.GetContactForce().X();
    cf[index][10] = limb_backright.GetContactForce().Y();
    cf[index][11] = limb_backright.GetContactForce().Z();

    cf[index][12] = fl_Cwrench.force.X();
    cf[index][13] = fl_Cwrench.force.Y();
    cf[index][14] = fl_Cwrench.force.Z();
    cf[index][15] = fr_Cwrench.force.X();
    cf[index][16] = fr_Cwrench.force.Y();
    cf[index][17] = fr_Cwrench.force.Z();
    cf[index][18] = bl_Cwrench.force.X();
    cf[index][19] = bl_Cwrench.force.Y();
    cf[index][20] = bl_Cwrench.force.Z();
    cf[index][21] = br_Cwrench.force.X();
    cf[index][22] = br_Cwrench.force.Y();
    cf[index][23] = br_Cwrench.force.Z();

    cf[index][24] = limb_frontleft.generalized_cf[0]-limb_frontleft.GetGeneralizedForce().X();
    cf[index][25] = limb_frontleft.generalized_cf[1]-limb_frontleft.GetGeneralizedForce().Y();
    cf[index][26] = limb_frontleft.generalized_cf[2]-limb_frontleft.GetGeneralizedForce().Z();

    cf[index][27] = limb_frontleft.GetThetaAccVel(1);
    cf[index][28] = limb_frontleft.GetThetaAccVel(2);
    cf[index][39] = limb_frontleft.GetThetaAccVel(3);

    cf[index][30] = m_accVel.X();
    cf[index][31] = m_accVel.Y();
    cf[index][32] = m_accVel.Z();
    index++;
    if( transfer_contact_force || index == CFROW)
    {
        matlabIdentMsg.Clear();
        for(int r = 0; r < CFROW; r++)
        {
            for(int c = 0; c < CFCOLUMN; c++)
            {
                matlabIdentMsg.add_contactforce(cf[r][c]);
            }
        }
        index = 0;
        transfer_contact_force = 0;
        pubMatlab.Publish(matlabIdentMsg);
    }

    switch(robot_state)
    {
    //Inertial parameters identification
    case IDENTIFICATION:
        IdentifyInertialParameters();
    break;
    default:
        //Control algorithm for various situations
        GaitAlgorithm();
    break;
    }
    //After state update, we should call this function to control robot
    ControlTransport();
}

void QuadRobot::SetJointsTargetPos(int indexlimb, int indextheta,double targetvalue, double angularVel)
{
    if(indexlimb > 4 || indexlimb <= 0 || indextheta > 3 || indextheta <= 0)
    {
        cout << "Error index in function SetJointsTorque !" << endl;
        return;
    }
/*
  * Now, we want to ensure a good trasitional process for a target pos. After experiment, the best transvelocity is ...
*/
    double currentPos = 0;
    double transVelPos = angularVel/m_factorTime;
    //Check the target value and transVelPos
    switch (indextheta%3)
    {
    case 1: //abs(theta1) must less than 1.5707
        if(abs(targetvalue) > 1.57)
        {
            if( targetvalue > 0)
            {     cout << "Limb" << indexlimb <<" : " << "Theta1 Out of range ! Targetvalue is "<< targetvalue <<" > 1.57, auto reset targetvalue : 1.57" <<endl; targetvalue = 1.57;}
            else
            {     cout << "Limb" << indexlimb <<" : " << "Theta1 Out of range ! Targetvalue is "<< targetvalue <<" < -1.57, auto reset targetvalue : -1.57" <<endl; targetvalue = -1.57; }
        }
        break;
    case 2:     //theta2 from PI/2 to 3*PI/2.
        if(targetvalue > 4.7)
         {     cout << "Limb" << indexlimb <<" : " << "Theta2 Out of range ! Targetvalue is "<< targetvalue <<" > 4.712, auto reset targetvalue : 4.7" <<endl; targetvalue = 4.7;}
        else if(targetvalue < 1.57)
         {     cout << "Limb" << indexlimb <<" : " << "Theta2 Out of range ! Targetvalue is "<< targetvalue <<" < 1.5707, auto reset targetvalue : 1.57" <<endl; targetvalue = 1.57; }
        break;
    case 0:     //theta3 from PI to 2*PI.
        if(targetvalue < 3.14159)
         {     cout << "Limb" << indexlimb <<" : " << "Theta3 Out of range ! Targetvalue is "<< targetvalue <<" < 3.14159, auto reset targetvalue : 3.14159" <<endl; targetvalue = 3.14159;}
        else if(targetvalue > 6.109)
         {     cout << "Limb" << indexlimb <<" : " << "Theta3 Out of range ! Targetvalue is "<< targetvalue <<" > 6.109, auto reset targetvalue : 6.109" <<endl; targetvalue = 6.109;}
        break;
    }
    if(transVelPos < 0)
    {     cout << "Angular speed is negative! TransVelPos value is " << transVelPos <<" SetJointsTargetPos() Failed. "<< endl; return;}

    //Get current selected joint positon
    switch(indexlimb)
    {
    case 1:
        currentPos = limb_frontleft.GetThetaPos(indextheta);
#ifdef QRDEBUG_JOINTPOS
        cout << "limb "<< indexlimb <<", joint" << indextheta << ", currentPos: "<< currentPos << endl;
#endif
        break;
    case 2:
        currentPos = limb_frontright.GetThetaPos(indextheta);
#ifdef QRDEBUG_JOINTPOS
        cout << "limb "<< indexlimb <<", joint" << indextheta << ", currentPos: "<< currentPos << endl;
#endif
        break;
    case 3:
        currentPos = limb_backleft.GetThetaPos(indextheta);
#ifdef QRDEBUG_JOINTPOS
        cout << "limb "<< indexlimb <<", joint" << indextheta << ", currentPos: "<< currentPos << endl;
#endif
        break;
    case 4:
        currentPos = limb_backright.GetThetaPos(indextheta);
#ifdef QRDEBUG_JOINTPOS
        cout << "limb "<< indexlimb <<", joint" << indextheta << ", currentPos: "<< currentPos << endl;
#endif
        break;
    }

    //Generate transitional positions using TPG class
    int pointsum = static_cast<int>(abs(currentPos - targetvalue) / transVelPos);
    if(!pointsum)
        pointsum = 1;
    qrmath.linearTPG[(indexlimb-1)*3+(indextheta-1)].Init(1, currentPos, targetvalue, pointsum);
    //cout << "indexlimb: "<< indexlimb << " indextheta: " << indextheta << " targetvalue: " << targetvalue << "pointsum :" << pointsum << endl;
    SetControlMode(indexlimb, indextheta, CONTROL_MODE::POSITION);
}

void QuadRobot::SetJointsPIDTargetPos(int indexlimb, int indextheta,double targetvalue)
{
    ctrl_jointsPos[3*(indexlimb-1) + indextheta-1] = targetvalue;
}

void QuadRobot::SetJointsTorque(int indexlimb, int indextheta,double torque)
{
    if(indexlimb > 4 || indexlimb <= 0 || indextheta > 3 || indextheta <= 0)
    {
        cout << "Error index in function SetJointsTorque !" << endl;
        return;
    }
    ctrl_jointsTor[3*(indexlimb-1) + indextheta-1] = torque;
    SetControlMode(indexlimb, indextheta, CONTROL_MODE::TORQUE);
}

void QuadRobot::GaitAlgorithm()
{
    /*
     * This function is called at 1/m_steptime
     * We should implement all gait algorithm in this function
     * Whatever state the robot will go into, the first job is adjusting attitude include height, joints' angle
    */
    static ulong state = 1;
    double quadperiod = 0.3;  //second, not > 0.1 now
    ulong interperiod = quadperiod/m_steptime;
    Vector3d f_supportposition(-m_exp_vel*quadperiod, 0, m_expheight);
    Vector3d f_1_supportposition(m_exp_vel*quadperiod, 0, m_expheight);
    Vector3d swingposition(m_exp_vel*quadperiod, 0 , 0.4);

    switch(robot_state)
    {
    case STOP:

        break;
    case WALK:
        break;
    case TROT:
        switch(state)
        {
        case 1: //four limbs support
            if(GetTimeCounter() % interperiod  == 0)
            {
                //forward
                SetFootPointPos(1, f_supportposition, quadperiod);
                SetFootPointPos(4, f_supportposition, quadperiod);
                SetFootPointPos(2, swingposition, quadperiod);
                SetFootPointPos(3, swingposition, quadperiod);
                state = 2;
            }
            break;
        case 2:
            if(GetTimeCounter() % interperiod == 0)
            {
                SetFootPointPos(1, f_supportposition, quadperiod);
                SetFootPointPos(4, f_supportposition, quadperiod);
                SetFootPointPos(2, f_1_supportposition, quadperiod);
                SetFootPointPos(3, f_1_supportposition, quadperiod);
                state = 3;
            }
            break;
        case 3:
            if(GetTimeCounter() % interperiod  == 0)
            {
                SetFootPointPos(1, swingposition, quadperiod);
                SetFootPointPos(4, swingposition, quadperiod);
                SetFootPointPos(2, f_supportposition, quadperiod);
                SetFootPointPos(3, f_supportposition, quadperiod);
                state = 4;
            }
            break;
        case 4: //four limbs support
            if(GetTimeCounter() % interperiod  == 0)
            {
                SetFootPointPos(1, f_1_supportposition, quadperiod);
                SetFootPointPos(4, f_1_supportposition, quadperiod);
                SetFootPointPos(2, f_supportposition, quadperiod);
                SetFootPointPos(3, f_supportposition, quadperiod);
                state = 1;
            }
        }
        break;
    case RUN:
        break;
    case JUMP:

        break;
    }
}

void QuadRobot::ControlTransport()
{
    /*This funtion called rate is 200Hz??? */
    for(int i = 1; i <= 4; i++)
    {
        for (int j = 1; j <= 3; j++)
        {
            switch(ctrl_modesel[(i-1)*3+j-1])
            {
            case CONTROL_MODE::POSITION:
                SetJointsPIDTargetPos(i, j, qrmath.linearTPG[(i-1)*3+j-1].generator());
                break;
            case CONTROL_MODE::TORQUE:
                break;
            }
        }
    }
    //For position control, every 5ms once, and every 1ms once in function ProcessJointsPositionData for torque control.???
   TransPosCtlMessage();
}

void QuadRobot::TransPosCtlMessage()
{
    controlMsg.Clear();
    controlMsg.set_type(CONTROL_MODE::POSITION);
    for(int i = 0; i < 12; i++)
    {
        controlMsg.add_mode(ctrl_modesel[i]);
        if(ctrl_modesel[i] == CONTROL_MODE::POSITION)
        {
            controlMsg.add_position(ctrl_jointsPos[i]);
            //cout << "ctrl_JointsPos["<< i << "]=" << ctrl_jointsPos[i] << endl;
        }
    }
    pubControl.Publish(controlMsg);
}

void QuadRobot::TransTorCtlMessage()
{
    controlMsg.Clear();
    controlMsg.set_type(CONTROL_MODE::TORQUE);
    for(int i = 0; i < 12; i++)
    {
        controlMsg.add_mode(ctrl_modesel[i]);
        if(ctrl_modesel[i] == CONTROL_MODE::TORQUE )
            controlMsg.add_torque(ctrl_jointsTor[i]);
    }
    pubControl.Publish(controlMsg);
}

bool QuadRobot::SetFootPointPos(int indexlimb, const Vector3d& position, double execute_time,  INV_KIN_SOLUTION_TYPE  type)
{
    bool result = false;
    double mindistance = 100000000, distance = 0;
    double v1 = 0, v2 = 0, v3 = 0;
    ulong choice = 0;
    LimbModel* plimbmodel[4] = {&limb_frontleft, &limb_frontright, &limb_backleft, &limb_backright};
    double c_theta1 = plimbmodel[indexlimb-1]->GetThetaPos(1);
    double c_theta2 = plimbmodel[indexlimb-1]->GetThetaPos(2);
    double c_theta3 = plimbmodel[indexlimb-1]->GetThetaPos(3);

    Vector3d target, c_thetas;
    ulong min_dist_index = 0;
    //Get expected target angles
    result = plimbmodel[indexlimb-1]->CalInvertKinetics(position);

    if(!result)
    {
        cout <<"Invert kinetics in function \"SetFootPointPos\": No solution to position: " << position << endl;
        return false;
    }
    //choose a expected solution
    c_thetas.X() = c_theta1;
    c_thetas.Y() = c_theta2;
    c_thetas.Z() = c_theta3;
    switch(type)
    {
    case MINI_DISTANCE:
        for(ulong i = 0; i < plimbmodel[indexlimb-1]->GetInvAnswer().size() ; i++)
        {
            distance = c_thetas.Distance(plimbmodel[indexlimb-1]->GetInvAnswer()[i]);
            if(distance < mindistance)
            {
                mindistance = distance;
                min_dist_index = i;
            }
        }
        target = plimbmodel[indexlimb-1]->GetInvAnswer()[min_dist_index];
        break;
    case MANUAL:
        cout << "Choosing a solution: " << endl;
        for(ulong i = 0; i < plimbmodel[indexlimb-1]->GetInvAnswer().size() ; i++)
        {
           cout << "solution " << i+1 << " in rad:" << "theta1 " << plimbmodel[indexlimb-1]->GetInvAnswer()[i].X() << "theta2 " << plimbmodel[indexlimb-1]->GetInvAnswer()[i].Y() << \
                 "theta3 " << plimbmodel[indexlimb-1]->GetInvAnswer()[i].Z() << endl;
        }
        cout << "Enter a number to choose a solution : " << endl;
        cin >> choice;
        target = plimbmodel[indexlimb-1]->GetInvAnswer()[choice-1];
        break;
    }

    v1 = abs(target.X() - c_theta1)/execute_time;
    v2 = abs(target.Y() - c_theta2)/execute_time;
    v3 = abs(target.Z() - c_theta3)/execute_time;

    SetJointsTargetPos(indexlimb, 1, target.X(), v1);
    SetJointsTargetPos(indexlimb, 2, target.Y(), v2);
    SetJointsTargetPos(indexlimb, 3, target.Z(), v3);

    return true;
}

void QuadRobot::TrotGait(double velocity)
{
    robot_state = ROBOT_STATE::TROT;
    m_exp_vel = velocity;
    //Calculate the gait period based on velocity

}

void QuadRobot::JumpGait()
{
    robot_state = ROBOT_STATE::JUMP;
}

void QuadRobot::IdentificationCOMPosition()
{
    /*TODO!! Only the body in level, this function behave properly.*/
    m_bodyCOMValid = true;
    double F1 = limb_frontleft.GetContactForce().Z(), F2 = limb_frontright.GetContactForce().Z(), F3 = limb_backleft.GetContactForce().Z(), F4 = limb_backright.GetContactForce().Z();
    double x1 = limborg_frontleft.X()+limb_frontleft.GetMainPointPos(3).X(),  x2 = limborg_frontright.X()+limb_frontright.GetMainPointPos(3).X();
    double x3 = limborg_backleft.X()+limb_backleft.GetMainPointPos(3).X(),  x4 = limborg_backright.X()+limb_backright.GetMainPointPos(3).X();
    double y1 = limborg_frontleft.Y()+limb_frontleft.GetMainPointPos(3).Y(), y2 = limborg_frontright.Y()+limb_frontright.GetMainPointPos(3).Y();
    double y3 = limborg_backleft.Y()+limb_backleft.GetMainPointPos(3).Y(),  y4 = limborg_backright.Y()+limb_backright.GetMainPointPos(3).Y();
    //Force equilibrium
    double Fbody = -(F1+F2+F3+F4);
    //Torque equilibrium
    m_bodyCOM.Y() = (F1*y1+F2*y2+F3*y3+F4*y4)/(-Fbody);
    m_bodyCOM.X() = (F1*x1+F2*x2+F3*x3+F4*x4)/(-Fbody);
    m_bodyCOM.Z() = 0;
}

void QuadRobot::IdentifyInertialParameters()
{
    GetLock();
/*Identification using Linear least square method. WX = Y, X is the vector of parameters to be identified*/
    //First we construnct a matrix to store W matrix data and set three joints' angls magnitude. t means time, finish indicates whether all data has been attained, omega means w in sin(wt).
    //The value of macro ROW should be an integer multiple of joints number of one limb.
#define IDENTI_TIME 4
#define ROW 3*IDENTI_TIME*1000
#define COLUMN 40
    static double W[ROW][COLUMN], torque[ROW], appliedTorque[3];
    double mag1 = 6, mag2 = 6, mag3 = 4.5;
    static double l1 = limb_frontleft.GetLink()[0]->GetLength(), l2 = limb_frontleft.GetLink()[1]->GetLength();
    //Choose a excitation trajectories. Here we use a sine curve.
    static ulong index = 0; static double g = 9.8, t1 = 0, t2 = 0, t3 = 0;
    double force_period = 1;
    ulong test = 1;
    if(GetTimeCounter() % test ==0)
    {
        t1 += test*m_steptime;
        appliedTorque[0] = mag1*sin(force_period*PI*t1);
        t2 += test*m_steptime;
        appliedTorque[1] = mag2*sin(force_period*PI*t2);
        t3 += test*m_steptime;
        appliedTorque[2] = mag3*sin(force_period*PI*t3)+mag3;
    }
        //Get the joints' torque from real/simulation environment
    torque[index] = fl_Jwrench[0].torque.X();
    torque[index+1] =  -fl_Jwrench[1].torque.Y();
    torque[index+2] = fl_Jwrench[2].torque.Y();

        //Fill the W matrix based on the joints' trajectories.
    double theta1 = limb_frontleft.GetThetaPos(1), theta2 = limb_frontleft.GetThetaPos(2), theta3 = limb_frontleft.GetThetaPos(3);
    double dtheta1 = limb_frontleft.GetThetaVel(1), dtheta2 = limb_frontleft.GetThetaVel(2), dtheta3 = limb_frontleft.GetThetaVel(3);
    double ddtheta1 = limb_frontleft.GetThetaAccVel(1), ddtheta2 = limb_frontleft.GetThetaAccVel(2), ddtheta3 = limb_frontleft.GetThetaAccVel(3);
        //just for debug
    W[index][36] = theta1;
    W[index+1][36] = theta2;
    W[index+2][36] = theta3;

    W[index][37] = dtheta1;
    W[index+1][37] = dtheta2;
    W[index+2][37] = dtheta3;

    W[index][38] = ddtheta1;
    W[index+1][38] = ddtheta2;
    W[index+2][38] = ddtheta3;

    W[index][39] = limb_frontleft.generalized_cf[0];
    W[index+1][39] = limb_frontleft.generalized_cf[1];
    W[index+2][39] = limb_frontleft.generalized_cf[2];
    //first row
    for(int i = 1; i < 7; i++)
    {
         W[index][i] = 0;
    }
    W[index][0] = ddtheta1;
    W[index][7] = -g*cos(theta1);
    W[index][8] = g*sin(theta1);
    W[index][9] = 0;
    W[index][10] = dtheta1;
    W[index][11] = sgn(dtheta1);

    W[index][12] = ddtheta1*pow(cos(theta2),2)-dtheta1*dtheta2*sin(2*theta2);
    W[index][13] = 0;
    W[index][14] = ddtheta1*pow(sin(theta2),2)+dtheta1*dtheta2*sin(2*theta2);
    W[index][15] = ddtheta2*cos(theta2)-pow(dtheta2,2)*sin(theta2);
    W[index][16] = -ddtheta1*sin(2*theta2)-2*dtheta1*dtheta2*cos(2*theta2);
    W[index][17] = -ddtheta2*sin(theta2)-pow(dtheta2,2)*cos(theta2);
    W[index][18] = -2*l1*ddtheta1*sin(theta2)-2*l1*dtheta1*dtheta2*cos(theta2)-g*sin(theta1)*sin(theta2);
    W[index][19] = -g*cos(theta1);
    W[index][20] = -2*l1*ddtheta1*cos(theta2)+2*l1*dtheta1*dtheta2*sin(theta2)-g*sin(theta1)*cos(theta2);
    W[index][21] = pow(l1,2)*ddtheta1+l1*g*sin(theta1);
    W[index][22] = 0;
    W[index][23] = 0;

    W[index][24] = ddtheta1*pow(cos(theta3-theta2),2)+(dtheta1*dtheta2-dtheta1*dtheta3)*sin(2*(theta3-theta2));
    W[index][25] = 0;
    W[index][26] = ddtheta1*pow(sin(theta3-theta2),2)-(dtheta1*dtheta2-dtheta1*dtheta3)*sin(2*(theta3-theta2));
    W[index][27] = (-ddtheta2+ddtheta3)*cos(theta3-theta2)-(pow(dtheta2,2)-2*dtheta2*dtheta3+pow(dtheta3,2))*sin(theta3-theta2);
    W[index][28] = sin(2*(theta3-theta2))*ddtheta1+2*(-dtheta1*dtheta2+dtheta1*dtheta3)*cos(2*(theta3-theta2));
    W[index][29] = (ddtheta3-ddtheta2)*sin(theta3-theta2)+(pow(dtheta2,2)-2*dtheta2*dtheta3+pow(dtheta3,2))*cos(theta3-theta2);
    W[index][30] = -2*(l1-l2*cos(theta2))*(sin(theta3-theta2)*ddtheta1+cos(theta3-theta2)*dtheta1*dtheta3)+2*(l1*cos(theta3-theta2)-l2*cos(2*theta2-theta3))*dtheta1*dtheta2-g*sin(theta1)*sin(theta3-theta2);
    W[index][31] = l2*ddtheta2*sin(theta2)+l2*pow(dtheta2,2)*cos(theta2)-g*cos(theta1);
    W[index][32] = 2*(l1-l2*cos(theta2))*(cos(theta3-theta2)*ddtheta1-sin(theta3-theta2)*dtheta1*dtheta3)+2*(l1*sin(theta3-theta2)+l2*sin(2*theta2-theta3))*dtheta1*dtheta2+g*sin(theta1)*cos(theta3-theta2);
    W[index][33] = pow(l1-l2*cos(theta2),2)*ddtheta1+2*(l1-l2*cos(theta2))*l2*dtheta1*dtheta2*sin(theta2)+(l1-l2*cos(theta2))*g*sin(theta1);
    W[index][34] = 0;
    W[index][35] = 0;

        //second row
    for(int i = 0; i < 12; i++)
    {
         W[index+1][i] = 0;
    }
    W[index+1][12] = 0.5*pow(dtheta1,2)*sin(2*theta2);
    W[index+1][13] = ddtheta2;
    W[index+1][14] = -W[index+1][12];
    W[index+1][15] = cos(theta2)*ddtheta1;
    W[index+1][16] = pow(dtheta1,2)*cos(2*theta2);
    W[index+1][17] = -sin(theta2)*ddtheta1;
    W[index+1][18] = l1*pow(dtheta1,2)*cos(theta2)+g*cos(theta1)*cos(theta2);
    W[index+1][19] = 0;
    W[index+1][20] = -l1*pow(dtheta1,2)*sin(theta2)-g*cos(theta1)*sin(theta2);
    W[index+1][21] = 0;
    W[index+1][22] = dtheta2;
    W[index+1][23] = sgn(dtheta2);

    W[index+1][24] = -0.5*pow(dtheta1,2)*sin(2*(theta3-theta2));
    W[index+1][25] = ddtheta2-ddtheta3;
    W[index+1][26] = -W[index+1][24];
    W[index+1][27] = -cos(theta3-theta2)*ddtheta1;
    W[index+1][28] = pow(dtheta1,2)*cos(2*(theta3-theta2));
    W[index+1][29] =  -sin(theta3-theta2)*ddtheta1;
    W[index+1][30] = 2*l2*sin(theta3)*ddtheta2-l2*ddtheta3*sin(theta3)+(-l1*cos(theta3-theta2)+l2*cos(2*theta2-theta3))*pow(dtheta1,2)+2*l2*dtheta2*dtheta3*cos(theta3)-l2*pow(dtheta3,2)*cos(theta3)\
            - g*cos(theta1)*cos(theta3-theta2);
    W[index+1][31] = l2*sin(theta2)*ddtheta1;
    W[index+1][32] = -2*l2*cos(theta3)*ddtheta2+l2*ddtheta3*cos(theta3)-(l1*sin(theta3-theta2)+l2*sin(2*theta2-theta3))*pow(dtheta1,2)+2*l2*dtheta2*dtheta3*sin(theta3)-l2*pow(dtheta3,2)*sin(theta3)\
            - g*cos(theta1)*sin(theta3-theta2);
    W[index+1][33] = pow(l2,2)*ddtheta2-l2*sin(theta2)*(l1-l2*cos(theta2))*pow(dtheta1,2)-g*l2*cos(theta1)*sin(theta2);
    W[index+1][34] = 0;
    W[index+1][35] = 0;

        //third row
    for(int i = 0; i < 24; i++)
    {
         W[index+2][i] = 0;
    }
    W[index+2][24] = 0.5*pow(dtheta1,2)*sin(2*(theta3-theta2));
    W[index+2][25] = ddtheta3-ddtheta2;
    W[index+2][26] = - W[index+2][24];
    W[index+2][27] = cos(theta3-theta2)*ddtheta1;
    W[index+2][28] = -pow(dtheta1,2)*cos(2*(theta3-theta2));
    W[index+2][29] = sin(theta3-theta2)*ddtheta1;
    W[index+2][30] = (l1-l2*cos(theta2))*pow(dtheta1,2)*cos(theta3-theta2)-l2*pow(dtheta2,2)*cos(theta3) - l2*sin(theta3)*ddtheta2 +g*cos(theta1)*cos(theta3-theta2);
    W[index+2][31] = 0;
    W[index+2][32] = (l1-l2*cos(theta2))*pow(dtheta1,2)*sin(theta3-theta2)-l2*pow(dtheta2,2)*sin(theta3) + l2*cos(theta3)*ddtheta2 +g*cos(theta1)*sin(theta3-theta2);
    W[index+2][33] = 0;
    W[index+2][34] = dtheta3;
    W[index+2][35] = sgn(dtheta3);

    //If W matrix has been full, the process of sampling data will be finished.Change the robot state to STOP, then transport W matrix to matlab communication program.
    if(index == ROW-3)
    {
        t1=0;
        t2=0;
        t3=0;
        index = 0;
        robot_state = QuadRobot::STOP;
        for(int r = 0; r < ROW; r++)
        {
            matlabIdentMsg.add_torque(torque[r]);
            for(int c = 0; c < COLUMN; c++)
            {
                matlabIdentMsg.add_matelement(W[r][c]);
            }
        }
        pubMatlab.Publish(matlabIdentMsg);
    }
    SetJointsTorque(1,1,  appliedTorque[0] );
    SetJointsTorque(1,2,  appliedTorque[1] );
    SetJointsTorque(1,3,  appliedTorque[2] );
    index += 3;
    ReleaseLock();
}

void QuadRobot::IdentifyInertialParametersUsingNoAccDynamics()
{
    GetLock();
/*Identification using Linear least square method. WX = Y, X is the vector of parameters to be identified*/
    //First we construnct a matrix to store W matrix data and set four joints' angls magnitude. t means time, finish indicates whether all data has been attained, omega means w in sin(wt).
    //The value of macro ROW should be an integer multiple of 4.
#define IDENTI_TIME 4
#define ROW 3*IDENTI_TIME*1000
#define COLUMN 40
    //Identification Matrix term
    static double L[ROW][COLUMN], torqueTrans[ROW], torque[4], torque_integral[4];
    //Integral term. Not every element in the term are used.
    static double L_Integralsum[4][COLUMN];
    static double l1 = limb_frontleft.GetLink()[0]->GetLength(), l2 = limb_frontleft.GetLink()[1]->GetLength(), l3 = limb_frontleft.GetLink()[2]->GetLength(), l4 = limb_frontleft.GetLink()[3]->GetLength();
    //Choose a excitation trajectories. Here we use a sine curve.
    static ulong index = 0;
    double ax = 0, ay = 0, az = 9.8;
    //Joint angle and velocity.
    static double theta1_t0 =limb_frontleft.GetThetaPos(1), theta2_t0 =limb_frontleft.GetThetaPos(2), theta3_t0 =limb_frontleft.GetThetaPos(3), theta4_t0 =limb_frontleft.GetThetaPos(4);
    static double dtheta1_t0 =limb_frontleft.GetThetaVel(1), dtheta2_t0 =limb_frontleft.GetThetaVel(2), dtheta3_t0 =limb_frontleft.GetThetaVel(3), dtheta4_t0 =limb_frontleft.GetThetaVel(4);
    double theta1 = limb_frontleft.GetThetaPos(1), theta2 = limb_frontleft.GetThetaPos(2), theta3 = limb_frontleft.GetThetaPos(3), theta4 = limb_frontleft.GetThetaPos(4);
    double dtheta1 = limb_frontleft.GetThetaVel(1), dtheta2 = limb_frontleft.GetThetaVel(2), dtheta3 = limb_frontleft.GetThetaVel(3), dtheta4 = limb_frontleft.GetThetaVel(4);

        //Reset integral time every 100 steps in order to avoid accumulated error
/*    if(GetTimeCounter()%100 == 0)
    {
        dtheta1_t0 =limb_frontleft.GetThetaVel(1);
        dtheta2_t0 =limb_frontleft.GetThetaVel(2);
        dtheta3_t0 =limb_frontleft.GetThetaVel(3);
        dtheta4_t0 =limb_frontleft.GetThetaVel(4);
        memset(L_Integralsum, 0, sizeof(L_Integralsum));
        memset(torque_integral, 0, sizeof(torque_integral));
    }*/
    //Get the joints' torque from real/simulation environment
    torque[0] = fl_Jwrench[0].torque.X();
    torque[1] =  -fl_Jwrench[1].torque.Y();
    torque[2] = fl_Jwrench[2].torque.Y();
    torque[3] = - fl_Jwrench[3].torque.Y();
    //Double integral of torque
    for(ulong i = 0; i < 4; i++)
    {
        torque_integral[i] += torque[i]*m_steptime;
        torqueTrans[index+i] = torque_integral[i];
    }
    //Torque magnitude
    double mag1 = 6, mag2 = 6, mag3 = 3, mag4 = 5;
    double force_period = 1;
    double appliedTorque[4];
    static double  t1 = 0, t2 = 0, t3 = 0, t4 = 0;
    t1 += m_steptime;
    appliedTorque[0] = mag1*sin(force_period*PI*t1);
    t2 += m_steptime;
    appliedTorque[1] = mag2*sin(force_period*PI*t2) + mag2/2+4.5;
    t3 += m_steptime;
    appliedTorque[2] = -mag3*sin(force_period*PI*t3)-mag3/2-7.5;
    t4 += m_steptime;
    appliedTorque[3] = mag4*sin(force_period*PI*t4) + mag4/2+3;
        //first row
    for(int i = 1; i <7; i++)
    {
        L[index][i] = 0;
    }
    L[index][0] = dtheta1 - dtheta1_t0;
    L_Integralsum[0][7] += m_steptime*(ay*sin(theta1)-az*cos(theta1));
    L[index][7] =  L_Integralsum[0][7];
    L_Integralsum[0][8] += m_steptime*(ay*cos(theta1)+az*sin(theta1));
    L[index][8] = L_Integralsum[0][8];
    L[index][9] = 0;
    L_Integralsum[0][10] += m_steptime*dtheta1;
    L[index][10] = L_Integralsum[0][10];
    L_Integralsum[0][11] += m_steptime*sgn(dtheta1);
    L[index][11] = L_Integralsum[0][11];

    L[index][12] = pow(cos(theta2),2)*dtheta1-pow(cos(theta2_t0),2)*dtheta1_t0;
    L[index][13] = 0;
    L[index][14] = pow(sin(theta2),2)*dtheta1-pow(sin(theta2_t0),2)*dtheta1_t0;
    L[index][15] = cos(theta2)*dtheta2- cos(theta2_t0)*dtheta2_t0;
    L[index][16] = -sin(2*theta2)*dtheta1+sin(2*theta2_t0)*dtheta1_t0;
    L[index][17] = -sin(theta2)*dtheta2+sin(theta2_t0)*dtheta2_t0;
    L_Integralsum[0][18] += m_steptime*sin(theta2)*(-ay*cos(theta1)-az*sin(theta1));
    L[index][18] = -2*l1*(sin(theta2)*dtheta1-sin(theta2_t0)*dtheta1_t0)+L_Integralsum[0][18];
    L_Integralsum[0][19] += m_steptime*(ay*sin(theta1)-az*cos(theta1));
    L[index][19] = L_Integralsum[0][19];
    L_Integralsum[0][20] += m_steptime*cos(theta2)*(-ay*cos(theta1)-az*sin(theta1));
    L[index][20] = -2*l1*(cos(theta2)*dtheta1-cos(theta2_t0)*dtheta1_t0)+L_Integralsum[0][20];
    L_Integralsum[0][21] += m_steptime*l1*(ay*cos(theta1)+az*sin(theta1));
    L[index][21] = pow(l1,2)*(dtheta1-dtheta1_t0)+L_Integralsum[0][21];
    L[index][22] = 0;
    L[index][23] = 0;

    L[index][24] = pow(cos(theta3-theta2),2)*dtheta1-pow(cos(theta3_t0-theta2_t0),2)*dtheta1_t0;
    L[index][25] = 0;
    L[index][26] = pow(sin(theta3-theta2),2)*dtheta1-pow(sin(theta3_t0-theta2_t0),2)*dtheta1_t0;
    L[index][27] = cos(theta3-theta2)*(-dtheta2+dtheta3)-cos(theta3_t0-theta2_t0)*(-dtheta2_t0+dtheta3_t0);
    L[index][28] = sin(2*(theta3-theta2))*dtheta1-sin(2*(theta3_t0-theta2_t0))*dtheta1_t0;
    L[index][29] = sin(theta3-theta2)*(dtheta3-dtheta2)-sin(theta3_t0-theta2_t0)*(dtheta3_t0-dtheta2_t0);
    L_Integralsum[0][30] += m_steptime*sin(theta3-theta2)*(-ay*cos(theta1)-az*sin(theta1));
    L[index][30] = -2*sin(theta3-theta2)*(l1-l2*cos(theta2))*dtheta1+2*sin(theta3_t0-theta2_t0)*(l1-l2*cos(theta2_t0))*dtheta1_t0+L_Integralsum[0][30];
    L_Integralsum[0][31] += m_steptime*(ay*sin(theta1)-az*cos(theta1));
    L[index][31] = l2*sin(theta2)*dtheta2-l2*sin(theta2_t0)*dtheta2_t0+L_Integralsum[0][31];
    L_Integralsum[0][32] += m_steptime*cos(theta3-theta2)*(ay*cos(theta1)+az*sin(theta1));
    L[index][32] = 2*cos(theta3-theta2)*(l1-l2*cos(theta2))*dtheta1-2*cos(theta3_t0-theta2_t0)*(l1-l2*cos(theta2_t0))*dtheta1_t0+L_Integralsum[0][32];
    L_Integralsum[0][33] += m_steptime*(l1-l2*cos(theta2))*(ay*cos(theta1)+az*sin(theta1));
    L[index][33] = pow(l1-l2*cos(theta2),2)*dtheta1-pow(l1-l2*cos(theta2_t0),2)*dtheta1_t0+L_Integralsum[0][33];
    L[index][34] = 0;
    L[index][35] = 0;

    L[index][36] = pow(cos(theta2+theta4-theta3),2)*dtheta1-pow(cos(theta2_t0+theta4_t0-theta3_t0),2)*dtheta1_t0;
    L[index][37] = 0;
    L[index][38] = pow(sin(theta2+theta4-theta3),2)*dtheta1-pow(sin(theta2_t0+theta4_t0-theta3_t0),2)*dtheta1_t0;
    L[index][39] = cos(theta2+theta4-theta3)*(dtheta2-dtheta3+dtheta4)-cos(theta2_t0+theta4_t0-theta3_t0)*(dtheta2_t0-dtheta3_t0+dtheta4_t0);
    L[index][40] = -sin(2*(theta2+theta4-theta3))*dtheta1+sin(2*(theta2_t0+theta4_t0-theta3_t0))*dtheta1_t0;
    L[index][41] = -sin(theta2+theta4-theta3)*(dtheta2-dtheta3+dtheta4)+sin(theta2_t0+theta4_t0-theta3_t0)*(dtheta2_t0-dtheta3_t0+dtheta4_t0);
    L_Integralsum[0][42] += m_steptime*sin(theta2+theta4-theta3)*(-ay*cos(theta1)-az*sin(theta1));
    L[index][42] = -2*sin(theta2+theta4-theta3)*(l1-l2*cos(theta2)+l3*cos(theta2-theta3))*dtheta1+2*sin(theta2_t0+theta4_t0-theta3_t0)*(l1-l2*cos(theta2_t0)+l3*cos(theta2_t0-theta3_t0))*dtheta1_t0\
                                +L_Integralsum[0][42];
    L_Integralsum[0][43] += m_steptime*(ay*sin(theta1)-az*cos(theta1));
    L[index][43] = l2*(sin(theta2)*dtheta2-sin(theta2_t0)*dtheta2_t0)+l3*sin(theta2-theta3)*(-dtheta2+dtheta3)-l3*sin(theta2_t0-theta3_t0)*(-dtheta2_t0+dtheta3_t0)+L_Integralsum[0][43];
    L_Integralsum[0][44] += m_steptime*cos(theta2+theta4-theta3)*(-ay*cos(theta1)-az*sin(theta1));
    L[index][44] = -2*cos(theta2+theta4-theta3)*(l1-l2*cos(theta2)+l3*cos(theta2-theta3))*dtheta1+2*cos(theta2_t0+theta4_t0-theta3_t0)*(l1-l2*cos(theta2_t0)+l3*cos(theta2_t0-theta3_t0))*dtheta1_t0 \
                                +L_Integralsum[0][44];
    L_Integralsum[0][45] += m_steptime*(l1-l2*cos(theta2)+l3*cos(theta2-theta3))*(ay*cos(theta1)+az*sin(theta1));
    L[index][45]  = pow(l1-l2*cos(theta2)+l3*cos(theta2-theta3),2)*dtheta1-pow(l1-l2*cos(theta2_t0)+l3*cos(theta2_t0-theta3_t0),2)*dtheta1_t0+L_Integralsum[0][45];
    L[index][46] = 0;
    L[index][47] = 0;
         //second row
    for(int i = 0; i < 12; i++)
    {
        L[index+1][i] = 0;
    }
    L_Integralsum[1][12] += m_steptime*0.5*pow(dtheta1,2)*sin(2*theta2);
    L[index+1][12] = L_Integralsum[1][12];
    L[index+1][13] = dtheta2-dtheta2_t0;
    L[index+1][14] = -L_Integralsum[1][12];
    L_Integralsum[1][15] += m_steptime*sin(theta2)*dtheta1*dtheta2;
    L[index+1][15] = cos(theta2)*dtheta1-cos(theta2_t0)*dtheta1_t0 + L_Integralsum[1][15];
    L_Integralsum[1][16] += m_steptime*cos(2*theta2)*pow(dtheta1,2);
    L[index+1][16] = L_Integralsum[1][16];
    L_Integralsum[1][17] += m_steptime*cos(theta2)*dtheta1*dtheta2;
    L[index+1][17] = -sin(theta2)*dtheta1+sin(theta2_t0)*dtheta1_t0+L_Integralsum[1][17];
    L_Integralsum[1][18] += m_steptime*(l1*pow(dtheta1,2)*cos(theta2)-ax*sin(theta2)-ay*sin(theta1)*cos(theta2)+az*cos(theta1)*cos(theta2));
    L[index+1][18] = L_Integralsum[1][18];
    L[index+1][19] = 0;
    L_Integralsum[1][20] += m_steptime*(-l1*pow(dtheta1,2)*sin(theta2)-ax*cos(theta2)+ay*sin(theta1)*sin(theta2)-az*cos(theta1)*sin(theta2));
    L[index+1][20] =  L_Integralsum[1][20];
    L[index+1][21] = 0;
    L_Integralsum[1][22] += m_steptime*dtheta2;
    L[index+1][22] = L_Integralsum[1][22];
    L_Integralsum[1][23] += m_steptime*sgn(dtheta2);
    L[index+1][23] = L_Integralsum[1][23];

    L_Integralsum[1][24] += m_steptime*(-0.5*pow(dtheta1,2)*sin(2*theta3-2*theta2));
    L[index+1][24] = L_Integralsum[1][24];
    L[index+1][25] = dtheta2-dtheta3-dtheta2_t0+dtheta3_t0;
    L[index+1][26] = -L_Integralsum[1][24];
    L_Integralsum[1][27] += m_steptime*sin(theta3-theta2)*(dtheta1*dtheta2-dtheta1*dtheta3);
    L[index+1][27] = -cos(theta3-theta2)*dtheta1+cos(theta3_t0-theta2_t0)*dtheta1_t0+L_Integralsum[1][27];
    L_Integralsum[1][28] += m_steptime*pow(dtheta1,2)*cos(2*theta3-2*theta2);
    L[index+1][28] =L_Integralsum[1][28];
    L_Integralsum[1][29] += m_steptime*cos(theta3-theta2)*(-dtheta1*dtheta2+dtheta1*dtheta3);
    L[index+1][29] = -sin(theta3-theta2)*dtheta1+sin(theta3_t0-theta2_t0)*dtheta1_t0+L_Integralsum[1][29];
    L_Integralsum[1][30] += m_steptime*((-l1*cos(theta3-theta2)+l2*cos(2*theta2-theta3))*pow(dtheta1,2)-ax*sin(theta3-theta2)+ay*sin(theta1)*cos(theta3-theta2)-az*cos(theta1)*cos(theta3-theta2));
    L[index+1][30] = 2*l2*(sin(theta3)*dtheta2-sin(theta3_t0)*dtheta2_t0)-l2*(sin(theta3)*dtheta3-sin(theta3_t0)*dtheta3_t0)+L_Integralsum[1][30];
    L_Integralsum[1][31] += m_steptime*(-l2*cos(theta2)*dtheta1*dtheta2);
    L[index+1][31] = l2*(sin(theta2)*dtheta1-sin(theta2_t0)*dtheta1_t0)+L_Integralsum[1][31];
    L_Integralsum[1][32] += m_steptime*((-l1*sin(theta3-theta2)-l2*sin(2*theta2-theta3))*pow(dtheta1,2)+ax*cos(theta3-theta2)+ay*sin(theta1)*sin(theta3-theta2)-az*cos(theta1)*sin(theta3-theta2));
    L[index+1][32] = -2*l2*(cos(theta3)*dtheta2-cos(theta3_t0)*dtheta2_t0)+l2*(cos(theta3)*dtheta3-cos(theta3_t0)*dtheta3_t0)+L_Integralsum[1][32];
    L_Integralsum[1][33] += m_steptime*(-l2*sin(theta2)*(l1-l2*cos(theta2))*pow(dtheta1,2)-ax*l2*cos(theta2)+ay*l2*sin(theta1)*sin(theta2)-az*l2*cos(theta1)*sin(theta2));
    L[index+1][33] = pow(l2,2)*(dtheta2-dtheta2_t0)+L_Integralsum[1][33];
    L[index+1][34] = 0;
    L[index+1][35] = 0;

    L_Integralsum[1][36] += m_steptime*0.5*pow(dtheta1,2)*sin(2*(theta2+theta4-theta3));
    L[index+1][36] = L_Integralsum[1][36];
    L[index+1][37] = dtheta2-dtheta3+dtheta4-dtheta2_t0+dtheta3_t0-dtheta4_t0;
    L[index+1][38] = -L_Integralsum[1][36];
    L_Integralsum[1][39] += m_steptime*sin(theta2+theta4-theta3)*(dtheta1*dtheta2-dtheta1*dtheta3+dtheta1*dtheta4);
    L[index+1][39] = cos(theta2+theta4-theta3)*dtheta1-cos(theta2_t0+theta4_t0-theta3_t0)*dtheta1_t0+L_Integralsum[1][39];
    L_Integralsum[1][40] += m_steptime*pow(dtheta1,2)*cos(2*(theta2+theta4-theta3));
    L[index+1][40] = L_Integralsum[1][40];
    L_Integralsum[1][41] += m_steptime*cos(theta2+theta4-theta3)*(dtheta1*dtheta2-dtheta1*dtheta3+dtheta1*dtheta4);
    L[index+1][41] = -sin(theta2+theta4-theta3)*dtheta1+sin(theta2_t0+theta4_t0-theta3_t0)*dtheta1_t0+L_Integralsum[1][41];
    L_Integralsum[1][42] = m_steptime*((l1*cos(theta2+theta4-theta3)-l2*cos(2*theta2+theta4-theta3)+l3*cos(2*theta2+theta4-2*theta3))*pow(dtheta1,2)-ax*sin(theta2+theta4-theta3)\
                                                  -(ay*sin(theta1)-az*cos(theta1))*cos(theta2+theta4-theta3));
    L[index+1][42] = -(l2*sin(theta3-theta4)+l3*sin(theta4))*(2*dtheta2+dtheta4)+(l2*sin(theta3_t0-theta4_t0)+l3*sin(theta4_t0))*(2*dtheta2_t0+dtheta4_t0) \
                                     +(l2*sin(theta3-theta4)+2*l3*sin(theta4))*dtheta3-(l2*sin(theta3_t0-theta4_t0)+2*l3*sin(theta4_t0))*dtheta3_t0+L_Integralsum[1][42];
    L_Integralsum[1][43] = m_steptime*(-l2*cos(theta2)*dtheta1*dtheta2+l3*cos(theta2-theta3)*(dtheta1*dtheta2-dtheta1*dtheta3));
    L[index+1][43] = (l2*sin(theta2)-l3*sin(theta2-theta3))*dtheta1-(l2*sin(theta2_t0)-l3*sin(theta2_t0-theta3_t0))*dtheta1_t0+L_Integralsum[1][43];
    L_Integralsum[1][44] = m_steptime*(-(l1*sin(theta2+theta4-theta3)-l2*sin(2*theta2+theta4-theta3)+l3*sin(2*theta2+theta4-2*theta3))*pow(dtheta1,2)-ax*cos(theta2+theta4-theta3)\
                                       +(ay*sin(theta1)-az*cos(theta1))*sin(theta2+theta4-theta3));
    L[index+1][44] =  (l2*cos(theta3-theta4)-l3*cos(theta4))*(2*dtheta2+dtheta4)-(l2*cos(theta3_t0-theta4_t0)-l3*cos(theta4_t0))*(2*dtheta2_t0+dtheta4_t0) \
                                     +(-l2*cos(theta3-theta4)+2*l3*cos(theta4))*dtheta3-(-l2*cos(theta3_t0-theta4_t0)+2*l3*cos(theta4_t0))*dtheta3_t0+L_Integralsum[1][44];
    L_Integralsum[1][45] = m_steptime*(-(l1-l2*cos(theta2)+l3*cos(theta2-theta3))*(l2*sin(theta2)-l3*sin(theta2-theta3))*pow(dtheta1,2)-ax*(l2*cos(theta2)-l3*cos(theta2-theta3))\
                                                  -(-ay*sin(theta1)+az*cos(theta1))*(l2*sin(theta2)-l3*sin(theta2-theta3)));
    L[index+1][45] = (pow(l2,2)+pow(l3,2)-2*l2*l3*cos(theta3))*dtheta2-(pow(l2,2)+pow(l3,2)-2*l2*l3*cos(theta3_t0))*dtheta2_t0 \
                                     +(l2*l3*cos(theta3)-pow(l3,2))*dtheta3-(l2*l3*cos(theta3_t0)-pow(l3,2))*dtheta3_t0+L_Integralsum[1][45];
    L[index+1][46] = 0;
    L[index+1][47] = 0;
         //third row
    for(int i = 0; i < 24; i++)
    {
        L[index+2][i] = 0;
    }
    L_Integralsum[2][24] += m_steptime*0.5*pow(dtheta1,2)*sin(2*(theta3-theta2));
    L[index+2][24] = L_Integralsum[2][24];
    L[index+2][25] = dtheta3-dtheta2-dtheta3_t0+dtheta2_t0;
    L[index+2][26] = -L_Integralsum[2][24];
    L_Integralsum[2][27] += m_steptime*sin(theta3-theta2)*(-dtheta1*dtheta2+dtheta1*dtheta3);
    L[index+2][27] = cos(theta3-theta2)*dtheta1-cos(theta3_t0-theta2_t0)*dtheta1_t0+L_Integralsum[2][27];
    L_Integralsum[2][28] += m_steptime*(-pow(dtheta1,2))*cos(2*(theta3-theta2));
    L[index+2][28] = L_Integralsum[2][28];
    L_Integralsum[2][29] += m_steptime*cos(theta3-theta2)*(dtheta1*dtheta2-dtheta1*dtheta3);
    L[index+2][29] = sin(theta3-theta2)*dtheta1-sin(theta3_t0-theta2_t0)*dtheta1_t0+L_Integralsum[2][29];
    L_Integralsum[2][30] += m_steptime*((l1-l2*cos(theta2))*pow(dtheta1,2)*cos(theta3-theta2)-l2*cos(theta3)*(pow(dtheta2,2)-dtheta2*dtheta3)\
                                                    +ax*sin(theta3-theta2)-(ay*sin(theta1)-az*cos(theta1))*cos(theta3-theta2));
    L[index+2][30] = -l2*sin(theta3)*dtheta2+l2*sin(theta3_t0)*dtheta2_t0+L_Integralsum[2][30];
    L[index+2][31] = 0;
    L_Integralsum[2][32] += m_steptime*((l1-l2*cos(theta2))*pow(dtheta1,2)*sin(theta3-theta2)-l2*sin(theta3)*(pow(dtheta2,2)-dtheta2*dtheta3)\
                                                    -ax*cos(theta3-theta2)-(ay*sin(theta1)-az*cos(theta1))*sin(theta3-theta2));
    L[index+2][32] = l2*cos(theta3)*dtheta2-l2*cos(theta3_t0)*dtheta2_t0+L_Integralsum[2][32];
    L[index+2][33] = 0;
    L_Integralsum[2][34] += m_steptime*dtheta3;
    L[index+2][34] = L_Integralsum[2][34];
    L_Integralsum[2][35] += m_steptime*sgn(dtheta3);
    L[index+2][35] = L_Integralsum[2][35];

    L_Integralsum[2][36] += m_steptime*(-0.5*pow(dtheta1,2)*sin(2*(theta2+theta4-theta3)));
    L[index+2][36] = L_Integralsum[2][36];
    L[index+2][37] = dtheta3-dtheta4-dtheta2-dtheta3_t0+dtheta4_t0+dtheta2_t0;
    L[index+2][38] = -L_Integralsum[2][36];
    L_Integralsum[2][39] += m_steptime*sin(theta2+theta4-theta3)*(-dtheta1*dtheta2+dtheta1*dtheta3-dtheta1*dtheta4);
    L[index+2][39] = -cos(theta2+theta4-theta3)*dtheta1+cos(theta2_t0+theta4_t0-theta3_t0)*dtheta1_t0+L_Integralsum[2][39];
    L_Integralsum[2][40] += m_steptime*(-cos(2*(theta2+theta4-theta3))*pow(dtheta1,2));
    L[index+2][40] = L_Integralsum[2][40];
    L_Integralsum[2][41] += m_steptime*cos(theta2+theta4-theta3)*(-dtheta1*dtheta2+dtheta1*dtheta3-dtheta1*dtheta4);
    L[index+2][41] = sin(theta2+theta4-theta3)*dtheta1-sin(theta2_t0+theta4_t0-theta3_t0)*dtheta1_t0+L_Integralsum[2][41];
    L_Integralsum[2][42] += m_steptime*(-((l1-l2*cos(theta2))*cos(theta2+theta4-theta3)+l3*cos(2*theta2+theta4-2*theta3))*pow(dtheta1,2)+l2*cos(theta3-theta4)*pow(dtheta2,2)\
                                        -l2*cos(theta3-theta4)*(dtheta2*dtheta3-dtheta2*dtheta4)+ax*sin(theta2+theta4-theta3)+(ay*sin(theta1)-az*cos(theta1))*cos(theta2+theta4-theta3));
    L[index+2][42] = l3*sin(theta4)*(2*dtheta2-2*dtheta3+dtheta4)-l3*sin(theta4_t0)*(2*dtheta2_t0-2*dtheta3_t0+dtheta4_t0)\
                                    +l2*sin(theta3-theta4)*dtheta2-l2*sin(theta3_t0-theta4_t0)*dtheta2_t0+L_Integralsum[2][42];
    L_Integralsum[2][43] += m_steptime*l3*cos(theta3-theta2)*(-dtheta1*dtheta2+dtheta1*dtheta3);
    L[index+2][43] = l3*sin(theta2-theta3)*dtheta1-l3*sin(theta2_t0-theta3_t0)*dtheta1_t0+L_Integralsum[2][43];
    L_Integralsum[2][44] += m_steptime*(((l1-l2*cos(theta2))*sin(theta2+theta4-theta3)+l3*sin(2*theta2+theta4-2*theta3))*pow(dtheta1,2)+l2*sin(theta3-theta4)*pow(dtheta2,2)\
                                        -l2*sin(theta3-theta4)*(dtheta2*dtheta3-dtheta2*dtheta4)+ax*cos(theta2+theta4-theta3)+(-ay*sin(theta1)+az*cos(theta1))*sin(theta2+theta4-theta3));
    L[index+2][44] = l3*cos(theta4)*(2*dtheta2-2*dtheta3+dtheta4)-l3*cos(theta4_t0)*(2*dtheta2_t0-2*dtheta3_t0+dtheta4_t0)\
                                    -l2*cos(theta3-theta4)*dtheta2+l2*cos(theta3_t0-theta4_t0)*dtheta2_t0+L_Integralsum[2][44];
    L_Integralsum[2][45] += m_steptime*(-l3*sin(theta2-theta3)*pow(dtheta1,2)*(l1-l2*cos(theta2)+l3*cos(theta2-theta3))-l2*l3*sin(theta3)*pow(dtheta2,2)+l2*l3*sin(theta3)*dtheta2*dtheta3\
                                                    -ax*l3*cos(theta2-theta3)+ay*l3*sin(theta1)*sin(theta2-theta3)-az*l3*cos(theta1)*sin(theta2-theta3));
    L[index+2][45] = pow(l3,2)*(dtheta3-dtheta2-dtheta3_t0+dtheta2_t0)+l2*l3*(cos(theta3)*dtheta2-cos(theta3_t0)*dtheta2_t0)+L_Integralsum[2][45];
    L[index+2][46] = 0;
    L[index+2][47] = 0;
         //fourth row
    for(int i = 0; i < 36; i++)
    {
        L[index+3][i] = 0;
    }
    L_Integralsum[3][36] += m_steptime*0.5*pow(dtheta1,2)*sin(2*(theta2+theta4-theta3));
    L[index+3][36] = L_Integralsum[3][36];
    L[index+3][37] = dtheta4+dtheta2-dtheta3-dtheta4_t0-dtheta2_t0+dtheta3_t0;
    L[index+3][38] = -L_Integralsum[3][36];
    L_Integralsum[3][39] += m_steptime*sin(theta2+theta4-theta3)*(dtheta1*dtheta2-dtheta1*dtheta3+dtheta1*dtheta4);
    L[index+3][39] = cos(theta2+theta4-theta3)*dtheta1-cos(theta2_t0+theta4_t0-theta3_t0)*dtheta1_t0+L_Integralsum[3][39];
    L_Integralsum[3][40] += m_steptime*cos(2*(theta2+theta4-theta3))*pow(dtheta1,2);
    L[index+3][40] = L_Integralsum[3][40];
    L_Integralsum[3][41] += m_steptime*cos(theta2+theta4-theta3)*(dtheta1*dtheta2-dtheta1*dtheta3+dtheta1*dtheta4);
    L[index+3][41] = -sin(theta2+theta4-theta3)*dtheta1+sin(theta2_t0+theta4_t0-theta3_t0)*dtheta1_t0+L_Integralsum[3][41];
    L_Integralsum[3][42] += m_steptime*(pow(dtheta1,2)*cos(theta2+theta4-theta3)*(l1-l2*cos(theta2)+l3*cos(theta2-theta3))-(l2*cos(theta3-theta4)-l3*cos(theta4))*(pow(dtheta2,2)-dtheta2*dtheta3+dtheta2*dtheta4)\
                                        -l3*cos(theta4)*(dtheta2*dtheta3-pow(dtheta3,2)+dtheta3*dtheta4)-ax*sin(theta2+theta4-theta3)+(-ay*sin(theta1)+az*cos(theta1))*cos(theta2+theta4-theta3));
    L[index+3][42] = l3*sin(theta4)*(dtheta3-dtheta2)-l3*sin(theta4_t0)*(dtheta3_t0-dtheta2_t0)-l2*sin(theta3-theta4)*dtheta2+l2*sin(theta3_t0-theta4_t0)*dtheta2_t0+L_Integralsum[3][42];
    L[index+3][43] = 0;
    L_Integralsum[3][44] += m_steptime*(-pow(dtheta1,2)*sin(theta2+theta4-theta3)*(l1-l2*cos(theta2)+l3*cos(theta2-theta3))-(l2*sin(theta3-theta4)+l3*sin(theta4))*(pow(dtheta2,2)-dtheta2*dtheta3+dtheta2*dtheta4)\
                                        +l3*sin(theta4)*(dtheta2*dtheta3-pow(dtheta3,2)+dtheta3*dtheta4)-ax*cos(theta2+theta4-theta3)+(ay*sin(theta1)-az*cos(theta1))*sin(theta2+theta4-theta3));
    L[index+3][44] = l3*cos(theta4)*(dtheta3-dtheta2)-l3*cos(theta4_t0)*(dtheta3_t0-dtheta2_t0)+l2*cos(theta3-theta4)*dtheta2-l2*cos(theta3_t0-theta4_t0)*dtheta2_t0+L_Integralsum[3][44];
    L[index+3][45] = 0;
    L_Integralsum[3][46] += m_steptime*dtheta4;
    L[index+3][46] = L_Integralsum[3][46];
    L_Integralsum[3][47] += m_steptime*sgn(dtheta4);
    L[index+3][47] = L_Integralsum[3][47];
    //If W matrix has been full, the process of sampling data will be finished.Change the robot state to STOP, then transport W matrix to matlab communication program.
    if(index == ROW-4)
    {
        index = 0;
        t1=0;
        t2=0;
        t3=0;
        t4=0;
        memset(L_Integralsum, 0, sizeof(L_Integralsum));
        memset(torque_integral, 0, sizeof(torque_integral));
        robot_state = QuadRobot::STOP;
        for(int r = 0; r < ROW; r++)
        {
            matlabIdentMsg.add_torque(torqueTrans[r]);
            for(int c = 0; c < COLUMN; c++)
            {
                matlabIdentMsg.add_matelement(L[r][c]);
            }
        }
        pubMatlab.Publish(matlabIdentMsg);
    }
    SetJointsTorque(1,1,  appliedTorque[0] );
    SetJointsTorque(1,2,  appliedTorque[1] );
    SetJointsTorque(1,3,  appliedTorque[2] );
    SetJointsTorque(1,4,  appliedTorque[3] );

    index += 4;
    ReleaseLock();
}

