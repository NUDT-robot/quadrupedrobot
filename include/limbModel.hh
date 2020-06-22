/*
 * Author : ChangXu
 * LimbModel class models the limb composed of links . Number of Links greater than four hasn't been supported.
 * Dimension : SI
 * Associated document :  Model Construction for Quadrupled Robot .doc
*/
#ifndef LIMBMODEL_HH_
#define LIMBMODEL_HH_

#include <iostream>
#include <Eigen/Dense>
#include "ds/tree.h"

//using namespace Eigen;

class QuadRobot;

/*Invert kinetics solutions are stored to a class InvertAnswer*/
class InverAnswer : public Tree<double>
{
    //endNode_V store the deepest nodes' pointers in order to get answer, this vector size equals quantities of the answer. Answer order in the tree layer: Root->theta1->theta3->theta2
    std::vector<TreeNode<double>*> endNode_V;
private:
    void FindAnswer( TreeNode<double>* parent)
    {
        ulong size = parent->GetChildren().size();
        if( size == 0 && parent->GetNodeType() == TreeNode<double>::ANSWER)
        {
                endNode_V.push_back(parent);
        }
        else
        {
            for(ulong i = 0; i < size; i++)
            {
                FindAnswer(parent->GetChildren()[i]);
            }
        }

    }
public:
    InverAnswer(){  }
    void ClearInvAns()
    {
        endNode_V.clear();
        ClearTree();
    }
    void GetAnswer(std::vector<Eigen::Vector3d>& answer, const Eigen::Vector3d& position, double length[3], double tol = 0.0001)
    {
        double px = position.x(), py = position.x(), pz = position.z();
        double l1 = length[0], l2 = length[1], l3 = length[2];
        double justinter = 0;
        Eigen::Vector3d verifyposition;
        TreeNode<double>* pRoot = GetRoot();
        Eigen::Vector3d answerbuffer;
        answer.clear();
        FindAnswer(pRoot);
        if(endNode_V.size() == 0)
            return;
         //Store answers. The order of element in Vector4d is X=theta1, Y=theta2,Z= theta3
        for(ulong i = 0; i < endNode_V.size(); i++)
        {
            answerbuffer.y() = endNode_V[i]->GetData();
            answerbuffer.z() = endNode_V[i]->GetParent()->GetData();
            answerbuffer.x() = endNode_V[i]->GetParent()->GetParent()->GetData();
            /*
            * We must discard some solutions, because in order to solving invert kinetics we get new equations that are not equivalent to that of original !!!
            *  For computation speed, calculating P point explicitly instead of calling function LimbModel::CalMainPointPos.
            */
            verifyposition.x() = l2*sin(answerbuffer.y()) - l3*sin(answerbuffer.y()-answerbuffer.z());
            justinter = -l2*cos(answerbuffer.y()) + l3*cos(answerbuffer.y()-answerbuffer.z());
            verifyposition.y() = -(l1+justinter)*sin(answerbuffer.x());
            verifyposition.z() = (l1+justinter)*cos(answerbuffer.x());
            if(!(position-verifyposition).isZero(tol))
                continue;
            answer.push_back(answerbuffer);
        }
        return;
    }
};

/*LinkModel is a component for LimbModel*/
class LinkModel
{
    double m_length = 0;      // length of rod
    double m_COP = 0;          // mass center of position, indicating the length between the rotation center point and the COP
    double m_mass = 0;        // mass
    Eigen::Matrix3d m_inertial;      // Inertial Tensor Matrix
public:
    LinkModel(double length = 0.1, double cop = 0.05, double mass = 1, Eigen::Matrix3d inertial = Eigen::Matrix3d::Identity() )
    {
        m_length = length;
        m_COP = cop;
        m_mass = mass;
        m_inertial = inertial;
    }
    void ModifyLength(double length){ m_length = length;}
    void ModifyCOP(double cop){m_COP = cop;}
    void ModifyMass(double mass){m_mass = mass;}
    void ModifyInertial(Eigen::Matrix3d inertial ){m_inertial = inertial;}
    double GetLength(){return  m_length;}
    double GetCOP(){return m_COP ;}
    double GetMass(){return m_mass ;}
    Eigen::Matrix3d GetInertial( ){return m_inertial ;}
};

/*LimbModel is composed of LinkModels
*LimbModel is considered to be able to govern angle between rods , angle velocity, main points position and velocities in rods
*/
class LimbModel
{
#define PI 3.1415926
#define PRECISION 0.0000001
#define FILTERSIZE 20
    //base parameters size
#define BASEPARASIZE 36

public:
    //This enum provide two limb states, swing or support.
    enum LIMB_STATE
    {
        SWING = 0,
        SUPPORT = 1
    };
private:
    bool m_isvalid = 0;       //indicates whether the model initialized correctly
    bool m_dataValid = false; //indicates whether the joint position data or other data is valid.
    u_char m_numOfLink = 3;    //number of link to compose of a Limb
    LinkModel* m_prod[3] = {0,0,0};   //support  four links at most
    double m_steptime = 0.001;    //a step time for calculating velocity and accelerated velocity
    double m_factorTime = 1000;  //For calculation speed, save this multiplication fator time
    LIMB_STATE limb_state = SUPPORT;    //Indicates whether the limb state is swing or support.
private:
/*Kinematics parameters*/
    //member for origin of O1XYZ frame
    Eigen::Vector3d m_originPos;     //Origin position of O1XYZ frame in OXYZ frame, Default value 0
    Eigen::Vector3d m_originVel;     //Origin velocity of O1XYZ frame in OXYZ frame, Default value 0
    Eigen::Vector3d m_originAccVel;     //Origin Accelerated velocity of O1XYZ frame in OXYZ frame, Default value 0
    //member for limb kinetics in O1XYZ frame
    double m_thetaPos[3] = {0,PI*5/4,3*PI/2};     // default value 0, 225, 90, 90 degree for theta1, theta2, theta3. When use 200Hz, we get joint angle precison 0.001 rad
    double m_thetaVel[3];     //theta Velocity array represented for angle velocities of theta1-theta3. Default value 0
    double m_thetaVelFilterBuffer[3][FILTERSIZE];    //Because difference error, theta angle velocity need average filter buffer
    double m_thetaAccVel[3];     //theta Accelerated Velocity array represented for angle velocities of theta1-theta4. Default value 0
    double m_thetaAccVelFilterBuffer[3][FILTERSIZE];   //Because difference error, theta Accelerated Velocity need average filter buffer
    Eigen::Vector3d m_pointPos[3];       //Position of point A,B,C. Default value 0 is invalid.
    Eigen::Vector3d m_pointVel[3];       //velocity array represented for velocities of point A,B,C. Default value 0
    Eigen::Vector3d m_pointAccVel[3];       //accelerated velocity array represented for velocities of point A,B,C. Default value 0
    InverAnswer inv_ans_tree;       //Inverse kinetics solution tree
    //Invert kinetics answers, vector size equals quantity of the answer
    std::vector<Eigen::Vector3d> m_invans;
    //Coordinate transform, from B frame to Body frame. B frame is attached to the last rod.
    Eigen::Matrix3d R_BtoBody;
/*Dynamics parameters*/
    //This parameter is filled by QuadRobot class
    Eigen::Vector3d  generalizedJointWrench;
    //Base parameters is attained by identification
    double baseParameters[3][BASEPARASIZE];
    //Force/Torque Jacobi Matrix
    Eigen::Matrix3d Jac;
    //P point Contact force, this force is calculated by dynamics formulation
    Eigen::Vector3d contactforce_P;
#define CFFILTERSIZE 50     //contact force filter buffer size
        //index_CFFB indicates 'contactforce_FilterBuffer' index and contactforce_PFilterBuffer use for average filtering
    ulong index_CFFB = 0;
    Eigen::Vector3d contactforce_FilterBuffer[CFFILTERSIZE];
        //contact force low pass filter, this filter should be changed with detection frequency, 0.965 is approximately 0.08s detection delay with threshold 130N
    double filter_cf = 0.965;
    //Generalized contact force
#define GFFILTERSIZE 1  //generalized force filter buffer size
    double generalized_cf[3] = {0,0,0};
    double generalized_cfint[3] = {0,0,0};
    ulong index_GFFB = 0;
    double generalized_cfFilterBuffer[3][GFFILTERSIZE];
        //generalized contact force low pass filter
    double filter_gf = 0;
        //dynamics formulation without accelerated velocity parameters
     double theta1_t0,theta2_t0,theta3_t0;
     double dtheta1_t0,dtheta2_t0,dtheta3_t0;
     double L_Integralsum[3][36];
     double torque_integral[3];
     double gcf_integral[3] = {0,0,0};
        //current evaluation about generalized force
     double gcf_eval[3]={0,0,0};
     ulong simstep = 0;
     bool resetIntegral = 1;
private:
    inline void SetValid(bool valid){m_dataValid = valid;}
    inline void SetOriginPos(Eigen::Vector3d& pos)   { m_originPos = pos;}
    inline void SetOriginVel(Eigen::Vector3d& vel)   { m_originVel = vel;}
    inline void SetOriginAccVel(Eigen::Vector3d& accvel)   { m_originAccVel = accvel;}
    //This function should be called by the class which include LimbModel class to implement dynamics calculating.
    inline void SetGeneralizedForce(Eigen::Vector3d gf) { generalizedJointWrench = gf; }
    inline Eigen::Vector3d GetGeneralizedForce() { return generalizedJointWrench; }
public:
    LimbModel(u_char numsOfLink = 3);
    ~ LimbModel(){for(int i = 0; i < m_numOfLink; i++) delete m_prod[i];}
public:
//property functions
    bool ModifyLinkParam(u_char rodindex,  double length, double cop, double mass, Eigen::Matrix3d inertial );
    void SetSteptime(double steptime) {assert(steptime > 0); m_steptime = steptime;m_factorTime= 1.0/m_steptime;}
    LIMB_STATE GetState(){ return limb_state; }
//Get kinetic functions
    u_char GetNumsOfLink()  {return m_numOfLink; }
    LinkModel** GetLink(){ return m_prod; }
    Eigen::Vector3d GetOriginPos()   {return m_originPos;}
    Eigen::Vector3d GetOriginVel()   {return m_originVel;}
    Eigen::Vector3d GetOriginAccVel()   {return m_originAccVel;}
    //if wrong index set, return 1e10.
    double GetThetaPos(u_char index)
        {
            if(!m_dataValid || index == 0 || index >3)
                {std::cout <<"data is not valid or bad index in function GetThetaPos"<<std::endl; return 1e10;}
            return m_thetaPos[index-1];
        }
    double GetThetaVel(u_char index)
        {if(!m_dataValid || index == 0 || index >3){std::cout <<"data is not valid or bad index in function GetThetaVel"<<std::endl; return 1e10;} return m_thetaVel[index-1];}
    double GetThetaAccVel(u_char index)
        {if(!m_dataValid || index == 0 || index >3){std::cout <<"data is not valid or bad index in function GetThetaAccVel, m_dataValid = "<< m_dataValid <<" , index = "<< index <<std::endl; return 1e10;} return m_thetaAccVel[index-1];}
//
    inline Eigen::Vector3d GetPointPos(u_char index)   {return m_pointPos[index-1];}
    inline Eigen::Vector3d GetPointVel(u_char index)   {return m_pointVel[index-1];}
    inline Eigen::Vector3d GetPointAccVel(u_char index)   {return m_pointAccVel[index-1];}
    inline Eigen::Matrix3d GetCoordTransformMatrix() {return R_BtoBody;}
//Set kinetic functions
    void SetThetaPos(u_char index, double pos)
        { if(index == 0 || index >3){std::cout <<"bad index in function SetThetaPos"<<std::endl; return;} m_thetaPos[index-1] = pos;}
    void SetThetaVel(u_char index, double vel)
        {if(index == 0 || index >3){std::cout <<"bad index in function SetThetaVel"<<std::endl; return;} m_thetaVel[index-1] = vel;}
    void SetThetaAccVel(u_char index, double accvel)
        {if(index == 0 || index >3) {std::cout <<"bad index in function SetThetaAccVel"<<std::endl; return;} m_thetaAccVel[index-1] = accvel;}
/*
 * Main Points' Postions ,velocities ,accelerated velocities should be calculated by theta ,so setting points' parameters should be considered carefully .
 * void SetPointPos(u_char index, Vector3d& pos)   { }
    void SetPointVel(u_char index, Vector3d& vel)   { }
    void SetPointAccVel(u_char index, Vector3d& accvel)   { }
*/
    //Step function calculate A,B,C point position, C point velocity, theta angle velocity and angle accelerated velocity in one step time "m_steptime". Then perform  dynamics calculation process.
    void Step();
//functions only for internal implementation
private:
    void SetupModel(void);
    //only called in CalInvertKinetics function
    bool InvCalTheta2(double cx, double d, double e, TreeNode<double>* ptheta3);
    bool InvCalTheta1and3Then2(const Eigen::Vector3d& position, TreeNode<double>* proot);
    //In this function, it calculate coordinate transform matrix
    void CoordinateTransform();
public:
    //calculate point position from theta angle
    void CalMainPointPos();
    //This function return A,B,C point position. Index range from 1 to 3
    Eigen::Vector3d GetMainPointPos(int index) {if(index <= 0 || index >3) {std::cout <<"bad index in function SetThetaAccVel"<<std::endl; return Eigen::Vector3d(0,0,0);}  return m_pointPos[index-1];}
    //Given postition, calculate invert kinetics and the result is stroed to member m_invans
    //If success, return true and the size of m_invans is not zero, otherwise return  false and m_invans is empty.
    bool CalInvertKinetics(const Eigen::Vector3d& postition);
    const  std::vector<Eigen::Vector3d>& GetInvAnswer() { return m_invans;}
    /*Dynamics calculating*/
    //This function calculating foot contact force using dynamics formulation
    void DynamicsCalFootContactForce();
    void DynamicsCalFootContactForceWithoutAcc();
    void EstimateFootState();
    inline Eigen::Vector3d GetContactForce() { return contactforce_P; }
    LIMB_STATE GetFootState();
friend class QuadRobot;
};

#endif
