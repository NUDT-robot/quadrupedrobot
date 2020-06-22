#include "include/limbModel.hh"
#include "include/ds/traprocgen.hh"
#include <cmath>

using namespace std;
using namespace Eigen;

LimbModel::LimbModel(u_char numOfLink)
{
    if(numOfLink > 3 || numOfLink == 0 )
    {
        cout << "Number of Links greater than 3 or equalling zero is forbidden !" << endl ;
        return;
    }
    m_numOfLink = numOfLink;
    memset(baseParameters, 0, sizeof(baseParameters));

    //Set base paramters here based on identification
/*    baseParameters[2][24] =  0.0733; baseParameters[2][25] = 0.5852; baseParameters[2][27] =0.0353; baseParameters[2][28] =  0.0342 ; baseParameters[2][29] = 0.0252;
    baseParameters[2][30] = 0.0306; baseParameters[2][32] = 0.5132; baseParameters[2][34] = 0.9052; baseParameters[2][35] =  -0.0158;

    baseParameters[1][12] = 0.2215; baseParameters[1][13] = 1.0774; baseParameters[1][15] = 0.1145; baseParameters[1][16] = 0.0510; baseParameters[1][17] = -0.0034;
    baseParameters[1][18] = 0.0023; baseParameters[1][20] = 1.4841; baseParameters[1][22] = 0.9111; baseParameters[1][23] = -0.0863;
    baseParameters[1][24] = baseParameters[2][24]; baseParameters[1][25] = baseParameters[2][25]; baseParameters[1][27] = baseParameters[2][27];
    baseParameters[1][28] = baseParameters[2][28]; baseParameters[1][29] =baseParameters[2][29]; baseParameters[1][30] = baseParameters[2][30];
    baseParameters[1][32] = baseParameters[2][32];

    baseParameters[0][0] = 1.4090; baseParameters[0][7] = 0.0135; baseParameters[0][8] = 0.5664; baseParameters[0][10] = 0.3358; baseParameters[0][11] = 0.1080;
    baseParameters[0][12] = baseParameters[1][12]; baseParameters[0][15] = baseParameters[1][15]; baseParameters[0][16] = baseParameters[1][16];
    baseParameters[0][17] = baseParameters[1][17]; baseParameters[0][18] = baseParameters[1][18];
    baseParameters[0][20] =  baseParameters[1][20];
    baseParameters[0][24] =  baseParameters[1][24]; baseParameters[0][27] = baseParameters[1][27]; baseParameters[0][28] =baseParameters[1][28];
    baseParameters[0][29] = baseParameters[1][29]; baseParameters[0][30] = baseParameters[1][30]; baseParameters[0][32] = baseParameters[1][32];*/
    baseParameters[2][24] = 0.0646; baseParameters[2][25] = 0.6019; baseParameters[2][27] =-0.0133; baseParameters[2][28] = -0.0210 ; baseParameters[2][29] =  0.0234;
        baseParameters[2][30] = 0.0179; baseParameters[2][32] = 0.5406; baseParameters[2][34] = 0.8616; baseParameters[2][35] = 0.0638;

        baseParameters[1][12] = 0.3103; baseParameters[1][13] = 1.0707; baseParameters[1][15] = 0.0638; baseParameters[1][16] =  0.0144; baseParameters[1][17] =  -0.0088;
        baseParameters[1][18] =  -0.0041; baseParameters[1][20] = 1.4771; baseParameters[1][22] = 1.0145; baseParameters[1][23] =  -0.1176;
        baseParameters[1][24] = baseParameters[2][24]; baseParameters[1][25] = baseParameters[2][25]; baseParameters[1][27] = baseParameters[2][27];
        baseParameters[1][28] = baseParameters[2][28]; baseParameters[1][29] =baseParameters[2][29]; baseParameters[1][30] = baseParameters[2][30];
        baseParameters[1][32] = baseParameters[2][32];

        baseParameters[0][0] = 1.3061; baseParameters[0][7] = 0.0122; baseParameters[0][8] =  0.4739;  baseParameters[0][10] = 0.5925; baseParameters[0][11] = 0.1180;
        baseParameters[0][12] = baseParameters[1][12]; baseParameters[0][15] = baseParameters[1][15]; baseParameters[0][16] = baseParameters[1][16];
        baseParameters[0][17] = baseParameters[1][17]; baseParameters[0][18] = baseParameters[1][18];
        baseParameters[0][20] =  baseParameters[1][20];
        baseParameters[0][24] =  baseParameters[1][24]; baseParameters[0][27] = baseParameters[1][27]; baseParameters[0][28] =baseParameters[1][28];
        baseParameters[0][29] = baseParameters[1][29]; baseParameters[0][30] = baseParameters[1][30]; baseParameters[0][32] = baseParameters[1][32];

    SetupModel();
    m_isvalid = 1;
}

/*This function setup the model for a limb based on member variable "m_numOfLink"   */
void LimbModel::SetupModel(void)
{
     assert(m_numOfLink <= 3);
     for(int i = 0; i < m_numOfLink; i++)
     {
        m_prod[i]  = new LinkModel;
     }
     memset(m_thetaVelFilterBuffer, 0, sizeof(m_thetaVelFilterBuffer));
     memset(m_thetaAccVelFilterBuffer, 0, sizeof(m_thetaAccVelFilterBuffer));
}

/*This function should not be called before  the model setup for a limb based on member variable "m_numOfLink"
*Return true if success, otherwise false .The index value must be between 1 and 3
*/
bool LimbModel::ModifyLinkParam(u_char index,  double length, double cop, double mass, Matrix3d inertial )
{
    if(!m_isvalid)
    {
        cout << "Error in function ModifyLinkParam : LimbModel has not been initialized !" << endl ;
        return false;
    }
    if( (index == 0 &&  index > m_numOfLink) || length < 0 || cop < 0 || mass < 0 || inertial.determinant() < 0)
    {
        cout << "Error in function ModifyLinkParam : check values of input parameters " << endl ;
        return false;
    }
    m_prod[index - 1]->ModifyLength(length);
    m_prod[index - 1]->ModifyCOP(cop);
    m_prod[index - 1]->ModifyMass(mass);
    m_prod[index - 1]->ModifyInertial(inertial);
    return true;
}

void LimbModel::Step()
{
    /*Calculating A,B,C points position*/
    CalMainPointPos();
    /*Calculating coordinate transformation, all coordinates transformation should be implemented here. */
    CoordinateTransform();
    /*Calculating dynamics process*/
    DynamicsCalFootContactForceWithoutAcc();
    /*Update foot state*/
    EstimateFootState();
}

void LimbModel::CalMainPointPos()
{
    //Point A
    double rod1length = m_prod[0]->GetLength();
    double Ax = 0;
    double Ay = (-1)*rod1length*sin(m_thetaPos[0]);
    double Az = rod1length*cos(m_thetaPos[0]);
    m_pointPos[0].x() = Ax;
    m_pointPos[0].y() = Ay;
    m_pointPos[0].z() = Az;
    //Point B
    double rod2length = m_prod[1]->GetLength();
    double Bx = Ax + rod2length * sin(m_thetaPos[1]);
    double By = Ay + rod2length * sin(m_thetaPos[0]) * cos(m_thetaPos[1]);
    double Bz = Az - rod2length * cos(m_thetaPos[0]) * cos(m_thetaPos[1]);
    m_pointPos[1].x() = Bx;
    m_pointPos[1].y() = By;
    m_pointPos[1].z() = Bz;
    //Point C, i.e., Foot point
    double rod3length = m_prod[2]->GetLength();
    double Cx = Bx - rod3length * sin(m_thetaPos[1] - m_thetaPos[2]);
    double Cy = By - rod3length * sin(m_thetaPos[0]) * cos(m_thetaPos[1] - m_thetaPos[2]);
    double Cz = Bz + rod3length * cos(m_thetaPos[0]) * cos(m_thetaPos[1] - m_thetaPos[2]);
    m_pointPos[2].x() = Cx;
    m_pointPos[2].y() = Cy;
    m_pointPos[2].z() = Cz;
}

bool LimbModel::InvCalTheta1and3Then2(const Vector3d& position, TreeNode<double>* proot )
{
    double cx =position.x() , cy =position.y() , cz = position.z();
    double l1 = m_prod[0]->GetLength(), l2 = m_prod[1]->GetLength(), l3 = m_prod[2]->GetLength();
   //theta1, its range is from -PI/2 to PI/2
   //calculate theta3 parameters, its range is from PI to 2*PI
   double theta1 = 0, c = 0, d = 0, e = 0;
   TreeNode<double>* ptheta1 = NULL;
   TreeNode<double>* ptheta3 = NULL;
//calculate theta3
   if(cy < PRECISION && cy >-PRECISION && cz < PRECISION && cz >-PRECISION )        //In case of all zero, theta1 is arbitrary. Here set theta1 its current value for saving energy
   {
       theta1 = GetThetaPos(1);
       c = ( pow(l1, 2) + pow(cx, 2) - pow(l2, 2) - pow(l3, 2) ) / (-2*l2*l3);
   }
   else if(cy > PRECISION && cy <-PRECISION)                                                                            //cy is not zero case
   {
       //Insuring using atan2 to calculate theta1.
      theta1 = atan2(-cy,cz);
      if(theta1 > PI/2+PRECISION  || theta1 < -PI/2-PRECISION)
          return false;
      c = ( pow(cy/sin(theta1)+l1, 2) + pow(cx, 2) - pow(l2, 2) - pow(l3, 2) ) / (-2*l2*l3);
   }
   else
   {
        //Insuring using atan2 to calculate theta1, because a=lsinx1,b=lcosx2, l > 0 here.
       theta1 = atan2(-cy,cz);
       if(theta1 > PI/2+PRECISION  || theta1 < -PI/2-PRECISION)
           return false;
       c = (pow(cz/cos(theta1)-l1, 2) + pow(cx, 2) - pow(l2, 2) - pow(l3, 2) ) / (-2*l2*l3);
   }
   if(c > 1|| c < -1)
       return false;
   else
   {
       //solution to theta1
       ptheta1 = inv_ans_tree.AddNode(proot);
       ptheta1->SetData(theta1);
       //Value of theta3 is from PI to 2*PI, so only -acos() can get the result what we want.
       double theta3 = 0;
       theta3 = -acos(c)+2*PI;
       ptheta3 = inv_ans_tree.AddNode(ptheta1);
       ptheta3->SetData(theta3);

       d= l3*sin(theta3);
       e = l2-l3*cos(theta3);

    }

    return InvCalTheta2(cx, d, e, ptheta3);
}

bool LimbModel::InvCalTheta2(double cx, double d, double e, TreeNode<double>* ptheta3)
{
    //calculate theta2, use d,e
        double f = cx/( sqrt(pow(d,2)+pow(e,2)) );
        double fai_2 = atan2(d, e);
        if(f > 1|| f < -1)
        {
            return false;
        }
        else
        {
            //first solution use f. The value of  theta2 is from PI/2 to 3*PI/2.
            double theta2_1 = 0;
            int solnum1 = 0;         // Indicates solution number.
            theta2_1 = asin(f) - fai_2;
            if(theta2_1 >= PI/2 && theta2_1 <= 3*PI/2)
            {
                solnum1++;
            }
            else if(theta2_1 >= -3*PI/2 && theta2_1 <= -PI/2)
            {
                theta2_1 += 2*PI;
                solnum1++;
            }

            if(solnum1)
            {
                TreeNode<double>* ptheta2_1 = inv_ans_tree.AddNode(ptheta3);
                ptheta2_1->SetType(ptheta2_1->ANSWER);
                ptheta2_1->SetData(theta2_1);
            }
            //second solution use f
            double theta2_2 = 0;
            int solnum2 = 0;
            theta2_2 = PI - asin(f) - fai_2;
            if(theta2_2 >= PI/2 && theta2_2 <= 3*PI/2)
            {
                solnum2++;
            }
            else if(theta2_2 >= -3*PI/2 && theta2_2 <= -PI/2)
            {
                theta2_2 += 2*PI;
                solnum2++;
            }

            if(solnum2)
            {
                TreeNode<double>* ptheta2_2 = inv_ans_tree.AddNode(ptheta3);
                ptheta2_2->SetType(ptheta2_2->ANSWER);
                ptheta2_2->SetData(theta2_2);
            }
            //if no solution to theta2
            if(solnum1 == 0 && solnum2 == 0)
                return false;
        }
        return true;
}

bool LimbModel::CalInvertKinetics(const Vector3d& position)
{
    double length[3];
    for(int i = 0; i < 3; i++)
    {
        length[i] = m_prod[i]->GetLength();
    }
    //Clear old answer tree for new answer
    inv_ans_tree.ClearInvAns();
    m_invans.clear();

    TreeNode<double>* proot = inv_ans_tree.GetRoot();
    //if py and pz equal 0;
    if(!InvCalTheta1and3Then2(position, proot))
    {
        inv_ans_tree.ClearInvAns();
        return false;
    }
    inv_ans_tree.GetAnswer(m_invans, position, length , PRECISION);
    if(m_invans.size() == 0)
        return false;
    return true;
}

void LimbModel::CoordinateTransform()
{
    double theta1 = GetThetaPos(1), theta2 = GetThetaPos(2), theta3 = GetThetaPos(3);
    R_BtoBody <<           cos(-theta2+theta3),                                0,                         sin(-theta2+theta3),
                               sin(theta1)*sin(-theta2+theta3),          cos(theta1),        -sin(theta1)*cos(-theta2+theta3),
                               -cos(theta1)*sin(-theta2+theta3),         sin(theta1),        cos(theta1)*cos(-theta2+theta3);
}

void LimbModel::DynamicsCalFootContactForce()
{
    /*This function using dynamics formulation to calculate generalized contact force, then using the 1,2,3 generalized coordinates to get contact force */
    double W[3][BASEPARASIZE];
    double l1 = GetLink()[0]->GetLength(), l2 = GetLink()[1]->GetLength(), l3 = GetLink()[2]->GetLength();
        //Gravity ccelerated velocity in x,y,z axis.
    double ax = -m_originAccVel.x();
    double ay = -m_originAccVel.y();
    double az = -m_originAccVel.z();
    //Dynamics equation : Mq..+Cq.+G = tau + g_ext . The result is stored in de_lf[3] (dynamics equation left side)
    double de_lfside[3] = {0,0,0};
        //Fill the W matrix based on the joints' trajectories.
    double theta1 = GetThetaPos(1), theta2 = GetThetaPos(2), theta3 = GetThetaPos(3);
    double dtheta1 = GetThetaVel(1), dtheta2 = GetThetaVel(2), dtheta3 = GetThetaVel(3);
    double ddtheta1 = GetThetaAccVel(1), ddtheta2 = GetThetaAccVel(2), ddtheta3 = GetThetaAccVel(3);
    //first generalized contact force
    for(int i = 1; i < 7; i++)
    {
         W[0][i] = 0;
    }
    W[0][0] = ddtheta1;
    W[0][7] = -az*cos(theta1);
    W[0][8] = az*sin(theta1);
    W[0][9] = 0;
    W[0][10] = dtheta1;
    W[0][11] = MyMathFunc::sgn(dtheta1);

    W[0][12] = ddtheta1*pow(cos(theta2),2)-dtheta1*dtheta2*sin(2*theta2);
    W[0][13] = 0;
    W[0][14] = ddtheta1*pow(sin(theta2),2)+dtheta1*dtheta2*sin(2*theta2);
    W[0][15] = ddtheta2*cos(theta2)-pow(dtheta2,2)*sin(theta2);
    W[0][16] = -ddtheta1*sin(2*theta2)-2*dtheta1*dtheta2*cos(2*theta2);
    W[0][17] = -ddtheta2*sin(theta2)-pow(dtheta2,2)*cos(theta2);
    W[0][18] = -2*l1*ddtheta1*sin(theta2)-2*l1*dtheta1*dtheta2*cos(theta2)-az*sin(theta1)*sin(theta2);
    W[0][19] = -az*cos(theta1);
    W[0][20] = -2*l1*ddtheta1*cos(theta2)+2*l1*dtheta1*dtheta2*sin(theta2)-az*sin(theta1)*cos(theta2);
    W[0][21] = pow(l1,2)*ddtheta1+l1*az*sin(theta1);
    W[0][22] = 0;
    W[0][23] = 0;

    W[0][24] = ddtheta1*pow(cos(theta3-theta2),2)+(dtheta1*dtheta2-dtheta1*dtheta3)*sin(2*(theta3-theta2));
    W[0][25] = 0;
    W[0][26] = ddtheta1*pow(sin(theta3-theta2),2)-(dtheta1*dtheta2-dtheta1*dtheta3)*sin(2*(theta3-theta2));
    W[0][27] = (-ddtheta2+ddtheta3)*cos(theta3-theta2)-(pow(dtheta2,2)-2*dtheta2*dtheta3+pow(dtheta3,2))*sin(theta3-theta2);
    W[0][28] = sin(2*(theta3-theta2))*ddtheta1+2*(-dtheta1*dtheta2+dtheta1*dtheta3)*cos(2*(theta3-theta2));
    W[0][29] = (ddtheta3-ddtheta2)*sin(theta3-theta2)+(pow(dtheta2,2)-2*dtheta2*dtheta3+pow(dtheta3,2))*cos(theta3-theta2);
    W[0][30] = -2*(l1-l2*cos(theta2))*(sin(theta3-theta2)*ddtheta1+cos(theta3-theta2)*dtheta1*dtheta3)+2*(l1*cos(theta3-theta2)-l2*cos(2*theta2-theta3))*dtheta1*dtheta2-az*sin(theta1)*sin(theta3-theta2);
    W[0][31] = l2*ddtheta2*sin(theta2)+l2*pow(dtheta2,2)*cos(theta2)-az*cos(theta1);
    W[0][32] = 2*(l1-l2*cos(theta2))*(cos(theta3-theta2)*ddtheta1-sin(theta3-theta2)*dtheta1*dtheta3)+2*(l1*sin(theta3-theta2)+l2*sin(2*theta2-theta3))*dtheta1*dtheta2+az*sin(theta1)*cos(theta3-theta2);
    W[0][33] = pow(l1-l2*cos(theta2),2)*ddtheta1+2*(l1-l2*cos(theta2))*l2*dtheta1*dtheta2*sin(theta2)+(l1-l2*cos(theta2))*az*sin(theta1);
    W[0][34] = 0;
    W[0][35] = 0;

        //generalized_cf(i.e. generalized contact force) = H(q,dq,ddq)*basePara - F_torque
    for(int i = 0; i < BASEPARASIZE; i++)
    {
        de_lfside[0] += W[0][i]*baseParameters[0][i];
    }
    //average filter and low pass filter for generalized force
    generalized_cfFilterBuffer[0][(index_GFFB+1)%GFFILTERSIZE] = filter_gf*generalized_cfFilterBuffer[0][index_GFFB%GFFILTERSIZE]+(1-filter_gf)*(de_lfside[0] - generalizedJointWrench.x());
    double gf_buffer0 = 0;
    for(int i = 0; i < GFFILTERSIZE; i++)
    {
         gf_buffer0 += generalized_cfFilterBuffer[0][i]/GFFILTERSIZE;
    }
    generalized_cf[0] = gf_buffer0;
    //second generalized contact force
    for(int i = 0; i < 12; i++)
    {
         W[1][i] = 0;
    }
    W[1][12] = 0.5*pow(dtheta1,2)*sin(2*theta2);
    W[1][13] = ddtheta2;
    W[1][14] = -W[1][12];
    W[1][15] = cos(theta2)*ddtheta1;
    W[1][16] = pow(dtheta1,2)*cos(2*theta2);
    W[1][17] = -sin(theta2)*ddtheta1;
    W[1][18] = l1*pow(dtheta1,2)*cos(theta2)+az*cos(theta1)*cos(theta2);
    W[1][19] = 0;
    W[1][20] = -l1*pow(dtheta1,2)*sin(theta2)-az*cos(theta1)*sin(theta2);
    W[1][21] = 0;
    W[1][22] = dtheta2;
    W[1][23] = MyMathFunc::sgn(dtheta2);

    W[1][24] = -0.5*pow(dtheta1,2)*sin(2*(theta3-theta2));
    W[1][25] = ddtheta2-ddtheta3;
    W[1][26] = -W[1][24];
    W[1][27] = -cos(theta3-theta2)*ddtheta1;
    W[1][28] = pow(dtheta1,2)*cos(2*(theta3-theta2));
    W[1][29] =  -sin(theta3-theta2)*ddtheta1;
    W[1][30] = 2*l2*sin(theta3)*ddtheta2-l2*ddtheta3*sin(theta3)+(-l1*cos(theta3-theta2)+l2*cos(2*theta2-theta3))*pow(dtheta1,2)+2*l2*dtheta2*dtheta3*cos(theta3)-l2*pow(dtheta3,2)*cos(theta3)\
            - az*cos(theta1)*cos(theta3-theta2);
    W[1][31] = l2*sin(theta2)*ddtheta1;
    W[1][32] = -2*l2*cos(theta3)*ddtheta2+l2*ddtheta3*cos(theta3)-(l1*sin(theta3-theta2)+l2*sin(2*theta2-theta3))*pow(dtheta1,2)+2*l2*dtheta2*dtheta3*sin(theta3)-l2*pow(dtheta3,2)*sin(theta3)\
            - az*cos(theta1)*sin(theta3-theta2);
    W[1][33] = pow(l2,2)*ddtheta2-l2*sin(theta2)*(l1-l2*cos(theta2))*pow(dtheta1,2)-az*l2*cos(theta1)*sin(theta2);
    W[1][34] = 0;
    W[1][35] = 0;

        //generalized contact force = H(q,dq,ddq)*basePara - F_torque
    for(int i = 12; i < BASEPARASIZE; i++)
    {
        de_lfside[1] += W[1][i]*baseParameters[1][i];
    }
    //average filter and low pass filter for generalized force
    generalized_cfFilterBuffer[1][(index_GFFB+1)%GFFILTERSIZE] = filter_gf*generalized_cfFilterBuffer[1][index_GFFB%GFFILTERSIZE]+(1-filter_gf)*(de_lfside[1] - generalizedJointWrench.y());
    double gf_buffer1 = 0;
    for(int i = 0; i < GFFILTERSIZE; i++)
    {
         gf_buffer1 += generalized_cfFilterBuffer[1][i]/GFFILTERSIZE;
    }
    generalized_cf[1] = gf_buffer1;
        //third generalized contact force
    for(int i = 0; i < 24; i++)
    {
         W[2][i] = 0;
    }
    W[2][24] = 0.5*pow(dtheta1,2)*sin(2*(theta3-theta2));
    W[2][25] = ddtheta3-ddtheta2;
    W[2][26] = - W[2][24];
    W[2][27] = cos(theta3-theta2)*ddtheta1;
    W[2][28] = -pow(dtheta1,2)*cos(2*(theta3-theta2));
    W[2][29] = sin(theta3-theta2)*ddtheta1;
    W[2][30] = (l1-l2*cos(theta2))*pow(dtheta1,2)*cos(theta3-theta2)-l2*pow(dtheta2,2)*cos(theta3) - l2*sin(theta3)*ddtheta2 +az*cos(theta1)*cos(theta3-theta2);
    W[2][31] = 0;
    W[2][32] = (l1-l2*cos(theta2))*pow(dtheta1,2)*sin(theta3-theta2)-l2*pow(dtheta2,2)*sin(theta3) + l2*cos(theta3)*ddtheta2 +az*cos(theta1)*sin(theta3-theta2);
    W[2][33] = 0;
    W[2][34] = dtheta3;
    W[2][35] = MyMathFunc::sgn(dtheta3);

         //generalized contact force = H(q,dq,ddq)*basePara - F_torque
    for(int i = 24; i < BASEPARASIZE; i++)
    {
        de_lfside[2] += W[2][i]*baseParameters[2][i];
    }
    //average filter and low pass filter for generalized force
    generalized_cfFilterBuffer[2][(index_GFFB+1)%GFFILTERSIZE] = filter_gf*generalized_cfFilterBuffer[2][index_GFFB%GFFILTERSIZE]+(1-filter_gf)*(de_lfside[2] - generalizedJointWrench.z());
    double gf_buffer2 = 0;
    for(int i = 0; i < GFFILTERSIZE; i++)
    {
         gf_buffer2 += generalized_cfFilterBuffer[2][i]/GFFILTERSIZE;
    }
    generalized_cf[2] = gf_buffer2;

    //Calculate foot contact force matrix
    double P11 = 0, P12 = -cos(theta1)*(l1-l2*cos(theta2)+l3*cos(theta2-theta3)), P13 = -sin(theta1)*(l1-l2*cos(theta2)+l3*cos(theta2-theta3)), \
                   P21 = l2*cos(theta2)-l3*cos(theta2-theta3), P22 = -sin(theta1)*(l2*sin(theta2)-l3*sin(theta2-theta3)), \
                   P23 = cos(theta1)*(l2*sin(theta2)-l3*sin(theta2-theta3)), P31 = l3*cos(theta2-theta3),  \
                   P32 = -l3*sin(theta1)*sin(theta2-theta3), P33 = l3*cos(theta1)*sin(theta2-theta3);
    Matrix3d A;
    A << P11,P12,P13,P21,P22,P23,P31,P32,P33;
    Vector3d g_cf(generalized_cf[0], generalized_cf[1], generalized_cf[2]);
    //Contact force average filter
    contactforce_FilterBuffer[(index_CFFB+1)%CFFILTERSIZE] = filter_cf*contactforce_FilterBuffer[index_CFFB%CFFILTERSIZE]+(1-filter_cf)*A.inverse()*g_cf;
    Vector3d cf(0,0,0);
    for(int i = 0; i < CFFILTERSIZE; i++)
    {
         cf += contactforce_FilterBuffer[i]/CFFILTERSIZE;
    }
    if(cf.z() > 0)
        cf.z() = 0;
    contactforce_P = cf;
    //contact force and generalized force filter buffer index counter
    index_CFFB++;
    index_GFFB++;
}

void LimbModel::DynamicsCalFootContactForceWithoutAcc()
{
    double L[3][BASEPARASIZE];
    double l1 = GetLink()[0]->GetLength(), l2 = GetLink()[1]->GetLength(), l3 = GetLink()[2]->GetLength();
        //Gravity ccelerated velocity in x,y,z axis.
    double ax = -m_originAccVel.x();
    double ay = -m_originAccVel.y();
    double az = -m_originAccVel.z();
    //Dynamics equation : int(Mq..+Cq.+G) = int(tau + g_ext) . The result is stored in de_lf[3] (dynamics equation left side)
    double de_lfside[3] = {0,0,0};
        //Fill the W matrix based on the joints' trajectories.
    double theta1 = GetThetaPos(1), theta2 = GetThetaPos(2), theta3 = GetThetaPos(3);
    double dtheta1 = GetThetaVel(1), dtheta2 = GetThetaVel(2), dtheta3 = GetThetaVel(3);
    simstep++;
//    if(simstep % 200 == 0)
//        resetIntegral = 1;
    if(resetIntegral)
    {
        //When reset integral term, get the initial value
        theta1_t0 =GetThetaPos(1), theta2_t0 =GetThetaPos(2), theta3_t0 =GetThetaPos(3);
        dtheta1_t0 =GetThetaVel(1), dtheta2_t0 =GetThetaVel(2), dtheta3_t0 =GetThetaVel(3);
        memset(L_Integralsum, 0, sizeof(L_Integralsum));
        memset(torque_integral, 0, sizeof(torque_integral));
        memset(gcf_integral, 0, sizeof(gcf_integral));
        resetIntegral = 0;
    }
    for(int i = 1; i <7; i++)
    {
        L[0][i] = 0;
    }
    L[0][0] = dtheta1 - dtheta1_t0;
    L_Integralsum[0][7] += m_steptime*(ay*sin(theta1)-az*cos(theta1));
    L[0][7] =  L_Integralsum[0][7];
    L_Integralsum[0][8] += m_steptime*(ay*cos(theta1)+az*sin(theta1));
    L[0][8] = L_Integralsum[0][8];
    L[0][9] = 0;
    L_Integralsum[0][10] += m_steptime*dtheta1;
    L[0][10] = L_Integralsum[0][10];
    L_Integralsum[0][11] += m_steptime*MyMathFunc::sgn(dtheta1);
    L[0][11] = L_Integralsum[0][11];

    L[0][12] = pow(cos(theta2),2)*dtheta1-pow(cos(theta2_t0),2)*dtheta1_t0;
    L[0][13] = 0;
    L[0][14] = pow(sin(theta2),2)*dtheta1-pow(sin(theta2_t0),2)*dtheta1_t0;
    L[0][15] = cos(theta2)*dtheta2- cos(theta2_t0)*dtheta2_t0;
    L[0][16] = -sin(2*theta2)*dtheta1+sin(2*theta2_t0)*dtheta1_t0;
    L[0][17] = -sin(theta2)*dtheta2+sin(theta2_t0)*dtheta2_t0;
    L_Integralsum[0][18] += m_steptime*sin(theta2)*(-ay*cos(theta1)-az*sin(theta1));
    L[0][18] = -2*l1*(sin(theta2)*dtheta1-sin(theta2_t0)*dtheta1_t0)+L_Integralsum[0][18];
    L_Integralsum[0][19] += m_steptime*(ay*sin(theta1)-az*cos(theta1));
    L[0][19] = L_Integralsum[0][19];
    L_Integralsum[0][20] += m_steptime*cos(theta2)*(-ay*cos(theta1)-az*sin(theta1));
    L[0][20] = -2*l1*(cos(theta2)*dtheta1-cos(theta2_t0)*dtheta1_t0)+L_Integralsum[0][20];
    L_Integralsum[0][21] += m_steptime*l1*(ay*cos(theta1)+az*sin(theta1));
    L[0][21] = pow(l1,2)*(dtheta1-dtheta1_t0)+L_Integralsum[0][21];
    L[0][22] = 0;
    L[0][23] = 0;

    L[0][24] = pow(cos(theta3-theta2),2)*dtheta1-pow(cos(theta3_t0-theta2_t0),2)*dtheta1_t0;
    L[0][25] = 0;
    L[0][26] = pow(sin(theta3-theta2),2)*dtheta1-pow(sin(theta3_t0-theta2_t0),2)*dtheta1_t0;
    L[0][27] = cos(theta3-theta2)*(-dtheta2+dtheta3)-cos(theta3_t0-theta2_t0)*(-dtheta2_t0+dtheta3_t0);
    L[0][28] = sin(2*(theta3-theta2))*dtheta1-sin(2*(theta3_t0-theta2_t0))*dtheta1_t0;
    L[0][29] = sin(theta3-theta2)*(dtheta3-dtheta2)-sin(theta3_t0-theta2_t0)*(dtheta3_t0-dtheta2_t0);
    L_Integralsum[0][30] += m_steptime*sin(theta3-theta2)*(-ay*cos(theta1)-az*sin(theta1));
    L[0][30] = -2*sin(theta3-theta2)*(l1-l2*cos(theta2))*dtheta1+2*sin(theta3_t0-theta2_t0)*(l1-l2*cos(theta2_t0))*dtheta1_t0+L_Integralsum[0][30];
    L_Integralsum[0][31] += m_steptime*(ay*sin(theta1)-az*cos(theta1));
    L[0][31] = l2*sin(theta2)*dtheta2-l2*sin(theta2_t0)*dtheta2_t0+L_Integralsum[0][31];
    L_Integralsum[0][32] += m_steptime*cos(theta3-theta2)*(ay*cos(theta1)+az*sin(theta1));
    L[0][32] = 2*cos(theta3-theta2)*(l1-l2*cos(theta2))*dtheta1-2*cos(theta3_t0-theta2_t0)*(l1-l2*cos(theta2_t0))*dtheta1_t0+L_Integralsum[0][32];
    L_Integralsum[0][33] += m_steptime*(l1-l2*cos(theta2))*(ay*cos(theta1)+az*sin(theta1));
    L[0][33] = pow(l1-l2*cos(theta2),2)*dtheta1-pow(l1-l2*cos(theta2_t0),2)*dtheta1_t0+L_Integralsum[0][33];
    L[0][34] = 0;
    L[0][35] = 0;

    //second row
    for(int i = 0; i < 12; i++)
    {
        L[0+1][i] = 0;
    }
    L_Integralsum[1][12] += m_steptime*0.5*pow(dtheta1,2)*sin(2*theta2);
    L[0+1][12] = L_Integralsum[1][12];
    L[0+1][13] = dtheta2-dtheta2_t0;
    L[0+1][14] = -L_Integralsum[1][12];
    L_Integralsum[1][15] += m_steptime*sin(theta2)*dtheta1*dtheta2;
    L[0+1][15] = cos(theta2)*dtheta1-cos(theta2_t0)*dtheta1_t0 + L_Integralsum[1][15];
    L_Integralsum[1][16] += m_steptime*cos(2*theta2)*pow(dtheta1,2);
    L[0+1][16] = L_Integralsum[1][16];
    L_Integralsum[1][17] += m_steptime*cos(theta2)*dtheta1*dtheta2;
    L[0+1][17] = -sin(theta2)*dtheta1+sin(theta2_t0)*dtheta1_t0+L_Integralsum[1][17];
    L_Integralsum[1][18] += m_steptime*(l1*pow(dtheta1,2)*cos(theta2)-ax*sin(theta2)-ay*sin(theta1)*cos(theta2)+az*cos(theta1)*cos(theta2));
    L[0+1][18] = L_Integralsum[1][18];
    L[0+1][19] = 0;
    L_Integralsum[1][20] += m_steptime*(-l1*pow(dtheta1,2)*sin(theta2)-ax*cos(theta2)+ay*sin(theta1)*sin(theta2)-az*cos(theta1)*sin(theta2));
    L[0+1][20] =  L_Integralsum[1][20];
    L[0+1][21] = 0;
    L_Integralsum[1][22] += m_steptime*dtheta2;
    L[0+1][22] = L_Integralsum[1][22];
    L_Integralsum[1][23] += m_steptime*MyMathFunc::sgn(dtheta2);
    L[0+1][23] = L_Integralsum[1][23];

    L_Integralsum[1][24] += m_steptime*(-0.5*pow(dtheta1,2)*sin(2*theta3-2*theta2));
    L[0+1][24] = L_Integralsum[1][24];
    L[0+1][25] = dtheta2-dtheta3-dtheta2_t0+dtheta3_t0;
    L[0+1][26] = -L_Integralsum[1][24];
    L_Integralsum[1][27] += m_steptime*sin(theta3-theta2)*(dtheta1*dtheta2-dtheta1*dtheta3);
    L[0+1][27] = -cos(theta3-theta2)*dtheta1+cos(theta3_t0-theta2_t0)*dtheta1_t0+L_Integralsum[1][27];
    L_Integralsum[1][28] += m_steptime*pow(dtheta1,2)*cos(2*theta3-2*theta2);
    L[0+1][28] =L_Integralsum[1][28];
    L_Integralsum[1][29] += m_steptime*cos(theta3-theta2)*(-dtheta1*dtheta2+dtheta1*dtheta3);
    L[0+1][29] = -sin(theta3-theta2)*dtheta1+sin(theta3_t0-theta2_t0)*dtheta1_t0+L_Integralsum[1][29];
    L_Integralsum[1][30] += m_steptime*((-l1*cos(theta3-theta2)+l2*cos(2*theta2-theta3))*pow(dtheta1,2)-ax*sin(theta3-theta2)+ay*sin(theta1)*cos(theta3-theta2)-az*cos(theta1)*cos(theta3-theta2));
    L[0+1][30] = 2*l2*(sin(theta3)*dtheta2-sin(theta3_t0)*dtheta2_t0)-l2*(sin(theta3)*dtheta3-sin(theta3_t0)*dtheta3_t0)+L_Integralsum[1][30];
    L_Integralsum[1][31] += m_steptime*(-l2*cos(theta2)*dtheta1*dtheta2);
    L[0+1][31] = l2*(sin(theta2)*dtheta1-sin(theta2_t0)*dtheta1_t0)+L_Integralsum[1][31];
    L_Integralsum[1][32] += m_steptime*((-l1*sin(theta3-theta2)-l2*sin(2*theta2-theta3))*pow(dtheta1,2)+ax*cos(theta3-theta2)+ay*sin(theta1)*sin(theta3-theta2)-az*cos(theta1)*sin(theta3-theta2));
    L[0+1][32] = -2*l2*(cos(theta3)*dtheta2-cos(theta3_t0)*dtheta2_t0)+l2*(cos(theta3)*dtheta3-cos(theta3_t0)*dtheta3_t0)+L_Integralsum[1][32];
    L_Integralsum[1][33] += m_steptime*(-l2*sin(theta2)*(l1-l2*cos(theta2))*pow(dtheta1,2)-ax*l2*cos(theta2)+ay*l2*sin(theta1)*sin(theta2)-az*l2*cos(theta1)*sin(theta2));
    L[0+1][33] = pow(l2,2)*(dtheta2-dtheta2_t0)+L_Integralsum[1][33];
    L[0+1][34] = 0;
    L[0+1][35] = 0;

    //third row
    for(int i = 0; i < 24; i++)
    {
        L[0+2][i] = 0;
    }
    L_Integralsum[2][24] += m_steptime*0.5*pow(dtheta1,2)*sin(2*(theta3-theta2));
    L[0+2][24] = L_Integralsum[2][24];
    L[0+2][25] = dtheta3-dtheta2-dtheta3_t0+dtheta2_t0;
    L[0+2][26] = -L_Integralsum[2][24];
    L_Integralsum[2][27] += m_steptime*sin(theta3-theta2)*(-dtheta1*dtheta2+dtheta1*dtheta3);
    L[0+2][27] = cos(theta3-theta2)*dtheta1-cos(theta3_t0-theta2_t0)*dtheta1_t0+L_Integralsum[2][27];
    L_Integralsum[2][28] += m_steptime*(-pow(dtheta1,2))*cos(2*(theta3-theta2));
    L[0+2][28] = L_Integralsum[2][28];
    L_Integralsum[2][29] += m_steptime*cos(theta3-theta2)*(dtheta1*dtheta2-dtheta1*dtheta3);
    L[0+2][29] = sin(theta3-theta2)*dtheta1-sin(theta3_t0-theta2_t0)*dtheta1_t0+L_Integralsum[2][29];
    L_Integralsum[2][30] += m_steptime*((l1-l2*cos(theta2))*pow(dtheta1,2)*cos(theta3-theta2)-l2*cos(theta3)*(pow(dtheta2,2)-dtheta2*dtheta3)\
                                        +ax*sin(theta3-theta2)-(ay*sin(theta1)-az*cos(theta1))*cos(theta3-theta2));
    L[0+2][30] = -l2*sin(theta3)*dtheta2+l2*sin(theta3_t0)*dtheta2_t0+L_Integralsum[2][30];
    L[0+2][31] = 0;
    L_Integralsum[2][32] += m_steptime*((l1-l2*cos(theta2))*pow(dtheta1,2)*sin(theta3-theta2)-l2*sin(theta3)*(pow(dtheta2,2)-dtheta2*dtheta3)\
                                        -ax*cos(theta3-theta2)-(ay*sin(theta1)-az*cos(theta1))*sin(theta3-theta2));
    L[0+2][32] = l2*cos(theta3)*dtheta2-l2*cos(theta3_t0)*dtheta2_t0+L_Integralsum[2][32];
    L[0+2][33] = 0;
    L_Integralsum[2][34] += m_steptime*dtheta3;
    L[0+2][34] = L_Integralsum[2][34];
    L_Integralsum[2][35] += m_steptime*MyMathFunc::sgn(dtheta3);
    L[0+2][35] = L_Integralsum[2][35];

    //torque and contact force integral
    torque_integral[0] += generalizedJointWrench.x()*m_steptime;
    torque_integral[1] += generalizedJointWrench.y()*m_steptime;
    torque_integral[2] += generalizedJointWrench.z()*m_steptime;

     //generalized contact force = H(q,dq,ddq)*basePara - F_torque
    for(int i = 0; i < BASEPARASIZE; i++)
    {
        de_lfside[0] += L[0][i]*baseParameters[0][i];
    }
        //average filter and low pass filter for generalized force 1
    generalized_cfFilterBuffer[0][(index_GFFB+1)%GFFILTERSIZE] = filter_gf*generalized_cfFilterBuffer[0][index_GFFB%GFFILTERSIZE]+(1-filter_gf)*(de_lfside[0] - torque_integral[0]);
    double gf_buffer0 = 0;
    for(int i = 0; i < GFFILTERSIZE; i++)
    {
         gf_buffer0 += generalized_cfFilterBuffer[0][i]/GFFILTERSIZE;
    }
    generalized_cfint[0] = gf_buffer0;

    for(int i = 12; i < BASEPARASIZE; i++)
    {
        de_lfside[1] += L[1][i]*baseParameters[1][i];
    }
        //average filter and low pass filter for generalized force 2
    generalized_cfFilterBuffer[1][(index_GFFB+1)%GFFILTERSIZE] = filter_gf*generalized_cfFilterBuffer[1][index_GFFB%GFFILTERSIZE]+(1-filter_gf)*(de_lfside[1] - torque_integral[1]);
    double gf_buffer1 = 0;
    for(int i = 0; i < GFFILTERSIZE; i++)
    {
         gf_buffer1 += generalized_cfFilterBuffer[1][i]/GFFILTERSIZE;
    }
    generalized_cfint[1] = gf_buffer1;

    for(int i = 24; i < BASEPARASIZE; i++)
    {
        de_lfside[2] += L[2][i]*baseParameters[2][i];
    }
        //average filter and low pass filter for generalized force 3
    generalized_cfFilterBuffer[2][(index_GFFB+1)%GFFILTERSIZE] = filter_gf*generalized_cfFilterBuffer[2][index_GFFB%GFFILTERSIZE]+(1-filter_gf)*(de_lfside[2] - torque_integral[2]);
    double gf_buffer2 = 0;
    for(int i = 0; i < GFFILTERSIZE; i++)
    {
        gf_buffer2 += generalized_cfFilterBuffer[2][i]/GFFILTERSIZE;
    }
    generalized_cfint[2] = gf_buffer2;
//Generalized momentum observer(GM). See the article "contact Model Fusion for event-based locomotion in unstructured terrains".
    double lambda=2000;
    for(int i = 0; i < 3; i++)
    {
         gcf_integral[i] += m_steptime*gcf_eval[i];
         gcf_eval[i] = lambda*(generalized_cfint[i]-gcf_integral[i]);
    }

    //Calculate foot contact force matrix
    double P11 = 0, P12 = -cos(theta1)*(l1-l2*cos(theta2)+l3*cos(theta2-theta3)), P13 = -sin(theta1)*(l1-l2*cos(theta2)+l3*cos(theta2-theta3)), \
                   P21 = l2*cos(theta2)-l3*cos(theta2-theta3), P22 = -sin(theta1)*(l2*sin(theta2)-l3*sin(theta2-theta3)), \
                   P23 = cos(theta1)*(l2*sin(theta2)-l3*sin(theta2-theta3)), P31 = l3*cos(theta2-theta3),  \
                   P32 = -l3*sin(theta1)*sin(theta2-theta3), P33 = l3*cos(theta1)*sin(theta2-theta3);
    Matrix3d A;
    A << P11,P12,P13,P21,P22,P23,P31,P32,P33;
    Jac  << P11,P12,P13,P21,P22,P23,P31,P32,P33;
    Vector3d g_cf(gcf_eval[0], gcf_eval[1], gcf_eval[2]);
    //Contact force low pass and average filter
    contactforce_FilterBuffer[(index_CFFB+1)%CFFILTERSIZE] = filter_cf*contactforce_FilterBuffer[index_CFFB%CFFILTERSIZE]+(1-filter_cf)*A.inverse()*g_cf;
    Vector3d cf(0,0,0);
    for(int i = 0; i < CFFILTERSIZE; i++)
    {
         cf += contactforce_FilterBuffer[i]/CFFILTERSIZE;
    }
    if(cf.z() > 0)
        cf.z() = 0;
    contactforce_P = cf;
    index_CFFB++;
    index_GFFB++;
}

void LimbModel::EstimateFootState()
{
    /*Estimate whether foot is in contact state or not.*/
    static double lowthreshold = 200;
    double norm = sqrt(pow(contactforce_P.z(),2));
    if( norm > lowthreshold)
    {
        limb_state = LIMB_STATE::SUPPORT;
        lowthreshold = 100;
    }
    else
    {
        limb_state = LIMB_STATE::SWING;
        lowthreshold = 300;
    }

}

LimbModel::LIMB_STATE LimbModel::GetFootState()
{
    return limb_state;
}
