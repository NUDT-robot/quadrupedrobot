#include "include/quadrupedrobot.hh"
#include "include/gait.hh"
#include <cmath>
#include <iostream>

using namespace Eigen;
using namespace std;

GaitGenerator::GaitGenerator(QuadRobot* parent) :initialposition(0,0,0), supportPlan(1,1,1,1), supportFact(1, 1, 1, 1), supportAdvise(1, 1, 1, 1)
{
    qrparent = parent;
    starttime = 0;
    globaltime = 0;
    fourstandtime = 0;
    flytime = 10;
    for(int i = 0; i < 4; i++)
    {
        targetposition[i] << 0, 0, 0;
    }
}


void GaitGenerator::GaitStop()
{
    supportPlan << 1, 1, 1, 1;
    //TODO
    supportAdvise << 1, 1, 1, 1;
}

void GaitGenerator::GaitWalk(const Vector3d& vc)
{
    for(int i = 0; i < 4; i++)
    {
        targetposition[i] =initialposition + 0.5*vc*(supporttime+fourstandtime)*timestep;
        targetposition[i].z() = initialposition.z() - liftheight;
        targetposition[i].x() = 0.05;
    }
    ulong elapsetime = globaltime - starttime;
    ulong cycletime = 4*swingtime + 4*fourstandtime;
    if(elapsetime <= fourstandtime)
    {
        supportPlan << 1, 1, 1, 1;
    }
    else if(elapsetime <= swingtime + fourstandtime) // four-legged stand time
    {
        supportPlan << 0, 1, 1, 1;
    }
    else if(elapsetime <= swingtime + 2*fourstandtime)
    {
        supportPlan << 1, 1, 1, 1;
    }
    else if(elapsetime <= 2*swingtime + 2*fourstandtime)
    {
        supportPlan << 1, 1, 1, 0;
    }
    else if(elapsetime <= 2*swingtime + 3*fourstandtime)
    {
        supportPlan << 1, 1, 1, 1;
    }
    else if(elapsetime <= 3*swingtime + 3*fourstandtime)
    {
        supportPlan << 1, 1, 0, 1;
    }
    else if(elapsetime <= 3*swingtime + 4*fourstandtime)
    {
        supportPlan << 1, 1, 1, 1;
    }
    else if(elapsetime <= 4*swingtime + 4*fourstandtime)
    {
        supportPlan << 1, 0, 1, 1;
    }

    if( elapsetime >= cycletime )
        starttime = globaltime;

    supportAdvise = supportPlan; // Simple but not good
}

void GaitGenerator::GaitTrot(const Vector3d& vc)
{
    for(int i = 0; i < 4; i++)
    {
        targetposition[i] =initialposition + 0.5*vc*(supporttime+fourstandtime)*timestep;
        targetposition[i].z() = initialposition.z() - liftheight;
    }
    ulong elapsetime = globaltime - starttime;
    ulong cycletime = 2*supporttime;
    if(elapsetime <= supporttime)
    {
        supportPlan << 0, 1, 1, 0;
        if( elapsetime >= supporttime/2)
        {
            targetposition[0].z() = initialposition.z();
            targetposition[3].z() = initialposition.z();
        }
    }
    else if(elapsetime <= supporttime + fourstandtime) // four-legged stand time
    {
        supportPlan << 1, 1, 1, 1;
    }
    else if(elapsetime <= cycletime + fourstandtime)
    {
        supportPlan << 1, 0, 0, 1;
        if(elapsetime >= 1.5*supporttime + fourstandtime)
        {
            targetposition[1].z() = initialposition.z();
            targetposition[2].z() = initialposition.z();
        }
    }
    else
    {
        supportPlan << 1, 1, 1, 1;
    }

    if( elapsetime >= cycletime+2*fourstandtime )
        starttime = globaltime;

    supportAdvise = supportPlan; // Simple but not good
}

void GaitGenerator::GaitPronk(const Vector3d& vc)
{
    for(int i = 0; i < 4; i++)
    {
        targetposition[i] =initialposition + 0.5*vc*(supporttime+fourstandtime)*timestep;
        targetposition[i].z() = initialposition.z() - liftheight;
    }
    ulong elapsetime = globaltime - starttime;
        // flyadd is only for test, it is stable time
    ulong flystabletime = 100;
    flytime = 2 * abs(qrparent->m_velocity_exp.z()) / 9.8 * 1000 + flystabletime;
    ulong cycletime = supporttime + flytime;

    if(elapsetime <= flytime)
    {
        supportPlan << 0, 0, 0, 0;
    }
    else if(elapsetime <= cycletime) // four-legged stand time
    {
        supportPlan << 1, 1, 1, 1;
        for(int i = 0; i < 4; i++)
        {
            targetposition[i] << 0, 0, 0;
        }
    }

    if( elapsetime >= cycletime )
        starttime = globaltime;

    supportAdvise = supportPlan; // Simple but not good
}

void GaitGenerator::GaitBound(const Vector3d& vc)
{
    for(int i = 0; i < 4; i++)
    {
        targetposition[i] = initialposition;
        targetposition[i].x() = 0.1;
        targetposition[i].z() = initialposition.z() - liftheight;
    }
    ulong elapsetime = globaltime - starttime;
        // flyadd is only for test, it is stable time
    ulong flystabletime = 0;
    flytime = 0.5 * 2 * abs(qrparent->m_velocity_exp.z()) / 9.8 * 1000 + flystabletime;
    ulong cycletime = fourstandtime + supporttime + flytime + swingtime;

    if(elapsetime <= fourstandtime)
    {
        supportPlan << 1, 1, 1, 1;
        for(int i = 0; i < 4; i++)
        {
            targetposition[i] << 0, 0, 0;
        }
    }
    else if(elapsetime <= fourstandtime + supporttime)
    {
        supportPlan << 0, 0, 1, 1;
        if(elapsetime > fourstandtime + supporttime/2)
        {
//            targetposition[0].z() = initialposition.z() ;
//            targetposition[1].z() = initialposition.z() ;
        }
        targetposition[2] << 0, 0, 0;
        targetposition[3] << 0, 0, 0;
    }
    else if(elapsetime <= fourstandtime + supporttime + flytime)
    {
        supportPlan << 0, 0, 0, 0;
    }
    else if(elapsetime <= cycletime)
    {
        supportPlan << 1, 1, 0, 0;
        if(elapsetime > cycletime - supporttime/2)
        {
//            targetposition[2].z() = initialposition.z() ;
//            targetposition[3].z() = initialposition.z() ;
        }
        targetposition[0] << 0, 0, 0;
        targetposition[1] << 0, 0, 0;
    }

    if( elapsetime >= cycletime )
        starttime = globaltime;

    supportAdvise = supportPlan; // Simple but not good
}

void GaitGenerator::init(GAIT gaitchoose, const Vector3d& initposition, double lifthei, ulong supporT, ulong  swingT)
{
    gait = gaitchoose;
    initialposition = initposition;
    supporttime = supporT;
    swingtime = swingT;
    liftheight = lifthei;
}

Vector4i GaitGenerator::GiveAdvise()
{
    return supportAdvise;
}

Vector3d* GaitGenerator::Step(const Vector3d& vc, const Vector4i& supportcase)
{
    double regular = 1;
    globaltime++;
    supportFact = supportcase;
    switch (gait)
    {
        case GAIT::STATIC:
            GaitStop();
            break;
        case GAIT::WALK:
            GaitWalk(vc);
            break;
        case GAIT::TROT:
            GaitTrot(vc);
                break;
        case GAIT::PRONK:
            GaitPronk(vc);
                break;
        case GAIT::BOUND:
            GaitBound(vc);
                break;
    }
    for(int i = 0; i < 4; i++)
    {
        // Transform world targetposition to body frame.
        targetposition[i] = qrparent->BodytoWorldDCM.transpose()*targetposition[i];
        // Generally, there is no solution to inverse kinetics of targetpos, so we should recalculate it.
        // This function should move to GaitGenerator class.
        double l1 = qrparent->limbVector[0]->GetLink()[0]->GetLength(), l2 = qrparent->limbVector[0]->GetLink()[1]->GetLength(), l3 = qrparent->limbVector[0]->GetLink()[2]->GetLength();
        if(abs(targetposition[i].x()) > abs(l2+l3))
        {
            // if Cx > abs(l2+l3)
            targetposition[i].x() = MyMathFunc::sgn(targetposition[i].x())*abs(l2+l3);
            targetposition[i].z() = l1;
        }
        else
        {
            targetposition[i].z() = std::min( sqrt(pow(l2+l3, 2) - pow(targetposition[i].x(), 2))+l1-1e-4, targetposition[i].z());
        }
    }

    return targetposition;

}

ulong GaitGenerator::GetSupportTime()
{
        return supporttime;
}

ulong GaitGenerator::GetSwingTime()
{
        return swingtime;
}
