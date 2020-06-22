#include "include/ds/traprocgen.hh"
#include <iostream>
#include <cmath>

using namespace std;

void TransProcGenerator::Init(int gtype, double ginput, double gtarget, int gpointsnum)
{
    type = gtype;
    input = ginput;
    target = gtarget;
    pointsnum = gpointsnum;

    value = input;
    stepvalue = (target - input)/pointsnum;
};

double TransProcGenerator::generator()
{
    //When achieve target, we should set type to zero to output target value thereafter.
    switch(type)
    {
    case 0:
        return value;
        break;
    case 1:   //linear generator
        value += stepvalue;
        if( (stepvalue > 0 && value > target) || (stepvalue <= 0 && value < target) )
        {
            value = target;
            type = 0;
        }
        return value;
        break;
    default:
        ;
    }
}
namespace  MyMathFunc{

int sgn(double d)
{
    double eps = std::numeric_limits<double>::epsilon();
    return d < -eps ? -1 : d>eps;
}

Eigen::Matrix3d CrossProductMatrixFromVector(const Eigen::Vector3d& v)
{
    Eigen::Matrix3d M;
    M << 0,       -v.z(),   v.y(),
               v.z(),      0,    -v.x(),
              -v.y(),  v.x(),     0    ;
    return M;
}
Eigen::Vector3d ClipEachElementInVector(const Eigen::Vector3d& v, const Eigen::Vector3d& low, const Eigen::Vector3d& high)
{
    if(low.x() > high.x() or low.y() > high.y() or low.z() > high.z())
    {
        cout << "Error : ClipEachElementInVector Function. Make sure that low.x() > high.x() or low.y() > high.y() or low.z() > high.z(). "<< endl;
        return Eigen::Vector3d::Zero();
    }
    Eigen::Vector3d r;
    r.x() = max( min(v.x(), high.x()), low.x());
    r.y() = max( min(v.y(), high.y()), low.y());
    r.z() = max( min(v.z(), high.z()), low.z());
    return r;
}

void RecalTarget(Eigen::Vector3d& targetpos, double l1, double l2, double l3)
{
    if(abs(targetpos.x()) > abs(l2+l3))
    {
        // if Cx > abs(l2+l3)
        targetpos.x() = MyMathFunc::sgn(targetpos.x())*abs(l2+l3);
        targetpos.z() = l1;
    }
    else
    {
        targetpos.z() = min( sqrt(pow(l2+l3, 2) - pow(targetpos.x(), 2))+l1-1e-4, targetpos.z());
    }
}

}
