/*
 * Transitional Process Generator
 *Author : Chang Xu
*/

#ifndef QRMATH_H
#define QRMATH_H

#include <Eigen/Dense>
class TransProcGenerator
{
    //type, 1 linear
    int type = 1;
    double input = 0;
    double target  = 0;
    int pointsnum  = 0;
    double stepvalue = 0;
    double value = 0;
public:
    TransProcGenerator() {}
    void Init(int gtype, double ginput, double gtarget, int gpointsnum);
    //Every time we call generator function, it gives a value according to parameters what we have assigned to
    double generator();
};
namespace  MyMathFunc{
int sgn(double d);
Eigen::Matrix3d CrossProductMatrixFromVector(const Eigen::Vector3d& v);
Eigen::Vector3d ClipEachElementInVector(const Eigen::Vector3d& v, const Eigen::Vector3d& low, const Eigen::Vector3d& high);

// Move this function to gait generator class
void RecalTarget(Eigen::Vector3d& targetpos, double l1, double l2, double l3);
}

#endif
