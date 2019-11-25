#include "include/ds/traprocgen.hh"
#include "math.h"

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

