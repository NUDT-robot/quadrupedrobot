/*
 * Transitional Process Generator
 *Author : Chang Xu
*/
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

