#include "test_imu.h"
#include <ignition/math.hh>

namespace  gazebo{

test_imu::test_imu()
{

}

void test_imu::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    parentPtr = _model;
    updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&test_imu::OnUpdate, this));

}
void test_imu::OnUpdate()
{
    ignition::math::Vector3d anglevel(2.56,1.83,4.589);
    //SetAngularVel reference frame is World frame !!!
     parentPtr->SetAngularVel(anglevel);
    ignition::math::Vector3d angleVel;
    angleVel = parentPtr->RelativeAngularVel();
    std::cout << "World AngleVelocity : " << angleVel << std::endl;
}

}


