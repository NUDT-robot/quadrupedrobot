/*
 * Author : ChangXu
*/
#ifndef TEST_IMU_H
#define TEST_IMU_H
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
namespace gazebo{
class test_imu : public ModelPlugin
{
    physics::ModelPtr parentPtr;
    event::ConnectionPtr updateConnection;
public:
    test_imu();
void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
void OnUpdate();
};

GZ_REGISTER_MODEL_PLUGIN(test_imu)
}

#endif // TEST_IMU_H
