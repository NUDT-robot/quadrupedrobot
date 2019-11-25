/*
 * Author : ChangXu
*/
#ifndef QRMESSAGE_H
#define QRMESSAGE_H
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

/*This is the Ekinox INS model plugin*/
namespace  gazebo
{
    //QRMessage
    class QRINSMessage : public SensorPlugin
    {
        sensors::ImuSensorPtr IMU_Sensor;
        ignition::math::Quaternion<double> orientation;
        event::ConnectionPtr updateConnection;
        ignition::transport::Node node;
        ignition::transport::Node::Publisher IMUdataPublisher;
    public:
        QRINSMessage();
        void Load(sensors::SensorPtr _sensor,sdf::ElementPtr _sdf) ;
        void OnUpdate();

    };
    //Do not forget to register !!!
    GZ_REGISTER_SENSOR_PLUGIN(QRINSMessage)
}//namespace gazebo



#endif // QRMESSAGE_H
