#include <iostream>
#include "qrINSmessage.h"

namespace  gazebo{

QRINSMessage::QRINSMessage()
{

}

void QRINSMessage::Load(sensors::SensorPtr _sensor,sdf::ElementPtr _sdf)
{
    this->IMU_Sensor =  std::dynamic_pointer_cast<sensors::ImuSensor>(_sensor);
    updateConnection = IMU_Sensor->ConnectUpdated(std::bind(&QRINSMessage::OnUpdate, this));
    IMUdataPublisher = node.Advertise<gazebo::msgs::IMU>("/EkinoxINS");
}
void QRINSMessage::OnUpdate()
{
    IMUdataPublisher.Publish(IMU_Sensor->ImuMessage());
}


}//namespace gazebo
