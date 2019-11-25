#include "include/quadrupedrobot.hh"

void Sensor_INS::ProcessGazeboINSData(const gazebo::msgs::IMU&  imu_msg,  const ignition::transport:: MessageInfo &_info)
{
    gazebo::msgs::Quaternion quaternion;
    quaternion = imu_msg.orientation();

    ignition::math::Quaternion<double> quater(quaternion.w(),quaternion.x(),quaternion.y(),quaternion.z());
    ignition::math::Vector3d result;
    result =quater.Euler();
    ignition::math::Vector3d linear_acc(imu_msg.linear_acceleration().x(), imu_msg.linear_acceleration().y(), imu_msg.linear_acceleration().z());
    GetINSFromGazebo(result.X(), result.Y(), result.Z(),linear_acc);
}

Sensor_INS::Sensor_INS(QuadRobot* _parent)
{
    if(ConnectGazeboIMU())
        m_ready = true;
    else
    {
        std::cout << "Connect Gazebo IMU failed !" << std::endl;
    }
    parent = _parent;
}

bool Sensor_INS::ConnectGazeboIMU()
{
    if( m_ready)
        return false;
    if (!node.Subscribe("/EkinoxINS", &Sensor_INS::ProcessGazeboINSData,this))
    {
      std::cerr << "Error subscribing to topic [/EkinoxINS]" << std::endl;
      return false;
    }

    return true;
}

bool Sensor_INS::GetAttitude(double& roll, double& pitch, double& yaw)
{
    if(!m_ready)
    {
        std::cout << "INS data isn't ready" <<std::endl;
        return false;
    }
    roll = preroll;
    pitch = prepitch;
    yaw = preyaw;

    return true;
}

inline bool Sensor_INS::GetINSFromGazebo(double& roll, double& pitch, double& yaw, ignition::math::Vector3d& linearAcc)
{
    //Save INS data in QuadRobot class
    parent->m_roll = roll;
    parent->m_pitch = pitch;
    parent->m_yaw = yaw;
    parent->m_accVel = linearAcc;
    //calculate body angle velocity and save data in QuadRobot class
    yawVel = (yaw - preyaw) * m_factorTime;
    pitchVel = (pitch - prepitch) * m_factorTime;
    rollVel = (roll - preroll) * m_factorTime;
    parent-> m_angleVel.X() = -sin(yaw) * pitchVel + cos(yaw)*cos(pitch)*rollVel;
    parent->m_angleVel.Y() = cos(yaw)*pitchVel + sin(yaw)*cos(pitch)*rollVel;
    parent->m_angleVel.Z() = yawVel - sin(pitch)*rollVel;
    //Save previous attitude angles in sensor
    preroll = roll;
    prepitch = pitch;
    preyaw = yaw;
    return true;
}
