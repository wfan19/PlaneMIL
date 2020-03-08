#include "include/PositionControl.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(PositionControl);

PositionControl::PositionControl()
{
}

PositionControl::~PositionControl()
{
}

void PositionControl::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    initConnection();
}

void PositionControl::initConnection()
{
    // Create a node for transportation
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init("default");

    this->rangeSub = this->node->Subscribe("~/plane/link/lidar", &PositionControl::onRange, this);
    this->imuSub = this->node->Subscribe("~/plane/imu", &PositionControl::onIMU, this);
    this->rcSub = this->node->Subscribe("~/plane/rc", &PositionControl::onRC, this);

    this->headingSPPub = this->node->Advertise<gazebo::msgs::Vector3d>("~/plane/headingSP");
}

void PositionControl::onIMU(IMUPtr &_imuMsg)
{
    lastIMUMsg = *_imuMsg;
    this->run();
}

void PositionControl::onRange(RangePtr &_rangeMsg)
{
    lastRangeMsg = *_rangeMsg;
    this->run();
}

void PositionControl::onRC(RCPtr &_rcMsg)
{
    lastRCInputMsg = *_rcMsg;
    this->run();
}

void PositionControl::run()
{
    this->headingOut.set_x(this->getPitchSP(lastRCInputMsg, lastIMUMsg, lastRangeMsg));
    this->headingOut.set_y(this->getRollSP(lastRCInputMsg, lastIMUMsg, lastRangeMsg));
    this->headingOut.set_z(0);
    this->headingSPPub->Publish(headingOut);
}

float PositionControl::getPitchSP(control_msgs::msgs::RC rcMsg, 
                        sensor_msgs::msgs::IMU imuMsg, 
                        sensor_msgs::msgs::Range rangeMsg)
{
    return 0;
}

float PositionControl::getRollSP(control_msgs::msgs::RC rcMsg, 
                        sensor_msgs::msgs::IMU imuMsg, 
                        sensor_msgs::msgs::Range rangeMsg)
{
    float out;
    out = -std::atan(imuMsg.linear_acceleration().y() / 9.80665f); //angle of force to counter
    out += rcMsg.roll();
    gzdbg << "Out: " << out << std::endl;
    return out;
    // return 0;
}