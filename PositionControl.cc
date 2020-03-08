#include "include/PositionControl.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(PositionControl);

PositionControl::PositionControl()
{
    initConnection();
}

PositionControl::~PositionControl()
{
}

void PositionControl::initConnection()
{
    // Create a node for transportation
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init("default");

    this->rangeSub = this->node->Subscribe("~/plane/link/lidar", &PositionControl::onRange, this);
    this->imuSub = this->node->Subscribe("~/plane/imu", &PositionControl::onIMU, this);
    this->rcSub = this->node->Subscribe("~/plane/rc", &PositionControl::onRC, this);

    this->headingSPPub = this->node->Advertise<gazebo::msgs::Vector3d>("~/plane/control/headingSP");
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
    this->publishSetpoints();
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
    out = std::atan(imuMsg.linear_acceleration.y / 9.80665f); //angle of force to counter
    out += rcMsg.roll();
    return out;
}


void PositionControl::publishSetpoints()
{
    this->headingSPPub->Publish(headingOut);
}