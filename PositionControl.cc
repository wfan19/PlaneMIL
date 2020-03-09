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
    this->model = _model;

    ignition::math::Vector3d altitude;

    altitude = _sdf->Get<ignition::math::Vector3d>("PID_altitude");
    altitudePID.Init(altitude.X(), altitude.Y(), altitude.Z(), 0, 0, PITCH_MAX, -PITCH_MAX);
    altitudePID.SetCmd(0);

    lastUpdateTime = this->model->GetWorld()->SimTime();

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
    bodyQuaternion = ignition::math::Quaterniond(
    lastIMUMsg.orientation().w(),
    lastIMUMsg.orientation().x(),
    lastIMUMsg.orientation().y(),
    lastIMUMsg.orientation().z());
    this->run();
}

void PositionControl::onRange(RangePtr &_rangeMsg)
{
    lastRangeMsg = *_rangeMsg;
    lastAltitude = this->lastRangeMsg.current_distance() * std::cos(this->bodyQuaternion.Roll()) * std::cos(this->bodyQuaternion.Pitch());
    this->run();
}

void PositionControl::onRC(RCPtr &_rcMsg)
{
    lastRCInputMsg = *_rcMsg;
    this->run();
}

void PositionControl::run()
{
    this->headingOut.set_x(this->getPitchSP());
    this->headingOut.set_y(this->getRollSP());
    this->headingOut.set_z(0);
    this->headingSPPub->Publish(headingOut);
}

float PositionControl::getPitchSP()
{
    // float altitudeTarget, error, pitchSP;
    // gazebo::common::Time currentTime;

    // pitchSP = 0.1f;
    // altitudeTarget = lastRCInputMsg.altitude();

    // error = lastAltitude - altitudeTarget;
    // currentTime = model->GetWorld()->SimTime();

    // pitchSP = altitudePID.Update(error, currentTime - lastUpdateTime);
    // lastUpdateTime = currentTime;
    // gzdbg << "Current pitch setpoint: " << pitchSP << ", error was "<< error << std::endl;
    // return pitchSP;

    return 0;
}

float PositionControl::getRollSP()
{
    float out;
    out = -std::atan(lastIMUMsg.linear_acceleration().y() / 9.80665f); //angle of force to counter
    out = lastRCInputMsg.roll();
    return out;
}