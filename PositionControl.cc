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

    initPIDs(_sdf);

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
    float altitudeTarget, error, pitchSP;
    gazebo::common::Time currentTime;

    pitchSP = 0.1f;
    altitudeTarget = lastRCInputMsg.altitude();

    error = lastAltitude - altitudeTarget;
    currentTime = model->GetWorld()->SimTime();
    gazebo::common::Time dt = currentTime - lastUpdateTime;

    if(dt != gazebo::common::Time(0,0)){
        pitchSP = -altitudePID.Update(error, dt.Double());

        lastUpdateTime = currentTime;
        gzdbg << "AltitudeSP: " << altitudeTarget << std::endl;
        gzdbg << "Current pitchSP: " << pitchSP * 360 / 6.28 << std::endl;
    } else {
        gzerr << "PID Update time == 0" << std::endl;
    }
    return pitchSP;
}

float PositionControl::getRollSP()
{
    float out;
    out = -std::atan(lastIMUMsg.linear_acceleration().y() / 9.80665f); //angle of force to counter
    out = lastRCInputMsg.roll();
    return out;
}

void PositionControl::initPIDs(sdf::ElementPtr &_sdf)
{
    ignition::math::Vector3d altitude;

    altitude = _sdf->Get<ignition::math::Vector3d>("PID_altitude"); 
    
    altitudePID.Init(altitude.X(), altitude.Y(), altitude.Z(), 0.001, -0.001, PITCH_MAX, -PITCH_MAX);
    altitudePID.SetCmd(0);

    gzdbg << "Got altitude PID with params " << altitude.X() << ", " << altitude.Y() << ", " << altitude.Z() << std::endl;

}