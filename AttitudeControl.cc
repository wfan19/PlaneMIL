#include "include/AttiduteControl.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(AttitudeControl);

AttitudeControl::AttitudeControl()
{
}

AttitudeControl::~AttitudeControl()
{
}

void AttitudeControl::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    initConnection();

    rollPID.Init(40, 0, 0, 0, 0, 1, -1);
    rollPID.SetCmd(0);
    
    this->model = _model;
    lastUpdateTime = this->model->GetWorld()->SimTime();

    run();
}

void AttitudeControl::initConnection()
{
    // Create a node for transportation
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init("default");

    this->headingSPSub = this->node->Subscribe("~/plane/headingSP", &AttitudeControl::onHeadingSP, this);
    this->imuSub = this->node->Subscribe("~/plane/imu", &AttitudeControl::onIMU, this);

    this->attitudeCtrlPub = this->node->Advertise<gazebo::msgs::Vector3d>("~/plane/control");
}

void AttitudeControl::onHeadingSP(gazebo::msgs::Vector3dPtr &_headingSP)
{
    lastHeadingSP = *_headingSP;
    run();
}

void AttitudeControl::onIMU(IMUPtr &_imuMsg)
{
    lastIMUMsg = *_imuMsg;
    bodyQuaternion = ignition::math::Quaterniond(
    lastIMUMsg.orientation().w(),
    lastIMUMsg.orientation().x(),
    lastIMUMsg.orientation().y(),
    lastIMUMsg.orientation().z());
    run();
}

void AttitudeControl::run()
{
    setControlRoll();
    attitudeCtrlPub->Publish(ctrlMsg);
}

void AttitudeControl::setControlPitch()
{
}

void AttitudeControl::setControlRoll()
{
    float target, error;
    gazebo::common::Time currentTime, dt;

    currentTime = this->model->GetWorld()->SimTime();
    dt = currentTime - this->lastUpdateTime;

    target = lastHeadingSP.y(); // get roll setpoint
    error = target - bodyQuaternion.Roll();
    
    double angleTarget = this->AILERON_LIMIT * this->rollPID.Update(error, dt); // Get target angle on a scale from -1 to 1, and then scale by limit
    this->ctrlMsg.set_cmd_left_aileron(angleTarget);
    this->ctrlMsg.set_cmd_right_aileron(-angleTarget);
}

void AttitudeControl::setControlYaw()
{
}