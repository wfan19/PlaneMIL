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
    initPIDs(_sdf);
    
    this->model = _model;
    lastUpdateTime = this->model->GetWorld()->SimTime();

    run();
}

void AttitudeControl::initConnection()
{
    // Create a node for transportation
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init("default");

    // this->headingSPSub = this->node->Subscribe("~/plane/headingSP", &AttitudeControl::onHeadingSP, this);
    this->headingSPSub = this->node->Subscribe("~/plane/headingSP", &AttitudeControl::onHeadingSP, this);
    this->imuSub = this->node->Subscribe("~/plane/imu", &AttitudeControl::onIMU, this);

    this->attitudeCtrlPub = this->node->Advertise<gazebo::msgs::Cessna>("~/plane/control");
}

void AttitudeControl::onHeadingSP(ConstVector3dPtr &_headingSP)
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
    setControlPitch();
    setControlRoll();
}

void AttitudeControl::setControlPitch()
{
    float target, error;
    gazebo::common::Time currentTime, dt;

    currentTime = this->model->GetWorld()->SimTime();
    dt = currentTime - this->lastUpdateTime;

    target = lastHeadingSP.x(); // get pitch setpoint
    error = target - bodyQuaternion.Pitch();
    
    double angleTarget = this->ELEVATOR_LIMIT * this->pitchPID.Update(error, dt); // Get target angle on a scale from -1 to 1, and then scale by limit
    this->ctrlMsg.set_cmd_elevators(-angleTarget);

    this->attitudeCtrlPub->Publish(ctrlMsg);
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

    this->attitudeCtrlPub->Publish(ctrlMsg);
}

void AttitudeControl::setControlYaw()
{
}

void AttitudeControl::initPIDs(sdf::ElementPtr &_sdf)
{
    float Kp_pitch = 0;
    float Ki_pitch = 0;
    float Kd_pitch = 0;

    Kp_pitch = _sdf->Get<float>("Kp_pitch");
    Ki_pitch = _sdf->Get<float>("Ki_pitch");
    Kd_pitch = _sdf->Get<float>("Kd_pitch");
    pitchPID.Init(Kp_pitch, Ki_pitch, Kd_pitch, 0, 0, 1, -1);
    pitchPID.SetCmd(0);

    float Kp_roll = 0;
    float Ki_roll = 0;
    float Kd_roll = 0;

    Kp_roll = _sdf->Get<float>("Kp_roll");
    Ki_roll = _sdf->Get<float>("Ki_roll");
    Kd_roll = _sdf->Get<float>("Kd_roll");
    rollPID.Init(Kp_roll, Ki_roll, Kd_roll, 0, 0, 1, -1);
    rollPID.SetCmd(0);
}