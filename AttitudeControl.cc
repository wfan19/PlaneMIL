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
    error = bodyQuaternion.Pitch() - target ;
    
    double angleTarget = -(this->ELEVATOR_LIMIT * this->pitchPID.Update(error, dt)); // Get target angle on a scale from -1 to 1, and then scale by limit
    this->ctrlMsg.set_cmd_elevators(angleTarget);
    // gzdbg << "Target, error, angleTarget: " << target << "," << error << "," << angleTarget << std::endl;

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

    lastUpdateTime = currentTime;
    
    this->attitudeCtrlPub->Publish(ctrlMsg);
}

void AttitudeControl::setControlYaw()
{
}

void AttitudeControl::initPIDs(sdf::ElementPtr &_sdf)
{
    ignition::math::Vector3d pitch, roll;

    pitch = _sdf->Get<ignition::math::Vector3d>("PID_pitch");
    roll = _sdf->Get<ignition::math::Vector3d>("PID_roll");
    
    pitchPID.Init(pitch.X(), pitch.Y(), pitch.Z(), 0.005, -0.005, 1, -1);
    pitchPID.SetCmd(0);

    rollPID.Init(roll.X(), roll.Y(), roll.Z(), 0.005, -0.005, 1, -1);
    rollPID.SetCmd(0);
}