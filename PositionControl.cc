#include "include/PositionControl.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(PositionControl);

PositionControl::PositionControl()
    : positionController()
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
    gazebo::common::Time currentTime;
    gazebo::common::Time dt = currentTime - lastUpdateTime;

    if(dt != gazebo::common::Time(0,0))
    {
        PositionController::UserSettings userInput;
        userInput.altitude = lastRCInputMsg.altitude();
        userInput.rollSP = lastRCInputMsg.roll();
        positionController.updateUserSettings(userInput);

        SensorDataStruct::SensorData sensorData;
        sensorData.pitch = bodyQuaternion.Pitch();
        sensorData.roll = bodyQuaternion.Roll();
        sensorData.range = lastRangeMsg.current_distance();
        positionController.updateSensors(sensorData);

        AttitudeController::AttitudeSP attitudeSP = positionController.controlPosition(dt.Double());
        lastUpdateTime = currentTime;
    }
    
}

void PositionControl::initPIDs(sdf::ElementPtr &_sdf)
{
    ignition::math::Vector3d altitude;

    altitude = _sdf->Get<ignition::math::Vector3d>("PID_altitude"); 
    
    altitudePID.Init(altitude.X(), altitude.Y(), altitude.Z(), 0.001, -0.001, PITCH_MAX, -PITCH_MAX);
    altitudePID.SetCmd(0);

    PIDFF::PID_config altitudeConfig;
    altitudeConfig.kp = altitude.X();
    altitudeConfig.ki = altitude.Y();
    altitudeConfig.kd = altitude.Z();
    altitudeConfig.imin = -0.001;
    altitudeConfig.imax = 0.001;
    altitudeConfig.min = -PITCH_MAX;
    altitudeConfig.max = PITCH_MAX;

    positionController.init(altitudeConfig);

    gzdbg << "Got altitude PID with params " << altitude.X() << ", " << altitude.Y() << ", " << altitude.Z() << std::endl;

}