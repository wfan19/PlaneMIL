#include "include/AttitudeController.hh"

AttitudeController::AttitudeController(PIDFF::PID_config PIDConfig_pitch, PIDFF::PID_config PIDConfig_roll)
    // Initialize the PIDs through the initialziation list:
    : pid_pitch(PIDConfig_pitch)
    , pid_roll(PIDConfig_roll)
{
}

AttitudeController::AttitudeController()
    : pid_pitch(0,0,0,0,0,0,0,0)
    , pid_roll(0,0,0,0,0,0,0,0)
{
}

AttitudeController::~AttitudeController()
{
}

void AttitudeController::init(PIDFF::PID_config PIDConfig_pitch, PIDFF::PID_config PIDConfig_roll)
{
    pid_pitch.init(PIDConfig_pitch);
    pid_roll.init(PIDConfig_roll);
}

void AttitudeController::updateSensors(SensorDataStruct::SensorData &_sensorData)
{
    this->lastPitch = _sensorData.pitch;
    this->lastRoll = _sensorData.roll;
}

void AttitudeController::updateAttitudeSP(AttitudeController::AttitudeSP &_attitudeSP)
{
    this->lastPitchSP = _attitudeSP.pitchSP;
    this->lastRollSP = _attitudeSP.rollSP;
}

AttitudeController::ActuatorsSP AttitudeController::controlAttitudes(double dt)
{
    AttitudeController::ActuatorsSP actuatorsSP;
    actuatorsSP.elevatorSP = controlPitch(dt);
    actuatorsSP.aileronsSP = controlRoll(dt);
    return actuatorsSP;
}

double AttitudeController::controlPitch(double dt)
{
    double error, out;
    error = lastPitchSP - lastPitch;
    out = pid_pitch.update(lastPitchSP, error, dt);
    return out;
}

double AttitudeController::controlRoll(double dt)
{
    double error, out;
    error = lastRollSP - lastRoll;
    out = pid_roll.update(lastRollSP, error, dt);
    return out;
}