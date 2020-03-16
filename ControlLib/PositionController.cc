#include "include/PositionController.hh"

PositionController::PositionController()
    : pid_altitude(0.13, 0.05, 0, 0, -1, 1, -1, 1) // PID controller initialization
{
}

PositionController::~PositionController()
{
}

void PositionController::updateSensors(SensorDataStruct::SensorData &_sensorData)
{
    this->lastPitch = _sensorData.pitch;
    this->lastRoll = _sensorData.roll;

    this->lastAltitude = _sensorData.range * std::cos(this->lastPitch) * std::cos(this->lastRoll);
}

void PositionController::updateUserSettings(PositionController::UserSettings &_userSettings)
{
    this->lastRollSP = _userSettings.rollSP;
    this->lastAltitudeSP = _userSettings.altitude;
}

AttitudeController::AttitudeSP PositionController::controlPosition(double dt)
{
    AttitudeController::AttitudeSP out;
    out.pitchSP = getPitchSP(dt);
    out.rollSP = getRollSP();

    return out;
}

double PositionController::getPitchSP(double dt)
{
    float error, pitchSP;
    error = lastAltitudeSP - lastAltitude;
    pitchSP = pid_altitude.update(lastAltitudeSP, error, dt);
    return pitchSP;
}

double PositionController::getRollSP()
{
    return this->lastRollSP;
}