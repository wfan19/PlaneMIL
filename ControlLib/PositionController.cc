#include "include/PositionController.hh"

PositionController::PositionController()
{
}

PositionController::~PositionController()
{
}

void PositionController::updateSensors(SensorData &_sensorData)
{
    this->lastPitch = _sensorData.pitch;
    this->lastRoll = _sensorData.roll;

    this->lastAltitude = _sensorData.range * std::cos(this->lastPitch) * std::cos(this->lastRoll);
}

void PositionController::updateUserSettings(UserSettings &_userSettings)
{
    this->lastRollSP = _userSettings.roll;
    this->lastAltitudeSP = _userSettings.altitude;
}

AttitudeController::AttitudeSP PositionController::controlPosition()
{

}