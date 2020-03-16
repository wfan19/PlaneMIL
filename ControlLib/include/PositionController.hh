#include <cmath>

#include "AttitudeController.hh"
#include "SensorDataStruct.hh"
#include "PIDFF.hh"

class PositionController
{
public:

    // Struct containing user settings 
    struct UserSettings
    {
        double altitude;
        double rollSP;
    };

    PositionController();
    ~PositionController();

    void PositionController::updateSensors(SensorDataStruct::SensorData &_sensorData);
    void PositionController::updateUserSettings(PositionController::UserSettings &_userSettings);

    AttitudeController::AttitudeSP PositionController::controlPosition(double dt);
    double PositionController::getPitchSP(double dt);
    double PositionController::getRollSP();


private:

    PIDFF pid_altitude;

    double lastAltitude{0.0};
    double lastPitch{0.0};
    double lastRoll{0.0};
    // double lastYaw{0.0};

    double lastAltitudeSP{0.0};
    double lastRollSP{0.0};
};