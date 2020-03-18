#ifndef POSITIONCONTROLLER_HH
#define POSITIONCONTROLLER_HH

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

    PositionController(PIDFF::PID_config PIDConfig_altitude);
    ~PositionController();

    void updateSensors(SensorDataStruct::SensorData &_sensorData);
    void updateUserSettings(PositionController::UserSettings &_userSettings);

    AttitudeController::AttitudeSP controlPosition(double dt);
    double getPitchSP(double dt);
    double getRollSP();


private:

    PIDFF pid_altitude;

    double lastAltitude{0.0};
    double lastPitch{0.0};
    double lastRoll{0.0};
    // double lastYaw{0.0};

    double lastAltitudeSP{0.0};
    double lastRollSP{0.0};
};

#endif