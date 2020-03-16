#include <cmath>

#include "AttitudeController.hh"
#include "PIDFF.hh"

class PositionController
{
public:
    // Struct containing sensor data
    struct SensorData
    {
        double range;

        double pitch; // assumes up is positive
        double roll; // assumes cw is positive (looking from behind)
        // double yaw;
    };

    // Struct containing user settings 
    struct UserSettings
    {
        double altitude;
        double rollSP;
    };

    PositionController();
    ~PositionController();

    void PositionController::updateSensors(SensorData &_sensorData);
    void PositionController::updateUserSettings(UserSettings &_userSettings);

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