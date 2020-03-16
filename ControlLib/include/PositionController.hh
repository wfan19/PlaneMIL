#include "AttitudeController.hh"

class PositionController
{
public:
    // Struct containing sensor data
    struct SensorData
    {
        double range;

        double pitch;
        double roll;
        double yaw;

        double acceleration_lateral;
    };

    // Struct containing user settings 
    struct UserSettings
    {
        double altitude;
        double roll;
    };

    PositionController();
    ~PositionController();

    void PositionController::updateSensors(SensorData &_sensorData);
    void PositionController::updateUserSettings(UserSettings &_userSettings);

    AttitudeController::AttitudeSP PositionController::controlPosition();

private:
    double lastRange{0.0};
    double lastPitch{0.0};
    double lastIMURoll{0.0};
    double lastYaw{0.0};
    double lastAccel_lateral{0.0};

    double lastAltitude{0.0};
    double lastUserRoll{0.0};
};