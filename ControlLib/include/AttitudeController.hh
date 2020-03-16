#include "SensorDataStruct.hh"
#include "PIDFF.hh"

class AttitudeController
{
public:
    struct AttitudeSP
    {
        double pitchSP;
        double rollSP;
    };

    struct ActuatorsSP
    {
        double elevatorSP;
        double aileronsSP;
    };

    AttitudeController();
    ~AttitudeController();

    void AttitudeController::updateSensors(SensorDataStruct::SensorData &_sensorData);
    void AttitudeController::updateAttitudeSP(AttitudeController::AttitudeSP &_attitudeSP);

    AttitudeController::ActuatorsSP AttitudeController::controlAttitudes(double dt);

    double AttitudeController::controlPitch(double dt);
    double AttitudeController::controlRoll(double dt);

private:

    PIDFF pid_pitch;
    PIDFF pid_roll;

    double lastPitch{0.0};
    double lastRoll{0.0};
    // double lastYaw{0.0};

    double lastPitchSP{0.0};
    double lastRollSP{0.0};
    //double lastYawSP{0.0};

};