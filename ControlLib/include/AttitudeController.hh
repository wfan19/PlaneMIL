#ifndef ATTITUDECONTROLLER_HH
#define ATTITUDECONTROLLER_HH

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

    AttitudeController(PIDFF::PID_config PIDConfig_pitch, PIDFF::PID_config PIDConfig_roll);
    ~AttitudeController();

    void updateSensors(SensorDataStruct::SensorData &_sensorData);
    void updateAttitudeSP(AttitudeController::AttitudeSP &_attitudeSP);

    AttitudeController::ActuatorsSP controlAttitudes(double dt);

    double controlPitch(double dt);
    double controlRoll(double dt);

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

#endif