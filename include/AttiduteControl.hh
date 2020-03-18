#ifndef ATTITUDECONTROL_HH
#define ATTITUDECONTROL_HH

#include <cmath>

#include <ignition/math/Quaternion.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>

#include "msgTypedefs.hh"

#include "../build/IMU.pb.h"

#include "../ControlLib/include/AttitudeController.hh"
#include "../ControlLib/include/SensorDataStruct.hh"

namespace gazebo{
    class GZ_PLUGIN_VISIBLE AttitudeControl : public ModelPlugin{
    public:
        AttitudeControl();
        ~AttitudeControl();

        // Documentation Inherited.
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    
    private:
        void initConnection();

        void onHeadingSP(ConstVector3dPtr &_headingSP);
        void onIMU(IMUPtr &_imuMsg);

        void run();

        void setControlPitch();
        void setControlRoll();
        void setControlYaw();

        void initPIDs(sdf::ElementPtr &_sdf);

        transport::NodePtr node;

        transport::SubscriberPtr headingSPSub;
        transport::SubscriberPtr imuSub;

        transport::PublisherPtr attitudeCtrlPub;

        gazebo::msgs::Vector3d lastHeadingSP;
        sensor_msgs::msgs::IMU lastIMUMsg;

        physics::ModelPtr model;
        gazebo::common::Time lastUpdateTime;
        gazebo::common::Time lastRollUpdateTime;

        msgs::Cessna ctrlMsg;

        gazebo::common::PID pitchPID;
        gazebo::common::PID rollPID;
        gazebo::common::PID yawPID;

        ignition::math::Quaterniond bodyQuaternion;

        AttitudeController attitudeController;

        float lastPitchOutput{0.0};
        float lastRollOutput{0.0};
        float lastYawOutput{0.0};

        const float AILERON_LIMIT = 30;  // +- 30deg
        const float ELEVATOR_LIMIT = 30; // +- 30deg
        const float RUDDER_LIMIT = 30;   // +- 30deg
    };
} //namespace gazebo
#endif