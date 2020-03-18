#ifndef POSITION_CONTROLHH
#define POSITION_CONTROLHH

#include <cmath>
#include <string>
#include <vector>

#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>

#include "../build/IMU.pb.h"
#include "../build/Range.pb.h"
#include "../build/RC.pb.h"

#include "msgTypedefs.hh"

#include "../ControlLib/include/PositionController.hh"
#include "../ControlLib/include/SensorDataStruct.hh"

namespace gazebo{


    class GZ_PLUGIN_VISIBLE PositionControl : public ModelPlugin
    {
    public:
        PositionControl();
        ~PositionControl();

        // Documentation Inherited.
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    private:
        void initConnection();

        void onIMU(IMUPtr &_imuMsg);

        void onRange(RangePtr &_rangeMsg);

        void onRC(RCPtr &_rcMsg);

        void run();

        float getPitchSP();

        float getRollSP();

        void initPIDs(sdf::ElementPtr &_sdf);

        gazebo::common::Time lastUpdateTime;
        gazebo::physics::ModelPtr model;

        transport::NodePtr node;

        transport::SubscriberPtr rangeSub;
        transport::SubscriberPtr imuSub;
        transport::SubscriberPtr rcSub;

        transport::PublisherPtr headingSPPub;

        sensor_msgs::msgs::IMU lastIMUMsg;
        sensor_msgs::msgs::Range lastRangeMsg;
        
        control_msgs::msgs::RC lastRCInputMsg;

        gazebo::msgs::Vector3d headingOut;

        gazebo::common::PID altitudePID;

        ignition::math::Quaterniond bodyQuaternion;

        PositionController positionController;

        float lastAltitude{0.0f};

        const float PITCH_MAX{30.0f * 3.14 / 180};
        const float ROLL_MAX{30.0f * 3.14 / 180};
        const float YAW_MAX{15.0f * 3.14 / 180};
    };
} // namespace gazebo

#endif