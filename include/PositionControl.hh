#ifndef POSITION_CONTROLHH
#define POSITION_CONTROLHH

#include <cmath>

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include "build/IMU.pb.h"
#include "build/Range.pb.h"
#include "build/RC.pb.h"

#include "msgTypedefs.hh"

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

        float getPitchSP(control_msgs::msgs::RC rcMsg, 
                            sensor_msgs::msgs::IMU imuMsg, 
                            sensor_msgs::msgs::Range rangeMsg);

        float getRollSP(control_msgs::msgs::RC rcMsg, 
                            sensor_msgs::msgs::IMU imuMsg, 
                            sensor_msgs::msgs::Range rangeMsg);
                            
        transport::NodePtr node;

        transport::SubscriberPtr rangeSub;
        transport::SubscriberPtr imuSub;
        transport::SubscriberPtr rcSub;

        transport::PublisherPtr headingSPPub;

        sensor_msgs::msgs::IMU lastIMUMsg;
        sensor_msgs::msgs::Range lastRangeMsg;
        
        control_msgs::msgs::RC lastRCInputMsg;

        gazebo::msgs::Vector3d headingOut;

        const float ROLL_MAX{45.0f};
        const float PITCH_MAX{45.0f};
        const float YAW_MAX{15.0f};
    };
} // namespace gazebo

#endif