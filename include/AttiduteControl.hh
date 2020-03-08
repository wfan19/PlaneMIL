#ifndef ATTITUDECONTROL_HH
#define ATTITUDECONTROL_HH

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo{
    class GZ_PLUGIN_VISIBLE AttitudeControl : public ModelPlugin{
    public:
        AttitudeControl();
        ~AttitudeControl();

        // Documentation Inherited.
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    
    private:
        void initConnection();

        void onHeadingSP();

        void run();

        float getControlPitch();
        float getControlRoll();
        float getControlYaw();

        transport::NodePtr node;

        transport::SubscriberPtr headingSPSub;

        transport::PublisherPtr attitudeCtrlPub;

        msgs::Cessna ctrl_msg;
    };
} //namespace gazebo
#endif