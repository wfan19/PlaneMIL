#include "include/AttiduteControl.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(AttitudeControl);

AttitudeControl::AttitudeControl()
{
}

AttitudeControl::~AttitudeControl()
{
}

void AttitudeControl::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    initConnection();
}

void AttitudeControl::initConnection()
{
    // Create a node for transportation
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init("default");

    this->rangeSub = this->node->Subscribe("~/plane/control/headingSP", &AttitudeControl::onHeadingSP, this);

    this->headingSPPub = this->node->Advertise<gazebo::msgs::Vector3d>("~/plane/control/headingSP");
}

void AttitudeControl::onHeadingSP()
{

}

void AttitudeControl::run()
{

}

float getControlPitch()
{
 
}

float getControlRoll()
{

}

float getControlYaw()
{

}