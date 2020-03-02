#include <sstream>
#include <gazebo/msgs/msgs.hh>
#include "PlaneMILGUIPlugin.hh"

using namespace gazebo;

GZ_REGISTER_GUI_PLUGIN(PlaneMILGUIPlugin)

PlaneMILGUIPlugin::PlaneMILGUIPlugin()
  : GUIPlugin()
{
  // This is needed to avoid the creation of a black widget with default size.
  this->move(-1, -1);
  this->resize(1, 1);
}

PlaneMILGUIPlugin::~PlaneMILGUIPlugin()
{
}