#ifndef PLANE_MIL_GUI_PLUGIN
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class GAZEBO_VISIBLE PlaneMILGUIPlugin : public GUIPlugin
  {
    Q_OBJECT

    public: PlaneMILGUIPlugin();

    public: virtual ~PlaneMILGUIPlugin();
  };
}
#endif