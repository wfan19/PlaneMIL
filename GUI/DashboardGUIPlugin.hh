#ifndef DASHBOARD_GUI_PLUGIN
#define DASHBOARD_GUI_PLUGIN

#include <string>
#include <sstream>
#include <cmath>

#include <ignition/math/Quaternion.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/transport/transport.hh>

#include "../build/IMU.pb.h"
#include "../build/Range.pb.h"

#include "include/msgTypedefs.hh"

namespace gazebo
{

    class GAZEBO_VISIBLE DashboardGUIPlugin : public GUIPlugin
    {
        Q_OBJECT

        public: DashboardGUIPlugin();

        public: virtual ~DashboardGUIPlugin();

        private: void initUI();

        private: void initConnection();

        private: void onRange(RangePtr &_range);
        private: void onIMU(IMUPtr &_imu);

        signals: void setAltitude(QString _alt);
        signals: void setHeading(QString _heading);

        private: std::string formatAltitude();

        private: std::string formatHeading();

        /// \brief Node used to establish communication with gzserver.
        private: transport::NodePtr node;

        /// \brief Subscriber to messages.
        private: transport::SubscriberPtr rangeSub;
        private: transport::SubscriberPtr imuSub;

        private: sensor_msgs::msgs::IMU lastIMUMsg;
        private: sensor_msgs::msgs::Range lastRangeMsg;

        private: ignition::math::Quaterniond bodyQuaterniond;
    };
}
#endif