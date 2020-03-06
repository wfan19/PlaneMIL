#include "../build/IMU.pb.h"
#include "../build/Range.pb.h"
#include "../build/RC.pb.h"

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo{
    typedef const boost::shared_ptr<const sensor_msgs::msgs::IMU> IMUPtr;
    typedef const boost::shared_ptr<const sensor_msgs::msgs::Range> RangePtr;
    typedef const boost::shared_ptr<const control_msgs::msgs::RC> RCPtr;
}