cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set (CMAKE_AUTOMOC ON)

# ------- #
# Find packages
# ------- #
set (required_packages
    gazebo
    Qt5Widgets
    Qt5Core
    Protobuf
)

foreach(package ${required_packages})
    find_package(${package} REQUIRED)
endforeach()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(SYSTEM 
  ${GAZEBO_INCLUDE_DIRS} 
  ${Qt5Core_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}/eigen3	# Workaround for Eigen3
  /home/bf01/Documents/Code/GazeboProjects/PlaneMIL/ControlLib/include
  )
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# ------- #
# Messages
# ------- #
set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set (sensor_msgs
  msgs/IMU.proto
  msgs/Range.proto
  ${PROTOBUF_IMPORT_DIRS}/vector3d.proto
  ${PROTOBUF_IMPORT_DIRS}/quaternion.proto
  ${PROTOBUF_IMPORT_DIRS}/header.proto
  ${PROTOBUF_IMPORT_DIRS}/time.proto
)

set (control_msgs
  msgs/RC.proto
)

PROTOBUF_GENERATE_CPP(SENSOR_PROTO_SRCS SENSOR_PROTO_HDRS ${sensor_msgs})
PROTOBUF_GENERATE_CPP(CONTROL_PROTO_SRCS CONTROL_PROTO_HDRS ${control_msgs})

add_library(sensor_msgs SHARED ${SENSOR_PROTO_SRCS})
add_library(control_msgs SHARED ${CONTROL_PROTO_SRCS})

link_libraries(sensor_msgs control_msgs ${PROTOBUF_LIBRARY})

# ------- #
# ControlLib
# ------- #
set (controlLib
    AttitudeController
    PIDFF
    PositionController
)

foreach(controlLibClass ${controlLib})
    add_library(${controlLibClass} SHARED ControlLib/${controlLibClass}.cc)
    target_link_libraries(${controlLibClass} ${GAZEBO_LIBRARIES})
endforeach()


# ------- #
# Plugins
# ------- #
set (plugins
    IMUPlugin
    LidarPlugin
    PlaneMILPlugin
    PositionControl
    AttitudeControl
)

set (gui_plugins
    PlaneMILGUIPlugin
    DashboardGUIPlugin
)

# Build normal plugins
foreach(plugin ${plugins})
    add_library(${plugin} SHARED ${plugin}.cc)
    target_link_libraries(${plugin} ${GAZEBO_LIBRARIES})
endforeach()
target_link_libraries(IMUPlugin Eigen3::Eigen)
target_link_libraries(LidarPlugin Eigen3::Eigen)

# Build GUI plugins
foreach(gui_plugin ${gui_plugins})
    add_library(${gui_plugin} SHARED ${headers_MOC} GUI/${gui_plugin}.cc)
    target_link_libraries(${gui_plugin} ${GAZEBO_LIBRARIES}
        # ${OGRE_LIBRARIES}
        # ${OGRE-Terrain_LIBRARIES}
        ${Qt5Core_LIBRARIES}
        ${Qt5Widgets_LIBRARIES}
        ${PROTOBUF_LIBRARIES})
endforeach()

add_executable(PlaneMILCLI PlaneMILCLI.cc)
target_link_libraries(PlaneMILCLI ${GAZEBO_LIBRARIES})
target_link_libraries(PlaneMILCLI readline)