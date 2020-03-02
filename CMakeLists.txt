cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set (CMAKE_AUTOMOC ON)

# Find Gazebo
find_package(gazebo REQUIRED)
find_package (Qt5Widgets REQUIRED)
find_package (Qt5Core REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(PlaneMIL SHARED PlaneMILPlugin.cc)
target_link_libraries(PlaneMIL ${GAZEBO_LIBRARIES})

add_library(PlaneMILGUI SHARED ${headers_MOC}
PlaneMILGUIPlugin.cc)
target_link_libraries(PlaneMILGUI ${GAZEBO_LIBRARIES}
    # ${OGRE_LIBRARIES}
    # ${OGRE-Terrain_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    ${PROTOBUF_LIBRARIES})

add_library(gui_example_time_widget SHARED ${headers_MOC}
GUIExampleTimeWidget.cc)
target_link_libraries(gui_example_time_widget ${GAZEBO_LIBRARIES}
    # ${OGRE_LIBRARIES}
    # ${OGRE-Terrain_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    ${PROTOBUF_LIBRARIES})

add_executable(PlaneMILCLI PlaneMILCLI.cc)
target_link_libraries(PlaneMILCLI ${GAZEBO_LIBRARIES})
target_link_libraries(PlaneMILCLI readline)