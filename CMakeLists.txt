cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set (CMAKE_AUTOMOC ON)

# Find packages
set (required_packages
    gazebo
    Qt5Widgets
    Qt5Core
    Protobuf
)

foreach(package ${required_packages)
    find_package(${package} REQUIRED)
endforeach()

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

set (plugins
    IMUPlugin
    LidarPlugin
    PlaneMILPlugin
)

set (gui_plugins
    PlaneMILGUIPlugin
)

# Build normal plugins
foreach(plugin ${plugins})
    add_library(${plugin} SHARED ${plugin}.cc)
    target_link_libraries(${plugin} ${GAZEBO_LIBRARIES})
endforeach()

# Build GUI plugins
foreach(gui_plugin ${gui_Plugins})
    add_library(${gui_plugin} SHARED ${headers_MOC} ${gui_plugin}.cc)
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