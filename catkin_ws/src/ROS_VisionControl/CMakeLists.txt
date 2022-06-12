cmake_minimum_required(VERSION 3.3.0)

message("Building ImFusion OrienAdj")
project(markerless_RUSS)

include(cmake/Macros.cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ImFusionLib REQUIRED)
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

foreach(plugin ${Qt5Network_PLUGINS})
  get_target_property(_loc ${plugin} LOCATION)
  message("Plugin ${plugin} is at location ${_loc}")
endforeach()

find_package(catkin REQUIRED
    iiwa_msgs
    iiwa_ros
    roscpp
    rospy
    sensor_msgs
    visualization_msgs
    std_msgs
    image_transport
    cv_bridge
    geometry_msgs
    tf2
    tf2_ros
    )
find_package(OpenIGTLink REQUIRED)

#set(OpenCV_DIR "/usr/local/lib")
find_package( OpenCV REQUIRED) # locate OpenCV in system #JZL
#include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "This is OpenCV INCLUDE DIRS" ${OpenCV_INCLUDE_DIRS})
message(STATUS "This is OpenCV LIBS DIRS" ${OpenCV_LIBS})

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  iiwa_msgs
  iiwa_ros
  roscpp
  rospy
  std_msgs
  sensor_msgs
  visualization_msgs
  image_transport
  cv_bridge
  geometry_msgs
  DEPENDS OpenIGTLink
  CFG_EXTRAS definitions.cmake
)

include("cmake/definitions.cmake")

file(GLOB PLUGIN_HEADERS "include/*.h" "include/vision_control/*.h")

set(PLUGIN_SOURCES
  src/MonitorChangeThread.cpp
  src/VisionPlugin.cpp
  src/VisionFactory.cpp
  src/VisionAlgorithm.cpp
  src/VisionController.cpp
  src/VisionStreamOptimizer.cpp
  src/ConfidentMapConvex.cpp
  src/controller.ui
)


#ADD_EXECUTABLE(adj_pro ${PLUGIN_SOURCES})

add_library(${PROJECT_NAME} SHARED ${PLUGIN_SOURCES} ${PLUGIN_FORMS} ${PLUGIN_HEADERS})

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${IMFUSION_SDK_INCLUDE}
  ${OpenIGTLink_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}
#  ${OpenCV_INCLUDE_DIRS}    #########JZL
  )

message(${Qt5Widgets_INCLUDE_DIRS})

target_link_libraries (${PROJECT_NAME}

        ${OpenCV_LIBS}   #####JZL
        ImFusionLib
        ImFusionStream
        OpenIGTLink
        ImFusionUS
        ${catkin_LIBRARIES}
        Qt5::Widgets

)


# Define output target directories
#imfusion_set_common_target_properties()


###### INSTALLATION
include(GNUInstallDirs)

install(TARGETS ${PROJECT_NAME}
  EXPORT "${PROJECT_NAME}Targets"
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
