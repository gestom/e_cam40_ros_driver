cmake_minimum_required(VERSION 2.8.3)
project(e_cam40_ros_driver)

# Enable most warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# Enable C++11 support
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -ggdb")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  camera_info_manager
  image_transport
  cv_bridge
  nodelet
  sensor_msgs
  std_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
        cfg/taraCamera.cfg
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    camera_info_manager
    image_transport
    cv_bridge
    nodelet
    sensor_msgs
    std_msgs
    dynamic_reconfigure
  LIBRARIES tara_camera_nodelet
)

# Dynamic reconfiguration
include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

# Find glib library
find_package(PkgConfig)
pkg_check_modules(GLIB glib-2.0)
include_directories(${GLIB_INCLUDE_DIRS})

include_directories( include ${catkin_INCLUDE_DIRS} )
link_directories( ${catkin_LINK_DIRS} )

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(tara_camera_node src/tara_camera_node.cpp src/tara_camera_driver.cpp src/CameraDevice.cpp src/tara/xunit_lib_tara.cpp src/tara/Tara.cpp)
target_link_libraries(tara_camera_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GLIB_LIBRARIES} udev v4l2)
add_dependencies(tara_camera_node ${PROJECT_NAME}_gencfg)

add_executable(tara_camera_tester src/tara_camera_tester.cpp src/CameraDevice.cpp src/tara/xunit_lib_tara.cpp src/tara/Tara.cpp)
target_link_libraries(tara_camera_tester ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GLIB_LIBRARIES} udev v4l2)
add_dependencies(tara_camera_tester ${PROJECT_NAME}_gencfg)

add_library(tara_camera_nodelet src/tara_camera_nodelet.cpp src/tara_camera_driver.cpp src/CameraDevice.cpp src/tara/xunit_lib_tara.cpp src/tara/Tara.cpp)
target_link_libraries(tara_camera_nodelet ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GLIB_LIBRARIES} udev v4l2)

add_executable(tara_imu_node src/tara_imu_node.cpp src/tara_imu_driver.cpp src/ImuDevice.cpp src/tara/Tara.cpp src/tara/xunit_lib_tara.cpp)
target_link_libraries(tara_imu_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GLIB_LIBRARIES} udev v4l2)

install(TARGETS tara_camera_node tara_camera_nodelet tara_imu_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES tara_camera_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
