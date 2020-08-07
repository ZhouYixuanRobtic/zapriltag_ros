cmake_minimum_required(VERSION 2.8.3)
project(zapriltag_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf2_eigen
  tf
  tf2_geometry_msgs
  tf2_ros message_generation
)

find_package(
        OpenCV REQUIRED
)
find_package(Boost REQUIRED system filesystem date_time thread)


add_message_files(
        FILES
        TagsDetection_msg.msg
        TagDetection_msg.msg
)
generate_messages(
        DEPENDENCIES
        std_msgs  # Or other packages containing msgs
        geometry_msgs
)
catkin_package(
  CATKIN_DEPENDS 
                 cv_bridge
                 std_msgs
                 sensor_msgs
                 image_transport
                 roscpp
                 geometry_msgs
                 tf
                 tf2_ros tf2_eigen tf2_geometry_msgs
                 message_runtime
        DEPENDS
                 EIGEN3
                 Boost
)

include_directories (
        SYSTEM
        ${PROJECT_SOURCE_DIR}/include/
        ${Boost_INCLUDE_DIR} 
        ${EIGEN3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        /usr/local/include/tf
        /usr/local/include/apriltag
        ${catkin_INCLUDE_DIRS}
)
link_directories(
        ${OpenCV_LIB_DIR}
        ${catkin_LIBRARY_DIRS}
        /user/local/lib
)


#libraries

add_library(apriltag_camera src/zapriltag.cpp)
target_link_libraries(apriltag_camera apriltag ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

add_library(apriltag_ros_wrapper src/apriltag_ros_wrapper.cpp)
target_link_libraries(apriltag_ros_wrapper apriltag_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )
#application

add_executable(zapriltag_ros src/main.cpp)
target_link_libraries(zapriltag_ros apriltag_ros_wrapper)
add_dependencies(zapriltag_ros ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

