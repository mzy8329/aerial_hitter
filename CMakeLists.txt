cmake_minimum_required(VERSION 3.0.2)
project(aerial_hitter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  visualization_msgs
  mavros_msgs
  std_msgs
  gazebo_msgs
  trajectory_msgs
  tf
)
find_package(gazebo REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aerial_hitter
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# plugin
add_library(joint_controller SHARED plugins/joint_controller.cc)
target_link_libraries(joint_controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


# lib
add_library(rviz_draw include/rviz_draw.h src/traj_prediction/rviz_draw.cpp)
target_link_libraries(rviz_draw ${catkin_LIBRARIES} ${Eigen3_LIBS})

add_library(TrajPredict src/traj_prediction/TrajPredict.h src/traj_prediction/TrajPredict.cpp)
target_link_libraries(TrajPredict ${Eigen3_LIBS})

add_library(AerialArm include/AerialArm.h src/AerialArm.cpp)
target_link_libraries(AerialArm ${catkin_LIBRARIES})


# node
# predict
add_executable(test_predict src/traj_prediction/test_predict.cpp)
target_link_libraries(test_predict  rviz_draw TrajPredict ${catkin_LIBRARIES} ${Eigen3_LIBS})

add_executable(take_off src/fly_test/take_off.cpp)
target_link_libraries(take_off ${catkin_LIBRARIES})

add_executable(get_gazebo_pose src/get_gazebo_pose.cpp)
target_link_libraries(get_gazebo_pose ${catkin_LIBRARIES})

add_executable(arm_test src/node/arm_test.cpp)
target_link_libraries(arm_test AerialArm ${catkin_LIBRARIES})

add_executable(arm_test_zero src/node/arm_test_zero.cpp)
target_link_libraries(arm_test_zero AerialArm ${catkin_LIBRARIES})