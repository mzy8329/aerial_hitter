cmake_minimum_required(VERSION 3.0.2)
project(aerial_hitter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  visualization_msgs
  mavros_msgs
  std_msgs
  nav_msgs
  gazebo_msgs
  trajectory_msgs
  tf
  std_srvs
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
  ../../devel/include/
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# plugin
add_library(joint_controller SHARED plugins/joint_controller.cc)
target_link_libraries(joint_controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


# lib
add_library(rviz_draw include/rviz_draw.h src/rviz_draw.cpp)
target_link_libraries(rviz_draw ${catkin_LIBRARIES} ${Eigen3_LIBS})

add_library(TrajPredict include/TrajPredict.h src/TrajPredict.cpp)
target_link_libraries(TrajPredict ${Eigen3_LIBS})

add_library(common_tools include/common_tools.h src/common_tools.cpp)
target_link_libraries(common_tools ${catkin_LIBRARIES})

add_library(AerialArm_sim include/AerialArm_sim.h src/AerialArm_sim.cpp)
target_link_libraries(AerialArm_sim common_tools ${catkin_LIBRARIES})

add_library(UAV_sim include/UAV_sim.h src/UAV_sim.cpp)
target_link_libraries(UAV_sim common_tools AerialArm_sim TrajPredict rviz_draw ${catkin_LIBRARIES})

add_library(AerialArm_inkind include/AerialArm_inkind.h src/AerialArm_inkind.cpp)
target_link_libraries(AerialArm_inkind common_tools ${catkin_LIBRARIES})

add_library(UAV_inkind include/UAV_inkind.h src/UAV_inkind.cpp)
target_link_libraries(UAV_inkind common_tools AerialArm_inkind TrajPredict rviz_draw ${catkin_LIBRARIES})



# node
add_executable(test_predict src/traj_prediction/test_predict.cpp)
target_link_libraries(test_predict  rviz_draw TrajPredict ${catkin_LIBRARIES} ${Eigen3_LIBS})

add_executable(take_off src/fly_test/take_off.cpp)
target_link_libraries(take_off ${catkin_LIBRARIES})

add_executable(get_gazebo_pose src/node/simulation/get_gazebo_pose.cpp)
target_link_libraries(get_gazebo_pose ${catkin_LIBRARIES})

add_executable(arm_test src/node/simulation/arm_test.cpp)
target_link_libraries(arm_test AerialArm_sim ${catkin_LIBRARIES} ${Eigen3_LIBS})

add_executable(throw_ball src/node/simulation/throw_ball.cpp)
target_link_libraries(throw_ball AerialArm_sim ${catkin_LIBRARIES})

add_executable(aerial_hitter_main_ctrl_sim src/node/simulation/aerial_hitter_main_ctrl_sim.cpp)
target_link_libraries(aerial_hitter_main_ctrl_sim AerialArm_sim UAV_sim ${catkin_LIBRARIES})

add_executable(aerial_hitter_main_ctrl_exp src/node/physical_experiment/aerial_hitter_main_ctrl_exp.cpp)
target_link_libraries(aerial_hitter_main_ctrl_exp AerialArm_inkind UAV_inkind ${catkin_LIBRARIES})

add_executable(cur_offset src/node/physical_experiment/cur_offset.cpp)
target_link_libraries(cur_offset ${catkin_LIBRARIES})

add_executable(pos_offset src/node/physical_experiment/pos_offset.cpp)
target_link_libraries(pos_offset ${catkin_LIBRARIES})