cmake_minimum_required(VERSION 3.0.2)
project(aerial_hitter)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
  mavros_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aerial_hitter
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

add_library(rviz_draw include/rviz_draw.h src/traj_prediction/rviz_draw.cpp)
target_link_libraries(rviz_draw ${catkin_LIBRARIES} ${Eigen3_LIBS})

add_library(TrajPredict src/traj_prediction/TrajPredict.h src/traj_prediction/TrajPredict.cpp)
target_link_libraries(TrajPredict ${Eigen3_LIBS})

add_executable(test_predict src/traj_prediction/test_predict.cpp)
target_link_libraries(test_predict  rviz_draw TrajPredict ${catkin_LIBRARIES} ${Eigen3_LIBS})

add_executable(fly_test src/test/test.cpp)
target_link_libraries(fly_test  ${catkin_LIBRARIES})
