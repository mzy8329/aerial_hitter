source /home/mzy/Code/workSpace/PX4-Autopilot/Tools/simulation/gazebo/setup_gazebo.bash /home/mzy/Code/workSpace/PX4-Autopilot /home/mzy/Code/workSpace/PX4-Autopilot/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/mzy/Code/workSpace/PX4-Autopilot:/home/mzy/Code/workSpace/PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/mzy/Code/workSpace/UAV_Hitter_ws/src/aerial_hitter/models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/mzy/Code/workSpace/UAV_Hitter_ws/devel/lib
