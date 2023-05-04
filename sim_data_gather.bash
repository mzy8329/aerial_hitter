#! /bin/zsh

for i in num{1..10}
do
roslaunch aerial_hitter aerial_hitter_sitl.launch & sleep 30

./src/aerial_hitter/throw_ball.bash & sleep 5

rosservice call /throw_ball/throw "{}"

sleep 5

pkill gz
pkill roscore

rosnode kill /aerial_hitter_main_ctrl_sim
rosnode kill /gazebo
rosnode kill /get_gazebo_pose
rosnode kill /mavros
rosnode kill /rosout
rosnode kill /rviz
rosnode kill /throw_ball
rosnode kill /trajHit2map
rosnode kill /trajHitPoint2map
rosnode kill /trajPredict2map
rosnode kill /trajView2map

pkill gz
rosnode kill --all
sleep 2s


done