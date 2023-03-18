#! /bin/zsh

rosservice call gazebo/delete_model "model_name: 'redball'" & sleep 2

roslaunch aerial_hitter throw_ball.launch & sleep 6

rosservice call /throw_ball/throw "{}"