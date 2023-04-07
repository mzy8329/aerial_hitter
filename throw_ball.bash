#! /bin/zsh
rosservice call /UAV_sim/hitingAllow true

rosservice call gazebo/delete_model "model_name: 'redball'" & sleep 1

rosrun gazebo_ros spawn_model -sdf -file src/aerial_hitter/models/redball/model.sdf -model redball -x 2.0 -y 0 -z 0.05 & sleep 1

rosservice call /throw_ball/throw "{}"