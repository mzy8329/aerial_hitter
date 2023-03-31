#include "UAV.h"


UAV::UAV(ros::NodeHandle nh, double ctrl_freq, double UAV_vel)
{
    _nh = nh;
    _ctrl_rate = ctrl_freq;
    _UAV_Vel = UAV_vel;

    _mode = wait;

    initParam(_nh);

    _state_sub = nh.subscribe("/mavros/state", 10, &UAV::state_Callback, this);
    _target_pose_sub = nh.subscribe("/UAV/target_pose", 1, &UAV::targetPose_Callback, this);
    _current_pose_sub = nh.subscribe("/UAV/pose", 1, &UAV::currentPose_Callback, this);
    _ballPoseVrpn_sub = nh.subscribe("/vrpn_client_node/RigidBody3/pose", 10, &UAV::ballPoseVrpnCallBack, this);
    _ballPoseGazebo_sub = nh.subscribe("/ball_odom", 10, &UAV::ballPoseGazeboCallBack, this);

    _local_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);
    _local_traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/mavros/setpoint_trajectory/local", 1);
    _rviz_marker_pub = nh.advertise<visualization_msgs::Marker>("/traj_view", 1);
    _hitPoint_pub = nh.advertise<nav_msgs::Odometry>("/UAV/hitPoint", 1);


    _set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    _arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    _landing_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/land");

    _pose_hover.pose.position.x = 0;
    _pose_hover.pose.position.y = 0;
    _pose_hover.pose.position.z = 1.5;
    tf::Quaternion q = tf::createQuaternionFromYaw(0);
    _pose_hover.pose.orientation.x = q.getX();
    _pose_hover.pose.orientation.y = q.getY();
    _pose_hover.pose.orientation.z = q.getZ();
    _pose_hover.pose.orientation.w = q.getW();

    _base_pose.resize(7,1);
    _hit_pose.resize(7,1);    

    _Arm.init(nh, _ctrl_rate);
    _isSet = false;

    _Predict.trajPredict.init(_Predict.fit_len, _Predict.check_len, _Predict.freeFallCheck_err, _Predict.beta);
}


void UAV::state_Callback(const mavros_msgs::StateConstPtr &msg)
{
    _current_state = *msg;
}

void UAV::currentPose_Callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    _currentPose = *msg;
}

void UAV::targetPose_Callback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr &msg)
{
    _targetPose = *msg;
}

void UAV::ballPoseVrpnCallBack(const geometry_msgs::PoseStampedConstPtr &body_msg)
{
    static int i_1 = 0;
    static int i_2 = 0;
    Eigen::Vector4d point;
    point << body_msg->pose.position.x, body_msg->pose.position.y, body_msg->pose.position.z, body_msg->header.stamp.toSec();

    if(i_2++ > 3)
    {
        _Rviz.mark = rviz_draw::draw(point, _Rviz.colar_pose, "traj_view", i_1++);
        _rviz_marker_pub.publish(_Rviz.mark);

        i_2 = 0;
    }
    
    _Predict.trajPredict.pushNewPoint(point);
}

void UAV::ballPoseGazeboCallBack(const nav_msgs::OdometryConstPtr &body_msg)
{
    static int i_1 = 0;
    static int i_2 = 0;
    Eigen::Vector4d point;
    point << body_msg->pose.pose.position.x, body_msg->pose.pose.position.y, body_msg->pose.pose.position.z, body_msg->header.stamp.toSec();

    if(i_2++ > 3)
    {
        _Rviz.mark = rviz_draw::draw(point, _Rviz.colar_pose, "traj_view", i_1++);
        _rviz_marker_pub.publish(_Rviz.mark);

        i_2 = 0;
        if(i_1 > 100)
        {
            i_1 = 0;
        }
    }
    
    _Predict.trajPredict.pushNewPoint(point);
}


void UAV::initParam(ros::NodeHandle nh)
{
    nh.getParam("/traj_prediction/fit_len", _Predict.fit_len);
    nh.getParam("/traj_prediction/check_len", _Predict.check_len);
    nh.getParam("/traj_prediction/fitKd_len", _Predict.fitKd_len);

    nh.getParam("/traj_prediction/freeFallCheck_err", _Predict.freeFallCheck_err);
    nh.getParam("/traj_prediction/pre_time", _Predict.pre_time);
    nh.getParam("/traj_prediction/pre_size", _Predict.pre_size);
    nh.getParam("/traj_prediction/beta", _Predict.beta);
}

void UAV::setArmParam(double* arm_length, double* arm_offset, Eigen::Vector3d axis2link, Eigen::Vector3d arm2base, double* arm_start, double* arm_end, double* arm_time_pass)
{
    _arm_length[0] = arm_length[0], _arm_length[1] = arm_length[1];
    _arm_offset[0] = arm_offset[0], _arm_offset[1] = arm_offset[1];

    _axis2link = axis2link;
    _arm2base = arm2base;

    _arm_start[0] = arm_start[0], _arm_start[1] = arm_start[1];
    _arm_end[0] = arm_end[0], _arm_end[1] = arm_end[1];
    _arm_time_pass[0] = arm_time_pass[0], _arm_time_pass[1] = arm_time_pass[1];

    for(int i = 0; i < 2; i++)
    {
        _arm_pos_target[i].clear();
        Eigen::Vector3d pt_temp;
        pt_temp << _arm_start[i], 0, 0;
        _arm_pos_target[i].push_back(pt_temp);
    }
}



void UAV::takeOff()
{
    static int i = 0;
    if(i++ < 100)
    {
        _local_pose_pub.publish(_pose_hover);
    }
    
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    static ros::Time last_request = ros::Time::now();

    if( _current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(3.0))){
        if( _set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
    } else {
        if( !_current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(3.0))){
            if( _arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
    }

    _pose_hover.header.stamp = ros::Time::now();
    _local_pose_pub.publish(_pose_hover);


    static int cnt = 0;
    if(abs(_currentPose.pose.position.z - _pose_hover.pose.position.z) < 0.2)
    {
        cnt++;
    }
    if(cnt > 100)
    {
        _mode = hover;
        cnt = 0;
    }
}

void UAV::Hover()
{
    _pose_hover.header.stamp = ros::Time::now();
    _local_pose_pub.publish(_pose_hover);
    

    if(!_isSet)
    {
        _targetTraj_xyz[0].clear();
        _targetTraj_xyz[1].clear();
        _targetTraj_xyz[2].clear();
        _targetPose.points.clear();

        for(int i = 0; i < 2; i++)
        {
            _arm_pos_target[i].clear();
            _arm_pos_target[i].push_back(Eigen::Vector3d({_arm_start[i], 0, 0}));
        }
        _hit_pose_temp = {0, 0, 0, 0};

        _isSet = _Arm.GetSet();
    }
    else
    {
        if(_Predict.trajPredict.freeFallCheck())
        {
            _mode = move;
        }
    }

}

void UAV::Move()
{
    if(_Predict.trajPredict.predictTraj_gOnly(_Predict.pre_time, _Predict.pre_size))
    {
        std::vector<Eigen::Vector4d> traj_predict = _Predict.trajPredict.getTraj();
        
        _Rviz.mark = rviz_draw::draw(traj_predict, _Rviz.colar_traj, "traj_predict");
        _rviz_marker_pub.publish(_Rviz.mark);

        // 计算临时击球点
        int k = 0;
        double dist_min = 1000000000;
        Eigen::Vector3d pose_now = {_currentPose.pose.position.x, _currentPose.pose.position.y, _currentPose.pose.position.z};
        Eigen::Vector3d pose_temp;
        for(int i = 0; i < traj_predict.size(); i++)
        {
            pose_temp = {traj_predict[i][0], traj_predict[i][1], traj_predict[i][2]};
            double dist = 0.4*abs(pose_now[0]-pose_temp[0]) + 0.4*abs(pose_now[1]-pose_temp[1]) + 0.2*abs(pose_now[0]-pose_temp[0]);
            if(dist < dist_min 
            && (_hit_pose_temp.isZero() || _hit_pose_temp[3] == 0 || (pose_temp-Eigen::Vector3d({_hit_pose_temp[0], _hit_pose_temp[1], _hit_pose_temp[2]})).norm()< 2.0)
            // && (pose_temp[0] > _SafeBox.x_lim[0] && pose_temp[0] < _SafeBox.x_lim[1])
            // && (pose_temp[1] > _SafeBox.y_lim[0] && pose_temp[1] < _SafeBox.y_lim[1])
            // && (pose_temp[2] > _SafeBox.z_lim[0] && pose_temp[2] < _SafeBox.z_lim[1])
            )
            {
                dist_min = dist;
                k = i;
            }
        }
        _hit_pose_temp = checkSafeBox(traj_predict[k]);


        // 检验临时击球点是否可选取为击球点
        Eigen::Vector3d hit_pose_temp = {traj_predict[k][0], traj_predict[k][1], traj_predict[k][2]};
        if((hit_pose_temp-pose_now).norm() < (_hit_pose_temp[3]-ros::Time::now().toSec())*_UAV_Vel
        && _Predict.trajPredict.predictTraj_hit(k, _point_target))
        {
            _Rviz.mark = rviz_draw::draw(_Predict.trajPredict.getTraj_hit(), _Rviz.colar_hit, "traj_hit");
            _rviz_marker_pub.publish(_Rviz.mark);
            
            Eigen::VectorXd hitPoint = _Predict.trajPredict.getPoint_hit();
            _Rviz.mark = rviz_draw::draw(hitPoint, _Rviz.colar_hitPoint, "traj_hitPoint");
            _rviz_marker_pub.publish(_Rviz.mark);

            if(isnan(hitPoint[3]) || isnan(hitPoint[4]) || isnan(hitPoint[5])
            || hitPoint[2] < 0.5
            )
            {
                ;;
            }
            else
            {
                // 规划无人机轨迹
                _hit_pose = hitPoint;
                hit2base();

                for(int i = 0; i < 3; i++)
                {
                    Eigen::Vector3d pt_s;
                    Eigen::Vector3d pt_e;
                    
                    if(_targetPose.points.empty())
                    {
                        pt_s[0] = pose_now[i];
                        pt_s[1] = 0;
                    }
                    else
                    {
                        switch (i)
                        {
                        case 0:
                            pt_s[0] = _targetPose.points[0].transforms[0].translation.x;
                            pt_s[1] = _targetPose.points[0].velocities[0].linear.x;
                            break;
                        case 1:
                            pt_s[0] = _targetPose.points[0].transforms[0].translation.y;
                            pt_s[1] = _targetPose.points[0].velocities[0].linear.y;
                            break;
                        case 2:
                            pt_s[0] = _targetPose.points[0].transforms[0].translation.z;
                            pt_s[1] = _targetPose.points[0].velocities[0].linear.z;
                            break;
                        default:
                            break;
                        }
                    }
                    pt_s[2] = ros::Time::now().toSec();

                    pt_e = {_base_pose[i], 0, ros::Time::now().toSec()+0.5*(_hit_pose_temp[3]-ros::Time::now().toSec())};

                    _targetTraj_xyz[i] = common_tools::triangleProfile(pt_s, pt_e, 2.0/_ctrl_rate);
                }

                // 求解无人机状态
                

                double p = sqrt(_hit_pose[3]*_hit_pose[3] + _hit_pose[4]*_hit_pose[4]);
                double q = _hit_pose[5];
                double a = -(_arm_length[0]*sin(_arm_hit_pos[0])+_arm_length[1]*sin(_arm_hit_pos[1]));
                double b = -_arm_length[1]*sin(_arm_hit_pos[1]);
                double c = (_arm_length[0]*cos(_arm_hit_pos[0])+_arm_length[1]*cos(_arm_hit_pos[1]));
                double d = _arm_length[1]*cos(_arm_hit_pos[1]);

                double temp = (a*q/c-p)/(a*d/c-b);
                double vel_arm[2] = {(p-b*temp)/a, temp};

                double t_1 = std::min(2*abs((_arm_hit_pos[0]-_arm_pos_target[0][0][0])/(_arm_pos_target[0][0][1] + vel_arm[0])),
                                      2*abs((_arm_hit_pos[1]-_arm_pos_target[1][0][0])/(_arm_pos_target[1][0][1] + vel_arm[1])));
                double t_2 = std::min(2*abs((_arm_end[0]-_arm_hit_pos[0])/vel_arm[0]),
                                      2*abs((_arm_end[1]-_arm_hit_pos[1])/vel_arm[1]));
    
                for(int i = 0; i < 2; i++)
                {
                    Eigen::Vector3d P_s = {_arm_pos_target[i][0][0],  _arm_pos_target[i][0][1], _hit_pose[6]-t_2-t_1};
                    Eigen::Vector3d P_p = {_arm_hit_pos[i], vel_arm[i], _hit_pose[6]-t_2};
                    Eigen::Vector3d P_e = {_arm_end[i], 0, _hit_pose[6]};
                    _arm_pos_target[i] =  common_tools::triangleProfile(P_s, P_e, P_p, 1.0/_ctrl_rate);
                }   
                _mode = hit;
            }            
        }
        else
        {
            // 规划无人机轨迹
            for(int i = 0; i < 3; i++)
            {
                Eigen::Vector3d pt_s;
                Eigen::Vector3d pt_e;
                
                if(_targetPose.points.empty())
                {
                    pt_s[0] = pose_now[i];
                    pt_s[1] = 0;
                }
                else
                {
                    switch (i)
                    {
                    case 0:
                        pt_s[0] = _targetPose.points[0].transforms[0].translation.x;
                        pt_s[1] = _targetPose.points[0].velocities[0].linear.x;
                        break;
                    case 1:
                        pt_s[0] = _targetPose.points[0].transforms[0].translation.y;
                        pt_s[1] = _targetPose.points[0].velocities[0].linear.y;
                        break;
                    case 2:
                        pt_s[0] = _targetPose.points[0].transforms[0].translation.z;
                        pt_s[1] = _targetPose.points[0].velocities[0].linear.z;
                        break;
                    default:
                        break;
                    }
                }
                pt_s[2] = ros::Time::now().toSec();

                pt_e = {(_hit_pose_temp[i]+pose_now[i])/2.0, 0, ros::Time::now().toSec()+(_hit_pose_temp[3]-ros::Time::now().toSec())};

                _targetTraj_xyz[i] = common_tools::triangleProfile(pt_s, pt_e, 2.0/_ctrl_rate);
            }
        }


    }

    if(_mode == move && !_targetTraj_xyz[0].empty())
    {
        _targetPose.header.stamp = ros::Time::now();

        trajectory_msgs::MultiDOFJointTrajectoryPoint temp_point;
        geometry_msgs::Transform temp_pos;
        geometry_msgs::Twist temp_vel;

        _targetTraj_xyz[0][0] = checkSafeBox(_targetTraj_xyz[0][0]);
        _targetTraj_xyz[1][0] = checkSafeBox(_targetTraj_xyz[1][0]);
        _targetTraj_xyz[2][0] = checkSafeBox(_targetTraj_xyz[2][0]);

        temp_pos.translation.x = _targetTraj_xyz[0][0][0];
        temp_pos.translation.y = _targetTraj_xyz[1][0][0];
        temp_pos.translation.z = _targetTraj_xyz[2][0][0];
        // std::cout<<temp_pos.translation.x<<" "<<temp_pos.translation.y<<" "<<temp_pos.translation.z<<std::endl;

        tf::Quaternion q = tf::createQuaternionFromYaw(0);
        temp_pos.rotation.x = q.getX();
        temp_pos.rotation.y = q.getY();
        temp_pos.rotation.z = q.getZ();
        temp_pos.rotation.w = q.getW();

        temp_vel.linear.x = _targetTraj_xyz[0][0][1];
        temp_vel.linear.y = _targetTraj_xyz[1][0][1];
        temp_vel.linear.z = _targetTraj_xyz[2][0][1];

        temp_point.transforms.clear();
        temp_point.velocities.clear();
        temp_point.transforms.push_back(temp_pos);
        temp_point.velocities.push_back(temp_vel);
        temp_point.time_from_start = ros::Duration(1.0/_ctrl_rate);
        _targetPose.points.clear();
        _targetPose.points.push_back(temp_point);

        _local_traj_pub.publish(_targetPose);


        // char name_1[] = "/home/mzy/Code/workSpace/UAV_Hitter_ws/src/aerial_hitter/data/targetTraj_x.txt";
        // char name_2[] = "/home/mzy/Code/workSpace/UAV_Hitter_ws/src/aerial_hitter/data/targetTraj_y.txt";
        // char name_3[] = "/home/mzy/Code/workSpace/UAV_Hitter_ws/src/aerial_hitter/data/targetTraj_z.txt";

        // common_tools::writeFile(name_1, _targetTraj_xyz[0]);
        // common_tools::writeFile(name_2, _targetTraj_xyz[1]);
        // common_tools::writeFile(name_3, _targetTraj_xyz[2]);


        if(_targetTraj_xyz[0].size()>1)
        {
            _targetTraj_xyz[0].erase(_targetTraj_xyz[0].begin());
            _targetTraj_xyz[1].erase(_targetTraj_xyz[1].begin());
            _targetTraj_xyz[2].erase(_targetTraj_xyz[2].begin());
        }
        else
        {
            _targetTraj_xyz[0][0][1] = 0;
            _targetTraj_xyz[1][0][1] = 0;
            _targetTraj_xyz[2][0][1] = 0;
        }

        if(ros::Time::now().toSec()-_hit_pose_temp[3]>2)
        {
            _mode = hover;
        }
    }

}

void UAV::Hit()
{    
    if(!_targetTraj_xyz[0].empty())
    {
        _targetPose.header.stamp = ros::Time::now();

        trajectory_msgs::MultiDOFJointTrajectoryPoint temp_point;
        geometry_msgs::Transform temp_pos;
        geometry_msgs::Twist temp_vel;

        _targetTraj_xyz[0][0] = checkSafeBox(_targetTraj_xyz[0][0]);
        _targetTraj_xyz[1][0] = checkSafeBox(_targetTraj_xyz[1][0]);
        _targetTraj_xyz[2][0] = checkSafeBox(_targetTraj_xyz[2][0]);

        temp_pos.translation.x = _targetTraj_xyz[0][0][0];
        temp_pos.translation.y = _targetTraj_xyz[1][0][0];
        temp_pos.translation.z = _targetTraj_xyz[2][0][0];

        tf::Quaternion q = tf::createQuaternionFromYaw(_base_pose[5]);
        temp_pos.rotation.x = q.getX();
        temp_pos.rotation.y = q.getY();
        temp_pos.rotation.z = q.getZ();
        temp_pos.rotation.w = q.getW();

        temp_vel.linear.x = _targetTraj_xyz[0][0][1];
        temp_vel.linear.y = _targetTraj_xyz[1][0][1];
        temp_vel.linear.z = _targetTraj_xyz[2][0][1];

        temp_point.transforms.clear();
        temp_point.velocities.clear();
        temp_point.transforms.push_back(temp_pos);
        temp_point.velocities.push_back(temp_vel);
        temp_point.time_from_start = ros::Duration(1.0/_ctrl_rate);
        _targetPose.points.clear();
        _targetPose.points.push_back(temp_point);

        _local_traj_pub.publish(_targetPose);


        if(_targetTraj_xyz[0].size()>1)
        {
            _targetTraj_xyz[0].erase(_targetTraj_xyz[0].begin());
            _targetTraj_xyz[1].erase(_targetTraj_xyz[1].begin());
            _targetTraj_xyz[2].erase(_targetTraj_xyz[2].begin());
        }
        else
        {
            _targetTraj_xyz[0][0][1] = 0;
            _targetTraj_xyz[1][0][1] = 0;
            _targetTraj_xyz[2][0][1] = 0;
        }

        nav_msgs::Odometry odom_hitPt;
        odom_hitPt.header.frame_id = "map";
        odom_hitPt.child_frame_id = "hitPoint";
        odom_hitPt.pose.pose.position.x = temp_pos.translation.x;
        odom_hitPt.pose.pose.position.y = temp_pos.translation.y;
        odom_hitPt.pose.pose.position.z = temp_pos.translation.z;
        odom_hitPt.pose.pose.orientation.x = q.getX();
        odom_hitPt.pose.pose.orientation.y = q.getY();
        odom_hitPt.pose.pose.orientation.z = q.getZ();
        odom_hitPt.pose.pose.orientation.w = q.getW();
        _hitPoint_pub.publish(odom_hitPt);
        
    }


    static bool hit_start = false;
    double time_now = ros::Time::now().toSec();
    // std::cout<< time_now <<"  " <<_arm_pos_targetca[0][0][2] <<"  " << _arm_pos_target[0].size() <<  std::endl;
    
    if(time_now - _arm_pos_target[0][0][2] > 1.0/_ctrl_rate)
    {

        hit_start = true;
        double pos[2] = {_arm_end[0] - _arm_offset[0], _arm_end[1] - _arm_offset[1]};

        if(_arm_pos_target[0].size() > 0)
        {
            if(_arm_pos_target[0][0][0]>0)
            {
                _arm_pos_target[0][0][0] = 0;
            }

            pos[0] = _arm_pos_target[0][0][0] - _arm_offset[0];
            _arm_pos_target[0].erase(_arm_pos_target[0].begin());
            // std::cout << _arm_pos_target[0][0][0] << " " << _arm_offset[0] << " ";
        } 

        if(_arm_pos_target[1].size() > 0)
        {
            if(_arm_pos_target[1][0][0]>0)
            {
                _arm_pos_target[1][0][0] = 0;
            }

            pos[1] = _arm_pos_target[1][0][0] - _arm_offset[1];
            _arm_pos_target[1].erase(_arm_pos_target[1].begin());
        }
        // std::cout << pos[0] << " " << pos[1] << std::endl;
        _Arm.ctrlArm(pos[0], pos[1]);
    }
    

    if(_arm_pos_target[0].size() <= 0 && _arm_pos_target[1].size() <= 0 && hit_start == true)
    {
        hit_start = false;

        _isSet = false;
        if(ros::Time::now().toSec()-_hit_pose[6]>3)
        {
            _mode = hover;
        }
    }
}


void UAV::printData()
{
    switch (_mode)
    {
    case wait:
        std::cout<<"wait..."<<std::endl;
        break;
    case take_off:
        std::cout<<"take_off"<<std::endl;
        break;
    case hover:
        std::cout<<"hover"<<std::endl;
        break;
    case move:
        std::cout<<"move"<<std::endl;
        break;
    case hit:
        std::cout<<"hit"<<std::endl;
        break;
    case land:
        std::cout<<"land"<<std::endl;
        break;
    case manual:
        std::cout<<"mannual"<<std::endl;
        break;
    default:
        break;
    }
}

void UAV::hit2base()
{
    double a = 0.5;
    double b = 1 - a;    

    Eigen::Vector3d Pt = {_hit_pose[0], _hit_pose[1], _hit_pose[2]};
    double m1_h = _hit_pose[3];
    double n1_h = _hit_pose[4];
    double p1_h = -(_hit_pose[3]*_hit_pose[3] + _hit_pose[4]*_hit_pose[4])/_hit_pose[5];
    Eigen::Vector3d e1_h = {m1_h/sqrt(m1_h*m1_h+n1_h*n1_h+p1_h*p1_h), n1_h/sqrt(m1_h*m1_h+n1_h*n1_h+p1_h*p1_h), p1_h/sqrt(m1_h*m1_h+n1_h*n1_h+p1_h*p1_h)};
    Eigen::Vector3d Pa_h;
    _arm_hit_pos[1] = atan2(p1_h, sqrt(m1_h*m1_h+n1_h*n1_h));
    if(_arm_hit_pos[1] > 0)
    {
        _arm_hit_pos[1] -= 3.14;
        Pa_h = _arm_length[1]*e1_h + Pt;
    }
    else
    {
        Pa_h = -_arm_length[1]*e1_h + Pt;
    }


    Eigen::Vector3d P_base = {_currentPose.pose.position.x, _currentPose.pose.position.y, _currentPose.pose.position.z};
    Eigen::Vector3d Pb_k = P_base + _arm2base + _axis2link;
    double tan0 = tan(_Arm._arm_current_pos[0] + _arm_offset[0]);
    double p2_k = -sqrt(tan0*tan0/(1+tan0*tan0));
    double p2 = (a*_arm_length[0]*(Pa_h[2]-Pb_k[2]) + b*(p2_k-sqrt(m1_h*m1_h+n1_h*n1_h)))/(a*_arm_length[0]*_arm_length[0] + b);
    Eigen::Vector3d e2_h = {m1_h, n1_h, p2};
    e2_h.normalize();
    Eigen::Vector3d Pb_h = -_arm_length[0]*e2_h + Pa_h;
    _arm_hit_pos[0] = atan2(p2, sqrt(m1_h*m1_h+n1_h*n1_h));

    // double p2_h = -1/sqrt(2);
    // double n2_h = sqrt(1-p2_h*p2_h)/(1+(m1_h*m1_h)/(n1_h*n1_h));
    // double m2_h = n2_h * m1_h/n1_h;
    // Eigen::Vector3d e2_h = {m2_h, n2_h, p2_h};
    // _arm_hit_pos[0] = -0.785;
    // Eigen::Vector3d Pb_h = -_arm_length[0]*e2_h + Pa_h;
        
    
    Eigen::Vector3d P_base_h = Pb_h - _arm2base - _axis2link;
    _base_pose[0] = P_base_h[0], _base_pose[1] = P_base_h[1], _base_pose[2] = P_base_h[2]; 
    _base_pose[3] = 0, _base_pose[4] = 0, _base_pose[5] = atan2(n1_h, m1_h);
    _base_pose[6] = _hit_pose[6];

    // std::cout<<Pt[0]<<" "<<Pt[1]<<" "<<Pt[2]<<std::endl;
    // std::cout<<Pa_h[0]<<" "<<Pa_h[1]<<" "<<Pa_h[2]<<std::endl;
    // std::cout<<Pb_h[0]<<" "<<Pb_h[1]<<" "<<Pb_h[2]<<std::endl;
    // std::cout<<_base_pose[0]<<" "<<_base_pose[1]<<" "<<_base_pose[2]<<std::endl<<std::endl;
    
}

template<typename T>
T UAV::checkSafeBox(T pt)
{
    if(pt[0] < _SafeBox.x_lim[0]) pt[0] = _SafeBox.x_lim[0];
    if(pt[0] > _SafeBox.x_lim[1]) pt[0] = _SafeBox.x_lim[1];
    if(pt[1] < _SafeBox.y_lim[0]) pt[1] = _SafeBox.y_lim[0];
    if(pt[1] > _SafeBox.y_lim[1]) pt[1] = _SafeBox.y_lim[1];
    if(pt[2] < _SafeBox.z_lim[0]) pt[2] = _SafeBox.z_lim[0];
    if(pt[2] > _SafeBox.z_lim[1]) pt[2] = _SafeBox.z_lim[1];

    return pt;
}


void UAV::mainLoop()
{
    ros::Rate loop_rate(_ctrl_rate);

    int i = 0;
    while(ros::ok())
    {
        switch (_mode)
        {
        case wait:
            if(_current_state.connected)
            {
                _mode = take_off;
            }
            break;
        
        case take_off:
            takeOff();
            break;

        case hover:
            Hover();
            break;
        
        case move:
            Move();
            break;

        case hit:
            Hit();
            break;

        default:
            break;
        }

        if(i++ > 50)
        {
            printData();
            i = 0;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    
}