#include "TrajPredict.h"





void TrajPredict::init(int fit_len, int check_len, double freeFallCheck_err, double beta)
{
    _fit_len = fit_len;
    _check_len = check_len;
    _freeFallCheck_err = freeFallCheck_err;  
    _beta = beta;  
}

void TrajPredict::pushNewPoint(Eigen::Vector4d point)
{
    if(_pose_list.empty())
    {
        _pose_list.push_back(point);
        _pose_list.push_back(point);
    }
    else
    {
        _pose_list.push_back(point);
    }

    Eigen::Vector4d vel;
    if(_vel_list.empty())
    {
        vel << 0, 0, 0, _pose_list.back()[3];
    }
    else
    {
        double dT = _pose_list.back()[3] - _pose_list[_pose_list.size()-2][3];
        vel[3] = _pose_list.back()[3];
        for(int i = 0; i < 3; i++)
        {
            vel[i] = (_pose_list.back()[i] - _pose_list[_pose_list.size()-2][i])/dT;
        }
    }
    _vel_list.push_back(vel);
}

bool TrajPredict::predictTraj_gOnly(double time, int size)
{
    if(_pose_list.size() <= _fit_len) return false;
    if(!freeFallCheck()) return false;
    double T_begin = _pose_list[_pose_list.size()-_fit_len-1][3];
    Eigen::MatrixXd phi;
    Eigen::MatrixXd Y[3];
    phi.resize(_fit_len, 3);
    Y[0].resize(_fit_len,1);
    Y[1].resize(_fit_len,1);
    Y[2].resize(_fit_len,1);
    for(int i = 0; i < _fit_len; i++)
    {
        double t = _pose_list[_pose_list.size()-_fit_len+i][3]-T_begin;
        phi.row(i) << 1, t, 1/2.0*t*t;
        Y[0].row(i) << _pose_list[_pose_list.size()-_fit_len+i][0];
        Y[1].row(i) << _pose_list[_pose_list.size()-_fit_len+i][1];
        Y[2].row(i) << _pose_list[_pose_list.size()-_fit_len+i][2];
    }

    Eigen::MatrixXd theta[3];
    for(int i = 0; i < 3; i++)
    {
        theta[i] = (phi.transpose()*phi).inverse()*phi.transpose()*Y[i];
    }

    double T_now = _pose_list.back()[3];
    double dT = time/(double)size;
    _traj_predict.clear();
    Eigen::MatrixXd X(1,3);
    Eigen::Vector4d point;

    for(double t = T_now-T_begin; t < T_now-T_begin + time; t+=dT)
    {
        X << 1, t, 1/2.0*t*t;
        point << (X*theta[0])(0), (X*theta[1])(0), (X*theta[2])(0), t+T_begin;
        _traj_predict.push_back(point);
    }
    return true;
}

bool TrajPredict::fitKd(int fitKd_len)
{
    if(_vel_list.size() <= fitKd_len) return false;
    if(!freeFallCheck()) return false;

    double T_begin = _pose_list[_pose_list.size()-fitKd_len-1][3];
    Eigen::MatrixXd phi;
    Eigen::MatrixXd Y;
    phi.resize(fitKd_len*3, 3);
    Y.resize(fitKd_len*3,1);


    for(int i = 0; i < fitKd_len; i++)
    {
        double t = _pose_list[_pose_list.size()-fitKd_len+i][3]-T_begin;
        double Vx = _vel_list[_vel_list.size()-fitKd_len+i][0];
        double Vy = _vel_list[_vel_list.size()-fitKd_len+i][1];
        double Vz = _vel_list[_vel_list.size()-fitKd_len+i][2];
        double V = sqrt(Vx*Vx + Vy*Vy + Vz*Vz);


        phi.row(i*3) << 1, t, -1/2.0*t*t*V*Vx;
        phi.row(i*3+1) << 1, t, -1/2.0*t*t*V*Vy;
        phi.row(i*3+2) << 1, t, -1/2.0*t*t*V*Vz;
        Y.row(i*3) << _pose_list[_pose_list.size()-_fit_len+i][0];
        Y.row(i*3+1) << _pose_list[_pose_list.size()-_fit_len+i][1];
        Y.row(i*3+2) << _pose_list[_pose_list.size()-_fit_len+i][2] + 1/2.0*9.8*t*t;
    }

    Eigen::MatrixXd theta = (phi.transpose()*phi).inverse()*phi.transpose()*Y;
    _Kd_list.push_back(theta(2));
    return true;
}

bool TrajPredict::predictTraj_hit(int index_trajPredict, Eigen::Vector3d point_target)
{
    if(_traj_predict.empty()) return false;
    double X = point_target[0] - _traj_predict[index_trajPredict][0];
    double Y = point_target[1] - _traj_predict[index_trajPredict][1];
    double Z = point_target[2] - _traj_predict[index_trajPredict][2];
    
    double B_temp = Z + sqrt(X*X+Y*Y);
    if(B_temp < 0) return false;
    double t = sqrt(2.0/9.8*B_temp);
    double v_x = X/t, v_y = Y/t, v_z = (Z+1/2.0*9.8*t*t)/t;

    _point_hit[0] = _traj_predict[index_trajPredict][0];
    _point_hit[1] = _traj_predict[index_trajPredict][1];
    _point_hit[2] = _traj_predict[index_trajPredict][2];
    _point_hit[6] = _traj_predict[index_trajPredict][3];
    double dT_temp = _traj_predict[index_trajPredict][3] - _traj_predict[index_trajPredict-1][3];
    Eigen::Vector3d vel_before = {
        (_traj_predict[index_trajPredict][0] - _traj_predict[index_trajPredict-1][0])/dT_temp,
        (_traj_predict[index_trajPredict][1] - _traj_predict[index_trajPredict-1][1])/dT_temp,
        (_traj_predict[index_trajPredict][2] - _traj_predict[index_trajPredict-1][2])/dT_temp
    };
    Eigen::Vector3d vel_after = {v_x, v_y, v_z};
    Eigen::Vector3d vel_hit = collisionModel(vel_before, vel_after);
    _point_hit[3] = vel_hit[0];
    _point_hit[4] = vel_hit[1];
    _point_hit[5] = vel_hit[2];

    Eigen::Vector4d pt;
    _traj_hit_predict.clear();
    for(double t_p = 0; t_p < t; t_p+=t/10.0)
    {
        pt[0] = _traj_predict[index_trajPredict][0] + v_x * t_p;
        pt[1] = _traj_predict[index_trajPredict][1] + v_y * t_p;
        pt[2] = _traj_predict[index_trajPredict][2] + v_z * t_p - 1/2.0*9.8*t_p*t_p;
        pt[3] = _traj_predict[index_trajPredict][3];
        _traj_hit_predict.push_back(pt);
    }
    return true;
}

Eigen::Vector3d TrajPredict::collisionModel(Eigen::Vector3d vel_before, Eigen::Vector3d vel_after)
{
    Eigen::Vector3d n = (vel_before - vel_after)/(vel_before - vel_after).norm();
    double Vb = vel_before.transpose()*n - 1/(1+_beta)*(vel_before - vel_after).norm();
    return Vb*n;
}




bool TrajPredict::freeFallCheck()
{
    if(_vel_list.size() < _check_len)
    {
        // std::cout << "too little points" << std::endl;
        return false;
    }
    else
    {
        double T_begin = _pose_list[_pose_list.size()-_check_len-1][3];

        // 求位置和速度的最小二乘估计
        Eigen::MatrixXd phi_pose;
        Eigen::MatrixXd y_pose;
        Eigen::MatrixXd phi_vel;
        Eigen::MatrixXd y_vel;
        phi_pose.resize(_check_len, 3);
        y_pose.resize(_check_len,1);
        phi_vel.resize(_check_len, 2);
        y_vel.resize(_check_len,1);

        for(int i = 0; i < _check_len; i++)
        {
            double t = _pose_list[_pose_list.size()-_check_len+i][3]-T_begin;

            phi_pose.row(i) << 1, t, 1/2.0*t*t;
            y_pose.row(i) << _pose_list[_pose_list.size()-_check_len+i][2];

            phi_vel.row(i) << 1, t;
            y_vel.row(i) << _vel_list[_vel_list.size()-_check_len+i][2];
        }
        Eigen::MatrixXd theta_pose = (phi_pose.transpose()*phi_pose).inverse()*phi_pose.transpose()*y_pose;
        Eigen::MatrixXd theta_vel = (phi_vel.transpose()*phi_vel).inverse()*phi_vel.transpose()*y_vel;

        // std::cout <<theta_pose(1)<<"  "<< theta_pose(2)<< std::endl;
        if(abs((theta_pose(2)+theta_pose(1))/2.0+9.8)<_freeFallCheck_err)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
   
}


double* TrajPredict::getKd()
{
    _Kd_avg_var[0] = 0, _Kd_avg_var[1] = 0;
    for(int i = 0; i < _Kd_list.size(); i++)
    {
        _Kd_avg_var[0] += _Kd_list[i]/(float)_Kd_list.size();
    }


    for(int i = 0; i < _Kd_list.size(); i++)
    {
        _Kd_avg_var[1] += (_Kd_list[i]-_Kd_avg_var[0])*(_Kd_list[i]-_Kd_avg_var[0]);
    }
    return _Kd_avg_var;
}

