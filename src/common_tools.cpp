#include "common_tools.h"



namespace common_tools
{
    Eigen::Vector3d quaternion2Euler(Eigen::Vector4d q)
    {
        double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
        Eigen::Vector3d Euler;
        Euler[0] = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
        Euler[1] = asin(2*(q0*q2-q1*q3));
        Euler[2] = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
        return Euler;
    }

    Eigen::Vector4d Euler2quaternion(Eigen::Vector3d Euler)
    {
        Eigen::Vector4d q;
        double r = Euler[0], p = Euler[1], y = Euler[2];
        q[0] = cos(r/2.0)*cos(p/2.0)*cos(y/2.0) + sin(r/2.0)*sin(p/2.0)*sin(y/2.0);
        q[1] = sin(r/2.0)*cos(p/2.0)*cos(y/2.0) - cos(r/2.0)*sin(p/2.0)*sin(y/2.0);
        q[2] = cos(r/2.0)*sin(p/2.0)*cos(y/2.0) + sin(r/2.0)*cos(p/2.0)*sin(y/2.0);
        q[3] = cos(r/2.0)*cos(p/2.0)*sin(y/2.0) - sin(r/2.0)*sin(p/2.0)*cos(y/2.0);
        return q;
    }

    Eigen::Vector3d orientation2Euler(geometry_msgs::Quaternion q)
    {
        Eigen::Vector4d Q;
        Q[0] = q.w, Q[1] = q.x, Q[2] = q.y, Q[3] = q.z;
        return quaternion2Euler(Q);
    }

    geometry_msgs::Quaternion Euler2orientation(Eigen::Vector3d Euler)
    {
        Eigen::Vector4d q = Euler2quaternion(Euler);
        geometry_msgs::Quaternion Q;
        Q.w = q[0], Q.x = q[1], Q.y = q[2], Q.z = q[3];
        return Q;
    }

    std::vector<Eigen::Vector2d> triangleProfile(Eigen::Vector2d pt_start, Eigen::Vector2d pt_end, Eigen::Vector2d pt_pass, double vel_pass)
    {
        // double t_pass_floor = floor(pt_pass[1]*CTRL_FREQ)/CTRL_FREQ;
        // double t_pass_ceil = ceil(pt_pass[1]*CTRL_FREQ)/CTRL_FREQ;
        double t_start_ceil = ceil(pt_start[1]*CTRL_FREQ)/CTRL_FREQ;
        double t_end_floor = floor(pt_end[1]*CTRL_FREQ)/CTRL_FREQ;

        std::vector<Eigen::Vector2d> traj_output;
        Eigen::Vector2d pt_temp;

        traj_output.clear();
        traj_output.push_back(pt_start);
        pt_temp[0] = pt_start[0], pt_temp[1] = t_start_ceil;
        traj_output.push_back(pt_temp);      

        // 第一段
        double T1 = pt_pass[1] - t_start_ceil;
        double V = vel_pass;
        double X1 = pt_pass[0] - pt_start[0];
        double a1 = ((4*X1 - 2*V*T1) + sqrt((2*V*T1-4*X1)*(2*V*T1-4*X1)+4*T1*T1*V*V))/(2*T1*T1);
        double t1_s = (T1+V/a1)/2.0;
        double t1_s_floor = floor(t1_s*CTRL_FREQ)/CTRL_FREQ;
        double t1_e = (T1-V/a1)/2.0;
        double t1_e_floor = floor(t1_e*CTRL_FREQ)/CTRL_FREQ;

        for(double dt = 1/CTRL_FREQ; dt <= t1_s_floor; dt += 1/CTRL_FREQ)
        {
            pt_temp[0] = pt_start[0] + 1/2.0*a1*dt*dt;
            pt_temp[1] = t_start_ceil + dt;
            traj_output.push_back(pt_temp);            
        }
        double x_temp = pt_temp[0];
        double t_temp = pt_temp[1];
        
        for(double dt = 1/CTRL_FREQ; dt <= t1_e_floor; dt += 1/CTRL_FREQ)
        {
            pt_temp[0] = x_temp + t1_s*a1*dt - 1/2.0*a1*dt*dt;
            pt_temp[1] = t_temp + dt;
            traj_output.push_back(pt_temp);
        }

        // 第二段
        double T2 = t_end_floor - pt_pass[1];
        double X2 = pt_end[0] - pt_pass[0];
        double a2 = ((6*T2*V-4*X2)+sqrt((6*T2*V-4*X2)*(6*T2*V-4*X2) + 4*V*V*T2*T2))/(2*T2*T2);
        double t2_s = (T2-V/a2)/2.0;
        double t2_s_floor = floor(t2_s*CTRL_FREQ)/CTRL_FREQ;
        double t2_e = (T2+V/a2)/2.0;
        double t2_e_floor = floor(t2_e*CTRL_FREQ)/CTRL_FREQ; 

        x_temp = pt_temp[0];
        t_temp = pt_temp[1];
        for(double dt = 1/CTRL_FREQ; dt <= t2_s_floor; dt += 1/CTRL_FREQ)
        {
            pt_temp[0] = x_temp + V*dt + 1/2.0*a2*dt*dt;
            pt_temp[1] = t_temp + dt;
            traj_output.push_back(pt_temp);
        }

        x_temp = pt_temp[0];
        t_temp = pt_temp[1];
        for(double dt = 1/CTRL_FREQ; dt <= t2_s_floor; dt += 1/CTRL_FREQ)
        {
            pt_temp[0] = x_temp + t2_s*a2*dt - 1/2.0*a2*dt*dt;
            pt_temp[1] = t_temp + dt;
            traj_output.push_back(pt_temp);
        }

        pt_temp[0] = pt_end[0], pt_temp[1] = t_end_floor;
        traj_output.push_back(pt_temp);  
        traj_output.push_back(pt_end);

        return traj_output;
    }


} // namespace common_tools
