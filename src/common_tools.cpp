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
        double t_pass_floor = floor(pt_pass[1]*CTRL_FREQ)/CTRL_FREQ;
        double t_pass_ceil = ceil(pt_pass[1]*CTRL_FREQ)/CTRL_FREQ;
        double t_start_ceil = ceil(pt_start[1]*CTRL_FREQ)/CTRL_FREQ;
        double t_end_floor = floor(pt_end[1]*CTRL_FREQ)/CTRL_FREQ;

        std::vector<Eigen::Vector2d> traj_output;
        Eigen::Vector2d pt_temp;

        double a_sp = vel_pass/(t_pass_floor - t_start_ceil);
        double a_pe = -vel_pass/(t_end_floor - t_pass_ceil);

        traj_output.push_back(pt_start);
        pt_temp[0] = pt_start[0], pt_temp[1] = t_start_ceil;
        traj_output.push_back(pt_temp);        
        for(double dt = 1/CTRL_FREQ; dt += 1/CTRL_FREQ; dt+t_start_ceil < t_pass_floor)
        {
            pt_temp[0] += 1/2.0*a_sp*dt*dt, pt_temp[1] += dt;
            traj_output.push_back(pt_temp);
        }

        pt_temp[0] += vel_pass/CTRL_FREQ, pt_temp[1] = t_pass_ceil;
        traj_output.push_back(pt_temp);
        for(double dt = 1/CTRL_FREQ; dt += 1/CTRL_FREQ; dt+t_pass_ceil < t_end_floor)
        {
            pt_temp[0] += 1/2.0*a_pe*dt*dt, pt_temp[1] += dt;
            traj_output.push_back(pt_temp);
        }
        traj_output.push_back(pt_end);

        return traj_output;
    }


} // namespace common_tools
