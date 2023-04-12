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

    std::vector<Eigen::Vector3d> triangleProfile(Eigen::Vector3d pt_start, Eigen::Vector3d pt_end, Eigen::Vector3d pt_pass, double dt)
    {
        double t_start_ceil = ceil(pt_start[2]/dt)*dt;
        double t_end_floor = floor(pt_end[2]/dt)*dt;

        std::vector<Eigen::Vector3d> traj_output;
        Eigen::Vector3d pt_temp;

        traj_output.clear();
        pt_temp[0] = pt_start[0], pt_temp[1] = pt_start[1], pt_temp[2] = t_start_ceil;
        traj_output.push_back(pt_temp);   

        double x_temp, v_temp, t_temp;   

        // 第一段
        double T1 = pt_pass[2] - t_start_ceil;
        double V1 = pt_pass[1] - pt_start[1];
        double X1 = pt_pass[0] - pt_start[0];
        double A1 = T1*T1; double B1 = -4*(X1-pt_start[1]*T1-1/2.0*T1*V1); double C1 = -V1*V1;
        double a1 = (-B1 + sqrt(B1*B1 - 4*A1*C1))/(2*A1);

        int mode = 0;
        if(a1 < 0)
        {
            mode = 1;
            a1 = (-B1 - sqrt(B1*B1 - 4*A1*C1))/(2*A1);
        }
        double t1_s = (T1+V1/a1)/2.0;
        double t1_s_floor = floor(t1_s/dt)*dt;
        double t1_e = (T1-V1/a1)/2.0;
        double t1_e_floor = floor(t1_e/dt)*dt;

        if(t1_s<0 || t1_e < 0)
        {
            if(mode == 0)
            {
                a1 = (-B1 - sqrt(B1*B1 - 4*A1*C1))/(2*A1);
            }
            else
            {
                a1 = (-B1 + sqrt(B1*B1 - 4*A1*C1))/(2*A1);
            }
            
            t1_s = (T1+V1/a1)/2.0;
            t1_s_floor = floor(t1_s/dt)*dt;
            t1_e = (T1-V1/a1)/2.0;
            t1_e_floor = floor(t1_e/dt)*dt;
        }



        for(double t = dt; t <= t1_s_floor; t += dt)
        {
            pt_temp[0] = pt_start[0] + pt_start[1]*t + 1/2.0*a1*t*t;
            pt_temp[1] = pt_start[1] + a1*t;
            pt_temp[2] = t_start_ceil + t;
            traj_output.push_back(pt_temp);            
        }
        x_temp = pt_temp[0], v_temp = pt_temp[1], t_temp = pt_temp[2];
        
        for(double t = dt; t <= t1_e_floor; t += dt)
        {
            pt_temp[0] = x_temp + v_temp*t - 1/2.0*a1*dt*dt;
            pt_temp[1] = v_temp - a1*t;
            pt_temp[2] = t_temp + t;
            traj_output.push_back(pt_temp);
        }

        // 第二段
        double T2 = t_end_floor - pt_pass[2];
        double V2 = pt_end[1] - pt_pass[1];
        double X2 = pt_end[0] - pt_pass[0];
        double A2 = T2*T2; double B2 = -4*(X2-pt_pass[1]*T2-1/2.0*T2*V2); double C2 = -V2*V2;
        double a2 = (-B2 + sqrt(B2*B2 - 4*A2*C2))/(2*A2);

        mode = 0;
        if(a2 < 0)
        {
            mode = 1;
            a2 = (-B2 - sqrt(B2*B2 - 4*A2*C2))/(2*A2);
        }
        double t2_s = (T2+V2/a2)/2.0;
        double t2_s_floor = floor(t2_s/dt)*dt;
        double t2_e = (T2-V2/a2)/2.0;
        double t2_e_floor = floor(t2_e/dt)*dt;


        if(t2_s<0 || t2_e < 0)
        {
            if(mode == 0)
            {
                a2 = (-B2 - sqrt(B2*B2 - 4*A2*C2))/(2*A2);
            }
            else
            {
                a2 = (-B2 + sqrt(B2*B2 - 4*A2*C2))/(2*A2);
            }

            
            t2_s = (T2+V2/a2)/2.0;
            t2_s_floor = floor(t2_s/dt)*dt;
            t2_e = (T2-V2/a2)/2.0;
            t2_e_floor = floor(t2_e/dt)*dt;
        }


        x_temp = pt_temp[0], v_temp = pt_temp[1], t_temp = pt_temp[2];
        for(double t = dt; t <= t2_s_floor; t += dt)
        {
            pt_temp[0] = x_temp + v_temp*t + 1/2.0*a2*t*t;
            pt_temp[1] = v_temp + a2*t;
            pt_temp[2] = t_temp + t;
            traj_output.push_back(pt_temp);
        }


        x_temp = pt_temp[0], v_temp = pt_temp[1], t_temp = pt_temp[2];
        for(double t = dt; t <= t2_e_floor; t += dt)
        {
            pt_temp[0] = x_temp + v_temp*t - 1/2.0*a2*t*t;
            pt_temp[1] = v_temp - a2*t;
            pt_temp[2] = t_temp + t;
            traj_output.push_back(pt_temp);
        }
        return traj_output;
    }

    std::vector<Eigen::Vector3d> triangleProfile(Eigen::Vector3d pt_start, Eigen::Vector3d pt_end, double dt)
    {
        double t_start_ceil = ceil(pt_start[2]/dt)*dt;
        double t_end_floor = floor(pt_end[2]/dt)*dt;

        std::vector<Eigen::Vector3d> traj_output;
        Eigen::Vector3d pt_temp;

        traj_output.clear();
        pt_temp[0] = pt_start[0], pt_temp[1] = pt_start[1], pt_temp[2] = t_start_ceil;
        traj_output.push_back(pt_temp);   

        double x_temp, v_temp, t_temp;   

        double T = t_end_floor - t_start_ceil;
        double X = pt_end[0] - pt_start[0];
        double a1 = (4*X-3*pt_start[1]*T)/(T*T);
        double a2 = (4*X-pt_start[1]*T)/(T*T);

        for(double t = dt; t <= T/2.0; t += dt)
        {
            pt_temp[0] = pt_start[0] + pt_start[1]*t + 1/2.0*a1*t*t;
            pt_temp[1] = pt_start[1] + a1*t;
            pt_temp[2] = t_start_ceil + t;
            traj_output.push_back(pt_temp);            
        }
        x_temp = pt_temp[0], v_temp = pt_temp[1], t_temp = pt_temp[2];
        
        for(double t = dt; t <= T/2.0; t += dt)
        {
            pt_temp[0] = x_temp + v_temp*t - 1/2.0*a2*t*t;
            pt_temp[1] = v_temp - a2*t;
            pt_temp[2] = t_temp + t;
            traj_output.push_back(pt_temp);
        }

        return traj_output;
    }

    void writeFile(char* name, std::vector<Eigen::Vector2d> data, fWrite_mode_e mode)
    {//写入文件
		std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        char file_name[150];
        std::strcpy(file_name, (std::string(name)+"-"+std::ctime(&now_c)+".txt").c_str());

        std::ofstream outfile;      //创建文件
        if(mode == file_new)
        {
            outfile.open(file_name,  std::ios::binary);
        }
        else
        {
            outfile.open(file_name,  std::ios::app|std::ios::binary);
        }

		for(int i = 0; i<data.size(); i++)
		{
			outfile << data[i][0] << "  "
					<< data[i][1] << "  ";
			outfile << std::endl;//保存初始的时间、六个关节角度
		}
		outfile.close();
	}

    void writeFile(char* name, std::vector<Eigen::Vector3d> data, fWrite_mode_e mode)
    {//写入文件
		std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        char file_name[150];
        std::strcpy(file_name, (std::string(name)+"-"+std::ctime(&now_c)+".txt").c_str());

        std::ofstream outfile;      //创建文件
        if(mode == file_new)
        {
            outfile.open(file_name,  std::ios::binary);
        }
        else
        {
            outfile.open(file_name,  std::ios::app|std::ios::binary);
        }

		for(int i = 0; i<data.size(); i++)
		{
			outfile << data[i][0] << "  "
					<< data[i][1] << "  "
					<< data[i][2] << "  ";
			outfile << std::endl;//保存初始的时间、六个关节角度
		}
		outfile.close();
	}

    void writeFile(char* name, std::vector<Eigen::Vector4d> data, fWrite_mode_e mode)
    {//写入文件
        std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        char file_name[150];
        std::strcpy(file_name, (std::string(name)+"-"+std::ctime(&now_c)+".txt").c_str());

        std::ofstream outfile;      //创建文件
        if(mode == file_new)
        {
            outfile.open(file_name,  std::ios::binary);
        }
        else
        {
            outfile.open(file_name,  std::ios::app|std::ios::binary);
        }

		for(int i = 0; i<data.size(); i++)
		{
			outfile << data[i][0] << "  "
					<< data[i][1] << "  "
					<< data[i][2] << "  "
                    << data[i][3] << "  ";
			outfile << std::endl;//保存初始的时间、六个关节角度
		}
		outfile.close();
	}

    void writeFile(char* name, Eigen::VectorXd data, fWrite_mode_e mode)
    {//写入文件
		std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        char file_name[150];
        std::strcpy(file_name, (std::string(name)+"-"+std::ctime(&now_c)+".txt").c_str());

        std::ofstream outfile;      //创建文件
        if(mode == file_new)
        {
            outfile.open(file_name,  std::ios::binary);
        }
        else
        {
            outfile.open(file_name,  std::ios::app|std::ios::binary);
        }

		for(int i = 0; i<data.size(); i++)
		{
			outfile << data[i] << "  ";
		}
        outfile << std::endl;//保存初始的时间、六个关节角度
		outfile.close();
	}

} // namespace common_tools
