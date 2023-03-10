function traj_out = triangleProfile(pt_s, pt_p, pt_e, dt)
    t_s_ceil = ceil(pt_s(3)/dt)*dt;
    t_e_floor = floor(pt_e(3)/dt)*dt;

    vel_s = pt_s(2); vel_p = pt_p(2); vel_e = pt_e(2);


    %第一段
    pt_temp = [pt_s(1); pt_s(2); t_s_ceil];
    traj_out = [pt_s, pt_temp];


    mode = 0;
    T1 = pt_p(3) - t_s_ceil;
    V1 = vel_p - vel_s;
    X1 = pt_p(1) - pt_s(1);
    A = T1^2; B = -4*(X1-vel_s*T1-1/2.0*T1*V1); C = -V1^2;
    a1 = (-B + sqrt(B^2 - 4*A*C))/(2*A);

    if a1 < 0
        a1 = (-B - sqrt(B^2 - 4*A*C))/(2*A);
        mode = 1;
    end

    t1_s = (T1+V1/a1)/2.0;
    t1_s_floor = floor(t1_s/dt)*dt;
    t1_e = (T1-V1/a1)/2.0;
    t1_e_floor = floor(t1_e/dt)*dt;

    
    if t1_s < 0 || t1_e < 0
        if mode ==0
            a1 = (-B - sqrt(B^2 - 4*A*C))/(2*A);
        else
            a1 = (-B + sqrt(B^2 - 4*A*C))/(2*A);
        end

        t1_s = (T1+V1/a1)/2.0;
        t1_s_floor = floor(t1_s/dt)*dt;
        t1_e = (T1-V1/a1)/2.0;
        t1_e_floor = floor(t1_e/dt)*dt;
    end
    

    x_temp = pt_temp(1); v_temp = pt_temp(2); t_temp = pt_temp(3);
    for t = dt:dt:t1_s_floor
        pt_temp(1) = x_temp + vel_s*t + 1/2.0*a1*t^2;
        pt_temp(2) = vel_s + a1*t;
        pt_temp(3) = t_temp + t;
        traj_out = [traj_out, pt_temp];
    end

    x_temp = pt_temp(1); v_temp = pt_temp(2); t_temp = pt_temp(3);
    for t = dt:dt:t1_e_floor
        pt_temp(1) = x_temp + (vel_s + t1_s_floor*a1)*t - 1/2.0*a1*t^2;
        pt_temp(2) = (vel_s + t1_s_floor*a1) - a1*t;
        pt_temp(3) = t_temp + t;
        traj_out = [traj_out, pt_temp];
    end
    
    %第二段
    mode = 0;
    T2 = t_e_floor - pt_p(3);
    X2 = pt_e(1) - pt_p(1);
    V2 = vel_e - vel_p;
    A = T2^2; B = -4*(X2-vel_p*T2-1/2.0*T2*V2); C = -V2^2;
    a2 = (-B + sqrt(B^2 - 4*A*C))/(2*A);

    if a2 < 0
        a2 = (-B - sqrt(B^2 - 4*A*C))/(2*A);
        mode = 1;
    end


    t2_s = (T2+V2/a2)/2.0;
    t2_s_floor = floor(t2_s/dt)*dt;
    t2_e = (T2-V2/a2)/2.0;
    t2_e_floor = floor(t2_e/dt)*dt;


    if t2_s < 0 || t2_e < 0
        if mode ==0
            a2 = (-B - sqrt(B^2 - 4*A*C))/(2*A);
        else
            a2 = (-B + sqrt(B^2 - 4*A*C))/(2*A);
        end
        t2_s = (T2+V2/a2)/2.0;
        t2_s_floor = floor(t2_s/dt)*dt;
        t2_e = (T2-V2/a2)/2.0;
        t2_e_floor = floor(t2_e/dt)*dt;
    end


    x_temp = pt_temp(1); v_temp = pt_temp(2); t_temp = pt_temp(3);
    for t= dt:dt:t2_s_floor
        pt_temp(1) = x_temp + vel_p*t + 1/2.0*a2*t^2;
        pt_temp(2) = vel_p + a2*t;
        pt_temp(3) = t_temp + t;
        traj_out = [traj_out, pt_temp];
    end

    x_temp = pt_temp(1); v_temp = pt_temp(2); t_temp = pt_temp(3);
    for t = dt:dt:t2_e_floor
        pt_temp(1) = x_temp + (vel_p +t2_s_floor*a2)*t - 1/2.0*a2*t^2;
        pt_temp(2) = vel_p + t2_s_floor*a2 - a2*t;
        pt_temp(3) = t_temp + t;
        traj_out = [traj_out, pt_temp];
    end

end 