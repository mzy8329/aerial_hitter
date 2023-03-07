function [uav_hit_pos, arm_hit_pos] = hit2base(uav_pos, arm_pos, pt_hit, vel_hit, l0, l1, alpha, beta)
    m1_h = vel_hit(1);
    n1_h = vel_hit(2);
    p1_h = -abs((m1_h^2 + n1_h^2)/vel_hit(3));
    
    e1_h = [m1_h; n1_h; p1_h]/norm([m1_h, n1_h, p1_h]);
    
    pos_arm_1 = atan2(p1_h, sqrt(m1_h^2 + n1_h^2));
    if pos_arm_1 > 0
        pos_arm_1 = pos_arm_1 - 3.14;
        Pt_arm_0_end = l1*e1_h + pt_hit(1:3);
    else
        Pt_arm_0_end = -l1*e1_h + pt_hit(1:3);
    end

    %
     Pt_uav_k = uav_pos;
     tan0 = tan(arm_pos(1));
     p2_k = -sqrt(tan0*tan0/(1+tan0^2));
     p2_h = (alpha*l0*(Pt_arm_0_end(3)-Pt_uav_k(3)) + beta*(p2_k-sqrt(m1_h^2+n1_h^2)))/(alpha*l0^2 + beta);
     
%      n2_h = sqrt((1-p2_h^2)/(1+m1_h^2/n1_h^2));
%      m2_h = n2_h * m1_h/n1_h;
%      e2_h = [m2_h; n2_h; p2_h];
     
     e2_h = [m1_h; n1_h; p2_h]/norm([m1_h, n1_h, p2_h]);
    
     pos_arm_0 = atan2(p2_h, sqrt(m1_h^2 + n1_h^2));
     Pt_base = -l0*e2_h + Pt_arm_0_end;
    
     %
   % p2_h = -1/sqrt(2);
   % n2_h = sqrt((1-p2_h^2)/(1+m1_h^2/n1_h^2));
   % m2_h = n2_h * m1_h/n1_h;
   % e2_h = [m2_h; n2_h; p2_h];
   % pos_arm_0 = -0.785;
   % Pt_base = -l0*e2_h + Pt_arm_0_end;

    uav_hit_pos = [Pt_base; atan2(n1_h,m1_h)];
    arm_hit_pos = [pos_arm_0; pos_arm_1];
end