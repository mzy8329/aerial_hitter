function pt_placed = hitterSim(pt_tar, vel_uav, vel_yaw, Beta, Dt, ifdraw)
    origin_pose = [0;0;0.5;0];
    l0 = 0.12;
    l1 = 0.18;
    p0_offset = -2.6374;
    p1_offset = 0;

    ball_r = 0.05;
    ball_origin_pose = [2.5+rand(); -0.5+rand(); ball_r/2.0];
    vel_throw = [-3.5+rand(); -0.5+rand(); 4.5+rand()];

    pt_target = pt_tar;
    beta = Beta;

    XLim = [-1, 3.5];
    YLim = [-1.0, 1.5];
    ZLim = [0, 1.5];

    t = 0;
    dt = Dt;

    t_predict = 0.5;
    t_predict_last = 0;
    t_predict_gap = 0.1;

    vel_UAV = vel_uav;
    vel_y_UAV = vel_yaw;

    ball_pose_predict = [0;0;0];

    arm_pose_set = [-2.3, -1.8];
    arm_pose_end = [-0.3, 0];
    arm_time_pass = [0.05, 0.05];


    hit = 0;
    predicted = 0;
    ishit = 0;
    hitfalse = 0;


    pt_hit = [-1; -1; -1; -1];
    vel_hit = [-1; -1; -1; -1];

    uav_hit_pos = [-1; -1; -1; -1; -1];
    arm_hit_pos = [-1; -1; -1];

    arm_hit_pos_before = [-2.3, -1.8];
    arm_hit_pos_after = [-0.2, -0.1];

    arm_0_hit_pos_list = [-2.3; 0; 0];
    arm_1_hit_pos_list = [-1.8; 0; 0];

    ball_traj_hit = [];
    ball_traj_hit_predict = [];
    ball_hit_t_0 = 0;

    ball_hit_pose_0 = [];
    ball_hit_vel_0 = [];

    % begin

    aerialHitter = UAV(origin_pose, l0, l1, p0_offset, p1_offset, XLim, YLim, ZLim, 0);
    aerialHitter_virtual = UAV(origin_pose, l0, l1, p0_offset, p1_offset, XLim, YLim, ZLim, 1);
    ball = Ball(ball_origin_pose, ball_r);


    aerialHitter.updata();
    ball.updata(ball_origin_pose, [0; 0; 0], ball.acc_0, 0, 0);

    if ifdraw == 1
        h1 = figure(1);
        aerialHitter.draw1();
        ball.draw();
        rectangle('Position',[pt_target(1)-0.5,pt_target(2)-0.5,2*0.5,2*0.5],'Curvature',[1,1],'EdgeColor','m');

        pause;
    end

    if ifdraw == 1
        pause(2)

        LineObjects = findall(figure(1),'type','line');
        for i = 1:length(LineObjects);
            h = LineObjects(i);
            delete(h);
        end
    end

    


    while 1
        t = t + dt;
        aerialHitter = aerialHitter.updata();
        if ifdraw == 1
            aerialHitter.draw1();
        end
        
        if ishit == 0
            ball = ball.updata(ball_origin_pose, vel_throw, ball.acc_0, t, 0);
            if ifdraw == 1
                ball.draw();
            end
        else
            ball_new = ball_new.updata(ball_hit_pose_0, ball_hit_vel_0, ball.acc_0, t, ball_hit_t_0);
            if ifdraw == 1
                ball_new.draw();
            end

            if ball_new.pose(3) < 1.1*ball_r
                break
            end
        end

        if ball.pose(3) < 1.1*ball_r && ishit == 0 && t > 0.5
%             pause;
            hitfalse = 2;
        end

        if hitfalse ~= 0
            break
        end

        % 预测与规划
        if(t - t_predict_last > t_predict_gap && hit == 0)
            predicted = 1;

            t_predict_last = t;

            ball_pose_predict = ball_origin_pose + vel_throw*(t+t_predict) + 1/2.0*ball.acc_0*(t+t_predict)^2;
            ball_vel_predict = vel_throw + ball.acc_0*(t+t_predict);

            pt_hit(1:3) = ball_pose_predict;
            vel_hit(1:3) = hitPredict(ball_vel_predict, ball_pose_predict, pt_target, beta);
            if ~isreal(vel_hit)
                hitfalse = 1;
                break
            end
            
            [uav_hit_pos(1:4), arm_hit_pos(1:2)] = hit2base(aerialHitter.pt_base_link, aerialHitter.pos_arm_0, pt_hit, vel_hit, l0, l1, 1, 1);
            pt_hit(4) = t+t_predict; vel_hit(4) = t+t_predict;
            uav_hit_pos(5) = t+t_predict; arm_hit_pos(3) = t+t_predict;

            p = sqrt(vel_hit(1)^2 + vel_hit(2)^2); q = vel_hit(3);
            a = -(l0*sin(arm_hit_pos(1)) + l1*sin(arm_hit_pos(2))); b = -l1*sin(arm_hit_pos(2));
            c = (l0*cos(arm_hit_pos(1)) + l1*cos(arm_hit_pos(2))); d = l1*cos(arm_hit_pos(2));

            arm_hit_pos(1:2);
            arm_twist(2) = (a*q/c - p)/(a*d/c - b);
            arm_twist(1) = (p - b*arm_twist(2))/a;


            arm_0_hit_pos_list = triangleProfile([arm_0_hit_pos_list(1:2,1); pt_hit(4)-arm_time_pass(1)], ...
                                                [arm_hit_pos(1); arm_twist(1); pt_hit(4)], ...
                                                [arm_hit_pos_after(1); 0; pt_hit(4)+arm_time_pass(2)],dt);
            arm_1_hit_pos_list = triangleProfile([arm_1_hit_pos_list(1:2,1); pt_hit(4)-arm_time_pass(1)], ...
                                                [arm_hit_pos(2); arm_twist(2); pt_hit(4)], ...
                                                [arm_hit_pos_after(2); 0; pt_hit(4)+arm_time_pass(2)],dt);

        
%             arm_0_hit_pos_list = acc_vel([arm_0_hit_pos_list(1:2,1); pt_hit(4)-arm_time_pass(1)], ...
%                                                 [arm_hit_pos(1); arm_twist(1); pt_hit(4)], ...
%                                                 [arm_hit_pos_after(1); 0; pt_hit(4)+arm_time_pass(2)],dt);
%             arm_1_hit_pos_list = acc_vel([arm_1_hit_pos_list(1:2,1); pt_hit(4)-arm_time_pass(1)], ...
%                                                 [arm_hit_pos(2); arm_twist(2); pt_hit(4)], ...
%                                                 [arm_hit_pos_after(2); 0; pt_hit(4)+arm_time_pass(2)],dt);


            % a = [arm_0_hit_pos_list(1:2,1); pt_hit(4)-arm_time_pass(1)]
            % b = [arm_hit_pos(1); arm_twist(1); pt_hit(4)]
            % c = [arm_hit_pos_after(1); 0; pt_hit(4)+arm_time_pass(2)]


        
            aerialHitter_virtual = aerialHitter_virtual.set(uav_hit_pos(1:4), arm_hit_pos(1:2)-[aerialHitter_virtual.pos_arm_0_offset; aerialHitter_virtual.pos_arm_1_offset]);
            if ifdraw == 1
                hold on;
                quiver3(pt_hit(1), pt_hit(2), pt_hit(3), vel_hit(1)*0.1, vel_hit(2)*0.1, vel_hit(3)*0.1);
                plot3(ball_pose_predict(1), ball_pose_predict(2), ball_pose_predict(3), 'go', 'LineWidth', 2);
                aerialHitter_virtual.draw2();
                
%                h5 = figure(2);
%                plot(arm_1_hit_pos_list(3,:), arm_1_hit_pos_list(1,:));
% 
%                 pause;
%                close(h5);

                pause(0.1);
            end
            
        end
        
        % 控制
        if(predicted == 1)
            
            if ifdraw == 1
                hold on;
                line([uav_hit_pos(1), aerialHitter.pt_base_link(1)], [uav_hit_pos(2), aerialHitter.pt_base_link(2)], [uav_hit_pos(1), aerialHitter.pt_base_link(1)]);

            end

            n_uav = uav_hit_pos(1:3) - aerialHitter.pose(1:3);
            dist2target = norm(n_uav);
            % 判断是否能到达

            if uav_hit_pos(5)-t > dist2target/vel_UAV
                hit = 1;
            end

            if dist2target < 0.09 && dist2target >= 0.05
                aerialHitter.pose(1:3) = aerialHitter.pose(1:3) + vel_UAV * log10(1+dist2target*100) * n_uav/dist2target*dt;
            elseif dist2target < 0.05
                aerialHitter.pose(1:3) = uav_hit_pos(1:3);
            else
                aerialHitter.pose(1:3) = aerialHitter.pose(1:3) + vel_UAV * n_uav/dist2target*dt;
            end


            d_y = uav_hit_pos(4) - aerialHitter.pose(4); 
            if d_y < 1.0 && d_y >= 0.05
                aerialHitter.pose(4) = aerialHitter.pose(4) + sign(d_y)*vel_y_UAV*log10(1+d_y*100)*dt;
            elseif d_y < 0.05
                aerialHitter.pose(4) = uav_hit_pos(4);
            else
                aerialHitter.pose(4) = aerialHitter.pose(4) + vel_y_UAV*dt;
            end

            if(length(arm_0_hit_pos_list) > 1 && length(arm_1_hit_pos_list) > 1)
                if t-arm_0_hit_pos_list(3,1) > -3*dt
                    aerialHitter.pos_arm_0 = arm_0_hit_pos_list(1,1)-aerialHitter.pos_arm_0_offset;
                    aerialHitter.vel_arm_0 = arm_0_hit_pos_list(2,1);
                    arm_0_hit_pos_list(:,1) = [];


                    aerialHitter.pos_arm_1 = arm_1_hit_pos_list(1,1)-aerialHitter.pos_arm_1_offset;
                    aerialHitter.vel_arm_1 = arm_1_hit_pos_list(2,1);
                    arm_1_hit_pos_list(:,1) = [];
<<<<<<< HEAD

                    if aerialHitter.pos_arm_1 > 0
                        aerialHitter.pos_arm_1 = 0;
                    end
=======
>>>>>>> 4a8fcc9960571167aede700e88c309a6cec9cf79
                end
            end
            
            % 判断是否击中
            pt_0 = aerialHitter.pt_arm_0_end; pt_1 = aerialHitter.pt_arm_1_end; xt = ball.pose;
            d = norm(cross((xt - pt_0), (xt - pt_1)))/norm(pt_0 - pt_1);
            if d < 0.05 && ishit == 0
                ishit = 1;

                theta_0 = aerialHitter.pos_arm_0 + aerialHitter.pos_arm_0_offset;
                thera_1 = aerialHitter.pos_arm_1 + aerialHitter.pos_arm_1_offset;
                vel_0 = aerialHitter.vel_arm_0;
                vel_1 = aerialHitter.vel_arm_1;
                % vel_hit

                yaw = aerialHitter.pose(4);

                v_xy = -(l0*sin(theta_0) + l1*sin(thera_1))*vel_0 - l1*sin(thera_1)*vel_1;
                v_z = (l0*cos(theta_0) + l1*cos(thera_1))*vel_0 + l1*cos(thera_1)*vel_1;
                
                ball_hit_vel_0 = [v_xy*cos(yaw); v_xy*sin(yaw); v_z] * beta;
                ball_hit_pose_0 = ball.pose;
                ball_hit_t_0 = t;
                
                ball_new = Ball(ball.pose, ball_r);
            end

            if ishit == 1
                ball_traj_hit = [ball_traj_hit, ball_hit_pose_0 + ball_hit_vel_0*(t-ball_hit_t_0) + 1/2.0*ball.acc_0*(t-ball_hit_t_0)^2];
                ball_traj_hit_predict = [ball_traj_hit_predict, ball_hit_pose_0 + vel_hit(1:3)*(t-ball_hit_t_0) + 1/2.0*ball.acc_0*(t-ball_hit_t_0)^2];

                if ifdraw == 1
                    hold on;
                    plot3(ball_traj_hit(1,:), ball_traj_hit(2,:), ball_traj_hit(3,:), 'g-.');
                    plot3(ball_traj_hit_predict(1,:), ball_traj_hit_predict(2,:), ball_traj_hit_predict(3,:), 'r:');
                
%                     pause;
                end

            end



        end
        
        
        

        % 删除figure中内容
        if ifdraw == 1
            pause(2*dt);
        end
        

        if ifdraw == 1
            LineObjects = findall(figure(1),'type','line');
            for i = 1:length(LineObjects);
                h = LineObjects(i);
                delete(h);
            end

            QuiverObjects = findall(figure(1),'type','quiver');
            for i = 1:length(QuiverObjects);
                h = QuiverObjects(i);
                delete(h);
            end
        end

    end





    if hitfalse == 0
        pt_placed =  ball_new.pose;
    else
        pt_placed = [];
    end

    if ifdraw == 1
        close(h1);
    end

end