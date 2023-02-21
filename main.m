clear; clc; close;

origin_pose = [0;0;0.5;0];
l0 = 0.12;
l1 = 0.18;
p0_offset = -2.6374;
p1_offset = 0;

ball_r = 0.1
ball_origin_pose = [4; 0; ball_r/2.0];
vel_throw = [-3; 0; 6];

XLim = [-1, 5];
YLim = [-1, 1];
ZLim = [0, 2];

t = 0;
dt = 0.01

t_predict = 0.5;
t_predict_last = 0;
t_predict_gap = 0.1;

vel_UAV = 1.0;

ball_pose_predict = [0;0;0];

arm_pose_set = [-2.3, -1.8];
arm_pose_end = [-0.3, 0];
arm_time_pass = [0.5, 0.5];


% begin

aerialHitter = UAV(origin_pose, l0, l0, p0_offset, p1_offset, XLim, YLim, ZLim);
ball = Ball(ball_origin_pose, ball_r);


aerialHitter.updata_draw();
ball.updata_draw(ball_origin_pose, [0; 0; 0], ball.acc_0, 0, 0);
pause(5)

LineObjects = findall(figure(1),'type','line');
for i = 1:length(LineObjects);
    h = LineObjects(i);
    delete(h);
end


while 1
    t = t + dt;
    if(t - t_predict_last > t_predict_gap)
        t_predict_last = t;

        ball_pose_predict = ball_origin_pose + vel_throw*(t+t_predict) + 1/2.0*ball.acc_0*(t+t_predict)^2;
        plot3(ball_pose_predict(1), ball_pose_predict(2), ball_pose_predict(3), 'go', 'LineWidth', 2);
    end
    

    

    
    aerialHitter.updata_draw();
    ball.updata_draw(ball_origin_pose, vel_throw, ball.acc_0, t, 0);
    pause(5*dt);

    LineObjects = findall(figure(1),'type','line');
    for i = 1:length(LineObjects);
        h = LineObjects(i);
        delete(h);
    end
end