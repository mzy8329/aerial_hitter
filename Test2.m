clc;clear;close all;

%% 

vel_s = 2.2052;
vel_p = 2.2808;
vel_e = 0;
pt_s = [-1.86; vel_s; 0.51];
pt_p = [-0.785; vel_p; 1.01];
pt_e = [-0.2; vel_e; 1.51];
dt = 0.01;

traj = triangleProfile(pt_s, pt_p, pt_e, dt);

plot(traj(3,:), traj(1,:))


%%

% pos_hit = [0; 0; 0.5];
% pos_target = [1; 1; 0];
% vel_before = [-0.1; -0.2; -0.5];
% beta = 0.3;

% vel_hit = hitPredict(vel_before, pos_hit, pos_target, beta)