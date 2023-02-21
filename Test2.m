clc;clear;close all;



vel_s = 0.2;
vel_p = 1;
vel_e = 0;

pt_s = [0.3; vel_s; 0];
pt_p = [1; vel_p; 0.5];
pt_e = [2; vel_e; 1.5];

dt = 0.01;

traj = triangleProfile(pt_s, pt_p, pt_e, dt);



plot(traj(3,:), traj(1,:))