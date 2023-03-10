clear; clc; close all;


pt_target = [1.0; 0.5; 0];
r = 0.5;
vel_UAV = 2.4;
vel_yaw = 2.0;
beta = 0.1;
dt = 0.01;

draw = 0;


pt_placed_list = [];
hit_false_num = 0;
hit_in_num = 0;
hit_out_num = 0;

for i = 1:1:1000
    pt_placed = hitterSim(pt_target, vel_UAV, vel_yaw, beta, dt, draw);
    if isempty(pt_placed) == 1
        hit_false_num = hit_false_num + 1;
    else
        pt_placed_list = [pt_placed_list, pt_placed];
        if norm(pt_placed - pt_target) <= r
            hit_in_num = hit_in_num + 1;
        else
            hit_out_num = hit_out_num + 1;
        end
    end
end

hit_in_num/(hit_out_num+hit_in_num+hit_false_num)

h1 = figure(1);
axis equal;
hold on;
grid on;
rectangle('Position',[pt_target(1)-r,pt_target(2)-r,2*r,2*r],'Curvature',[1,1],'EdgeColor','y', 'LineWidth', 3);
rectangle('Position',[pt_target(1)-r*3/4,pt_target(2)-r*3/4,2*r*3/4,2*r*3/4],'Curvature',[1,1],'EdgeColor','g', 'LineWidth', 3);
rectangle('Position',[pt_target(1)-r/2,pt_target(2)-r/2,2*r/2,2*r/2],'Curvature',[1,1],'EdgeColor','b', 'LineWidth', 3);
rectangle('Position',[pt_target(1)-r/4,pt_target(2)-r/4,2*r/4,2*r/4],'Curvature',[1,1],'EdgeColor','r', 'LineWidth', 3);
plot(pt_placed_list(1,:), pt_placed_list(2,:), '.r', 'LineWidth', 1, 'DisplayName', 'hitPoints');


ellipse_pt_list = confidenceEllipse(pt_placed_list(1:2,:), 0.3, 100);
h = plot(ellipse_pt_list(1,:), ellipse_pt_list(2,:), 'r--', 'LineWidth', 1.5, 'DisplayName', '30%');


ellipse_pt_list = confidenceEllipse(pt_placed_list(1:2,:), 0.6, 100);
plot(ellipse_pt_list(1,:), ellipse_pt_list(2,:), 'b--', 'LineWidth', 1.5, 'DisplayName', '60%');

ellipse_pt_list = confidenceEllipse(pt_placed_list(1:2,:), 0.9, 100);
plot(ellipse_pt_list(1,:), ellipse_pt_list(2,:), 'g--', 'LineWidth', 1.5, 'DisplayName', '90%');

legend

h2 = figure(2);
hold on;
X = categorical({'false' 'out' 'in'});
X = reordercats(X,{'false' 'out' 'in'});

b = bar(X, [hit_false_num, hit_out_num, hit_in_num], 0.6);