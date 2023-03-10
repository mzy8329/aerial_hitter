close all; clc; clear;


g = 9.8;
V = 7;


a = 1:0.01:10;


h_list = [];
a_list = [];



for h = 0:0.01:2
    X = a./(1+a.^2)./g.*(V.^2 + V.*sqrt((1+a.^2).*2.*g.*h));
    for i = 1:901
        if X(i+1)<X(i)
            a_list = [a_list, a(i)];
            break
        end
    end

    h_list = [h_list, h];
end


angle_list = 90 - atan(a_list) * 180/3.14;

p = polyfit(h_list, angle_list, 3)

plot( h_list, angle_list, 'LineWidth', 1.5 ,'DisplayName', '∂(h)')
legend;


H = 1.76;
angle_best = p*[H^3; H^2; H; 1]
