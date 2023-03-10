function bound_points = confidenceEllipse(pt_list, confidence_rate, ellipse_pt_num)
    u = sum(pt_list, 2)/length(pt_list);
    C = cov(pt_list(1,:), pt_list(2,:));

    [P, A] = eig(C);
    c_p = sqrt(2*log(1/(1-confidence_rate)));

    a = sqrt(A(1,1))*c_p; b = sqrt(A(2,2))*c_p; 


    bound_points = [];
    pt_temp = [];

    half_num = (ellipse_pt_num-mod(ellipse_pt_num, 2));
    dx = 2*a/half_num;
    for i = 0:half_num
        x = -a + i*dx;
        pt_temp = [x; -b/a*sqrt(a^2-x^2)];
        bound_points = [bound_points, pt_temp];
    end

    for i = 1:half_num
        x = a - i*dx;
        pt_temp = [x; b/a*sqrt(a^2-x^2)];
        bound_points = [bound_points, pt_temp];
    end

    bound_points = transpose(P)*bound_points + u;

end