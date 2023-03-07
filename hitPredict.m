function vel_hit = hitPredict(vel_before, pos_hit, pos_target, beta)
    X = pos_target(1) - pos_hit(1);
    Y = pos_target(2) - pos_hit(2);
    Z = pos_target(3) - pos_hit(3);


    angle = [-2.0942    8.7797  -15.1488   42.7971] * [pos_hit(3)^3; pos_hit(3)^2; pos_hit(3); 1] *3.14/180;

    t = sqrt(2/9.8*((sqrt(X^2+Y^2)*tan(angle)-Z)));

    vel_after = [X/t; Y/t; (-Z+1/2.0/9.8*t^2)/t];

    n = (vel_before + vel_after)/norm(vel_before + vel_after);
    if transpose(vel_before)*n > 0
        Vb = transpose(vel_before)*n + 1/(1+beta)*norm(vel_before - vel_after);
    else
        Vb = -transpose(vel_before)*n + 1/(1+beta)*norm(vel_before - vel_after);
    end

    vel_hit = Vb*n;
end