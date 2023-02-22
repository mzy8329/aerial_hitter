function vel_hit = hitPredict(vel_before, pos_hit, pos_target, beta)
    X = pos_target(1) - pos_hit(1);
    Y = pos_target(2) - pos_hit(2);
    Z = pos_target(3) - pos_hit(3);

    t = sqrt(2/9.8*((sqrt(X^2+Y^2)-Z)));

    vel_after = [X/t; Y/t; (Z+1/2.0/9.8*t^2)/t];

    n = (vel_before - vel_after)/norm(vel_before - vel_after);
    Vb = transpose(vel_before)*n - 1/(1+beta)*norm(vel_before - vel_after);

    vel_hit = Vb*n;
end