function [Px, Py, Pz] = Forward_Kinematic_Update(theta1, theta2, theta3)
    % Thông số hình học
    R_base = 60;         % mm
    R_platform = 42.62;  % mm
    rf = 350;            % mm
    re = 150;            % mm

    r = R_base - R_platform;

    % Đổi sang radian
    theta = deg2rad([theta1, theta2, theta3]);
    alpha = [0, 2*pi/3, 4*pi/3];  % góc motor

    % Tính toạ độ 3 khớp động
    P = zeros(3, 3);  % mỗi hàng là 1 điểm (Px, Py, Pz)

    for i = 1:3
        angle = theta(i);
        alpha_i = alpha(i);
        Px = (r + re * cos(angle)) * cos(alpha_i);
        Py = (r + re * cos(angle)) * sin(alpha_i);
        Pz = -re * sin(angle);
        P(i, :) = [Px, Py, Pz];
    end

    % Vector J1J2 và J1J3
    v12 = P(2, :) - P(1, :);
    v13 = P(3, :) - P(1, :);

    % ex = v12 / ||v12||
    ex = v12 / norm(v12);
    a = dot(ex, v13);
    
    % ey = (v13 - a*ex) / norm(...)
    ey = (v13 - a * ex);
    ey = ey / norm(ey);

    % ez = cross(ex, ey)
    ez = cross(ex, ey);

    b = dot(ey, v13);
    d = norm(v12);

    x = d / 2;
    y = (a^2 + b^2)/(2*b) - a*x/b;
    z_sq = rf^2 - x^2 - y^2;


    z = sqrt(z_sq);

    % Tọa độ end-effector trong hệ gốc
    P_ee = P(1, :) + x*ex + y*ey - z*ez;
    Px = P_ee(1)
    Py = P_ee(2)
    Pz = P_ee(3)
end
