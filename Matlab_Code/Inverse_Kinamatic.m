function K = Inverse_Kinamatic(X_ee,Y_ee,Z_ee)
    R_Base = 60;
    R_platform = 42.62; % Khoảng cách từ tâm end_effector tới khớp cầu
    r = R_Base - R_platform;   % Khoảng cách từ tâm top tới động cơ
    
    re = 150;       % Chiều dài cánh tay trên (mm)
    rf = 350;       % Chiều dài cánh tay dưới (mm)
    

    % Góc xoay động cơ 
    alpha = [0, 120, 240] * pi / 180;
    
    J = zeros(3, 1);
    threshold = 1e-3;
    for i = 1:3
        A = -2*re*(-r+X_ee*cos(alpha(i))+Y_ee*sin(alpha(i)));
        B = -2*re*Z_ee;
        C = X_ee^2 + Y_ee^2 + Z_ee^2 + r^2 + re^2 - rf^2 - 2*r*(X_ee*cos(alpha(i))+Y_ee*sin(alpha(i)));
        ABC = [A;B;C]
        theta_1 = atan2(B,A) + acos(-C/sqrt(A^2+B^2));
        theta_2 = atan2(B,A) - acos(-C/sqrt(A^2+B^2));
        if theta_1 > theta_2
            theta = theta_2;
        else theta = theta_1;
        end
        % Kiểm tra nếu góc quá nhỏ, đặt theta = 0
        if abs(theta) < threshold
            theta = 0;
        end
        theta = theta*180/pi;
        J(i,1) = -theta;
    end
   K = J(:,:)
end