function [Px,Py,Pz] = Forward_Kinamatic(theta1, theta2, theta3)
    % Các thông số hình học của robot Delta

    R_Base = 60 ;  % Khoảng cách từ tâm top tới động cơ
    R_platform = 42.62; % Khoảng cách từ tâm end_effector tới khớp cầu
    r = R_Base-R_platform;

    re = 150;       % Chiều dài cánh tay trên (mm)
    rf = 350;       % Chiều dài cánh tay dưới (mm)
    theta_in = [theta1, theta2, theta3] * pi / 180;
    
    % Góc xoay động cơ 
    alpha_base = [0, 120, 240] * pi / 180;
    
    % Tính toán vị trí của các khớp di động
    P = zeros(3, 3);
    for i = 1:3
        Px = r + re * cos(theta_in(i));
        Py = 0;
        Pz = -re * sin(theta_in(i));
        K = [Px; Py; Pz];
        P(i, :) = Rz(alpha_base(i)) * K;
    end
    P_Original = P
    P_origin = -P(1, 1:3)';

    motor_positions = zeros(3,3);
    for i = 1:3
        motor_positions(i,1) = P_origin(1) + (r+R_platform) * cos(alpha_base(i));
        motor_positions(i,2) = P_origin(2) + (r+R_platform) * sin(alpha_base(i));
        motor_positions(i,3) = P_origin(3); % Động cơ nằm trên cùng một mặt phẳng
    end   
    
    % Đặt J1 làm gốc tọa độ
    P = P - P(1, :)

    % Tính toán hệ trục tọa độ
    J1_J3 = [P(3,1)-P(1,1); P(3,2)-P(1,2); P(3,3)-P(1,3)];
    J3_J1 = [P(1,1)-P(3,1); P(1,2)-P(3,2); P(1,3)-P(3,3)];
    J1_J2 = [P(2,1)-P(1,1); P(2,2)-P(1,2); P(2,3)-P(1,3)];
    
    ex_hat = J1_J2 / norm(J1_J2)
    a = dot(ex_hat, J1_J3);
    ey_hat = (J3_J1 + a * ex_hat) / norm(J3_J1 + a * ex_hat)
    % ey_hat = [(J3_J1(1)+a*ex_hat(1))/sqrt((J3_J1(1)+a*ex_hat(1))^2+(J3_J1(2)+a*ex_hat(2))^2) ; ...
    %            (J3_J1(2)+a*ex_hat(2))/sqrt((J3_J1(1)+a*ex_hat(1))^2+(J3_J1(2)+a*ex_hat(2))^2);0]
    ez_hat = cross(ex_hat, ey_hat)
    
    % Tính toán vị trí end-effector
    b = dot(ey_hat, J1_J3);
    d = norm(J1_J2);
    x = d / 2;
    y = (a^2 + b^2) / (2*b) - a*x/b;
    z = sqrt(rf^2 - x^2 - y^2);
    
    % end_effector = x * ex_hat + y * ey_hat + z * ez_hat
    P_0_ee = [P_Original(1)+x*ex_hat(1)+y*ey_hat(1)+z*ez_hat(1);...
                    P_Original(1,2)+x*ex_hat(2)+y*ey_hat(2)+z*ez_hat(2);...
                    P_Original(1,3)+x*ex_hat(3)+y*ey_hat(3)+z*ez_hat(3)];

    P_J1_ee = P_0_ee + P_origin
    % P_0_ee = P_J1_e - P_origin;
    Px = P_0_ee(1)
    Py = P_0_ee(2)
    Pz = P_0_ee(3)
    % Vẽ tọa độ các điểm và hệ trục tọa độ
    figure;
    hold on;
    grid on;
    axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Delta Robot - Forward Kinematics (J1 as Origin)');

    % Vẽ motor
    plot3(motor_positions(:,1), motor_positions(:,2), motor_positions(:,3), 'ro', 'MarkerSize', 7, 'MarkerFaceColor', 'r');
    text(motor_positions(:,1), motor_positions(:,2), motor_positions(:,3), {'Motor 1', 'Motor2', 'Motor 3'}, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');
    
    % Vẽ các điểm P1, P2, P3
    plot3(P_origin(1), P_origin(2), P_origin(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    text(P_origin(1), P_origin(2), P_origin(3), '  Origin', 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');

    plot3(P(:,1), P(:,2), P(:,3), 'ro', 'MarkerSize', 7, 'MarkerFaceColor', 'r');
    text(P(:,1), P(:,2), P(:,3), {'J1', 'J2', 'J3'}, 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');

     % Vẽ các đường thẳng nối J1, J2, J3 với Motor1, Motor2, Motor3
    for i = 1:3
        plot3([motor_positions(i,1), P(i,1)], [motor_positions(i,2), P(i,2)], [motor_positions(i,3), P(i,3)], 'k-', 'LineWidth', 2);
    end
    
     % Vẽ các đường nối Motor1 - Motor2 - Motor3
    motor_order = [1, 2; 2, 3; 3, 1];
    for i = 1:3
        plot3([motor_positions(motor_order(i,1),1), motor_positions(motor_order(i,2),1)],...
              [motor_positions(motor_order(i,1),2), motor_positions(motor_order(i,2),2)],...
              [motor_positions(motor_order(i,1),3), motor_positions(motor_order(i,2),3)], 'b-', 'LineWidth', 2);
    end

    % Vẽ hệ trục tọa độ local tại J1
    quiver3(0, 0, 0, ex_hat(1), ex_hat(2), ex_hat(3), 50, 'r', 'LineWidth', 2);
    quiver3(0, 0, 0, ey_hat(1), ey_hat(2), ey_hat(3), 50, 'g', 'LineWidth', 2);
    quiver3(0, 0, 0, ez_hat(1), ez_hat(2), ez_hat(3), 50, 'b', 'LineWidth', 2);

    % Vẽ điểm end-effector
    plot3(P_J1_ee(1), P_J1_ee(2), P_J1_ee(3), 'bo', 'MarkerSize', 7, 'MarkerFaceColor', 'b');
    text(P_J1_ee(1), P_J1_ee(2), P_J1_ee(3), 'EE', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left');

    legend('Motors', 'Joints', 'ex\_hat', 'ey\_hat', 'ez\_hat', 'End-Effector', 'Arms');;
  
    hold off;

end

function R = Rz(q)
R=[cos(q) -sin(q) 0 
    sin(q) cos(q) 0;
    0 0 1];
end

function T = Rotz(q)
R=Rz(q);
T=[R(1,1) R(1,2) R(1,3) 0;
   R(2,1) R(2,2) R(2,3) 0;
   R(3,1) R(3,2) R(3,3) 0;
   0 0 0 1];
end
