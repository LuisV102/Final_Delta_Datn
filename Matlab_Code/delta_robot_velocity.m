function result = delta_robot_velocity(mode, input_values, theta_hientai, postion_hientai)
    % mode: 'joint_to_cartesian' hoặc 'cartesian_to_joint'
    % input_values: [X_dot; Y_dot; Z_dot] (end_eff_dot) 
    % hoặc [theta1_dot;theta2_dot; theta3_dot] (theta_dot)
    % r: Khoảng cách dịch chuyển của joint1,2,3
    % re: độ dài cánh tay trên
    % theta: [theta1; theta2; theta3] - góc quay các khớp
    
% Cách sử dụng: delta_robot_velocity('joint_velocity',[40;30;20](vận tốc giả định của end),[0;0;0](góc theta hiện tại 3 khớp),[0;0;-310.2515](vị trí hiện tại của end))

    r = 12;
    re = 150;
    theta_hientai = deg2rad(theta_hientai)
    X_ee = postion_hientai(1);
    Y_ee = postion_hientai(2);
    Z_ee = postion_hientai(3);
    % Tính toán Jacobian J_p và J_theta
    J_p = [ X_ee - (r + re * cos(theta_hientai(1))),  Y_ee,  Z_ee - re * sin(theta_hientai(1));
            X_ee + 0.5 * (r + re * cos(theta_hientai(2))),  Y_ee - (sqrt(3)/2) * (r + re * cos(theta_hientai(2))),  Z_ee - re * sin(theta_hientai(2));
            X_ee + 0.5 * (r + re * cos(theta_hientai(3))),  Y_ee + (sqrt(3)/2) * (r + re * cos(theta_hientai(3))),  Z_ee - re * sin(theta_hientai(3))];

    J_theta = [ re*(-X_ee*sin(theta_hientai(1)) + Z_ee*cos(theta_hientai(1)) + r*sin(theta_hientai(1))), 0, 0;
                0, re*(0.5*X_ee*sin(theta_hientai(2)) - sqrt(3)/2*Y_ee*sin(theta_hientai(2)) + Z_ee*cos(theta_hientai(2)) + r*sin(theta_hientai(2))), 0;
                0, 0, re*(0.5*X_ee*sin(theta_hientai(3)) + sqrt(3)/2*Y_ee*sin(theta_hientai(3)) + Z_ee*cos(theta_hientai(3)) + r*sin(theta_hientai(3)))];

    
    % Xác định chế độ hoạt động
    if strcmp(mode, 'joint_velocity')
        % Tính vận tốc khớp: dot_theta = inv(J_theta) * J_p * dot_P
        result = J_theta \ (J_p * input_values); % result đơn vị rad/s cho nên input_value sẽ là mm/s
    elseif strcmp(mode, 'end_effector_velocity')
        % Tính vận tốc điểm hiệu ứng cuối: dot_P = J_p * J_theta * dot_theta
        result = J_p \ (J_theta * input_values); % result đơn vị mm/s cho nên input_value sẽ là rad/s
    else
        error('Chế độ không hợp lệ. Chọn "cartesian_to_joint" hoặc "joint_to_cartesian".');
    end

end
