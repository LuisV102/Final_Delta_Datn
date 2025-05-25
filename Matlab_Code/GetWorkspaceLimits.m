function [xlim, ylim, zlim] = GetWorkspaceLimits()
    % Dải quét góc rộng
    theta_range = -8:5:45;

    % Giới hạn góc hợp lệ
    theta_min = -8.5;
    theta_max = 40;

    fprintf('Giới hạn góc đặt trước: [%.2f°, %.2f°]\n', theta_min, theta_max);

    % Khởi tạo mảng chứa các điểm đầu ra và góc
    X = []; Y = []; Z = [];
    valid_theta1 = []; valid_theta2 = []; valid_theta3 = [];

    for theta1 = theta_range
        for theta2 = theta_range
            for theta3 = theta_range
                % Kiểm tra giới hạn góc
                if theta1 < theta_min || theta1 > theta_max || ...
                   theta2 < theta_min || theta2 > theta_max || ...
                   theta3 < theta_min || theta3 > theta_max
                    continue;
                end

                try
                    [Px, Py, Pz] = Forward_Kinamatic(theta1, theta2, theta3);
                    if isreal([Px, Py, Pz]) && Pz >= -400 && Pz <= 0
                        X(end+1) = Px;
                        Y(end+1) = Py;
                        Z(end+1) = Pz;

                        % Lưu lại các góc hợp lệ
                        valid_theta1(end+1) = theta1;
                        valid_theta2(end+1) = theta2;
                        valid_theta3(end+1) = theta3;
                    end
                catch
                    continue;
                end
            end
        end
    end

    if isempty(X)
        warning('Không có điểm nào hợp lệ.');
        xlim = [0, 0];
        ylim = [0, 0];
        zlim = [0, 0];
        return;
    end

    % Tính giới hạn không gian
    xlim = [min(X), max(X)];
    ylim = [min(Y), max(Y)];
    zlim = [min(Z), max(Z)];

    % Tính giới hạn góc thực tế đã sử dụng
    theta1_range = [min(valid_theta1), max(valid_theta1)];
    theta2_range = [min(valid_theta2), max(valid_theta2)];
    theta3_range = [min(valid_theta3), max(valid_theta3)];

    % In kết quả
    fprintf('Giới hạn thực tế của góc theta1: [%.2f°, %.2f°]\n', theta1_range(1), theta1_range(2));
    fprintf('Giới hạn thực tế của góc theta2: [%.2f°, %.2f°]\n', theta2_range(1), theta2_range(2));
    fprintf('Giới hạn thực tế của góc theta3: [%.2f°, %.2f°]\n', theta3_range(1), theta3_range(2));
    fprintf('Giới hạn X: [%.2f, %.2f]\n', xlim(1), xlim(2));
    fprintf('Giới hạn Y: [%.2f, %.2f]\n', ylim(1), ylim(2));
    fprintf('Giới hạn Z: [%.2f, %.2f]\n', zlim(1), zlim(2));

    % Vẽ workspace
    scatter3(X, Y, Z, '.');
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Robot Delta Workspace');
    grid on; axis equal;

    % % Giới hạn quét cho các góc servo (giả sử từ -60 đến 60 độ)
    % theta_range = -60:5:60; % bước 5 độ để giảm thời gian tính
    % 
    % % Khởi tạo mảng chứa các điểm đầu ra
    % X = [];
    % Y = [];
    % Z = [];
    % 
    % % Quét tất cả tổ hợp của theta1, theta2, theta3
    % for theta1 = theta_range
    %     for theta2 = theta_range
    %         for theta3 = theta_range
    %             try
    %                 [Px, Py, Pz] = Forward_Kinamatic(theta1, theta2, theta3);
    %                 if isreal([Px, Py, Pz]) % Kiểm tra nghiệm thực
    %                     X(end+1) = Px;
    %                     Y(end+1) = Py;
    %                     Z(end+1) = Pz;
    %                 end
    %             catch
    %                 % Bỏ qua các giá trị gây lỗi (góc không hợp lệ)
    %                 continue;
    %             end
    %         end
    %     end
    % end
    % 
    % % Tính giới hạn
    % xlim = [min(X), max(X)];
    % ylim = [min(Y), max(Y)];
    % zlim = [min(Z), max(Z)];
    % 
    % % Hiển thị kết quả
    % fprintf('Giới hạn X: [%.2f, %.2f]\n', xlim(1), xlim(2));
    % fprintf('Giới hạn Y: [%.2f, %.2f]\n', ylim(1), ylim(2));
    % fprintf('Giới hạn Z: [%.2f, %.2f]\n', zlim(1), zlim(2));
    % 
    % scatter3(X, Y, Z, '.');
    % xlabel('X'); ylabel('Y'); zlabel('Z');
    % title('Robot Delta Workspace');
    % grid on; axis equal;
    % 

end
