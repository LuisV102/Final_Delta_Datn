function [x, y, z] = PulseToXYZ(pulse1, pulse2, pulse3)
    % Hàm chuyển đổi giá trị xung encoder sang tọa độ XYZ của robot
    % Đầu vào:
    %   pulse1, pulse2, pulse3: Giá trị xung từ 3 encoder
    % Đầu ra:
    %   [x, y, z]: Tọa độ vị trí đầu cuối robot
    
    % Các thông số cố định
    PULSES_PER_REV = 10000;  % Số xung trên mỗi vòng quay
    MICROSTEPPING = 8;     % Số vi bước
    
    % Tính toán góc quay từ xung encoder (đơn vị độ)
    % Công thức: degree = (100000 - pulse) / (MICROSTEPPING * PULSES_PER_REV / 360)
    theta1 = (100000 - pulse1) / (MICROSTEPPING * PULSES_PER_REV / 360);
    theta2 = (100000 - pulse2) / (MICROSTEPPING * PULSES_PER_REV / 360);
    theta3 = (100000 - pulse3) / (MICROSTEPPING * PULSES_PER_REV / 360);
    
    % Gọi hàm động học thuận để tính vị trí XYZ
    [x, y, z] = Forward_Kinamatic(theta1, theta2, theta3);
    
    % Hiển thị kết quả trung gian (có thể bỏ qua nếu không cần)
    fprintf('Góc quay các khớp:\n');
    fprintf('Theta1: %.2f độ\n', theta1);
    fprintf('Theta2: %.2f độ\n', theta2);
    fprintf('Theta3: %.2f độ\n', theta3);
    fprintf('Vị trí đầu cuối:\n');
    fprintf('X: %.2f mm\n', x);
    fprintf('Y: %.2f mm\n', y);
    fprintf('Z: %.2f mm\n', z);
end