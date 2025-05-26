void waitUntilMotorsStop() {
  while ((motorRunning_1 || motorRunning_2 || motorRunning_3) && !emergencyStop) {
    RunMotor_1();
    RunMotor_2();
    RunMotor_3();
    delay(1); // để tránh chiếm CPU quá mức
  }
}
void Trajectory_2point(float tf, float x0, float y0, float z0, float xf, float yf, float zf)
{
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input[0] == 's' || input.equalsIgnoreCase("stop")) {
      StopAllMotors();
      return;
    }
  }
  if(!emergencyStop)
  {
      // Tính toán góc quay
    float angles_start[3], angles_end[3];
    if (inverse_kinematic(x0, y0, z0, angles_start) && 
        inverse_kinematic(xf, yf, zf, angles_end)) {

      deg1_old = angles_end[0];
      deg2_old = angles_end[1];
      deg3_old = angles_end[2];
      
      float angle_deg_1 = angles_end[0] - angles_start[0];
      float angle_deg_2 = angles_end[1] - angles_start[1];
      float angle_deg_3 = angles_end[2] - angles_start[2];

      // Cập nhật thông số động cơH
      updateMotorParameters(tf, angle_deg_1, angle_deg_2, angle_deg_3);
      
      // Khởi động động cơ
      motorRunning_1 = (nPulse_1 > 0);
      motorRunning_2 = (nPulse_2 > 0);
      motorRunning_3 = (nPulse_3 > 0);

      waitUntilMotorsStop();
    }
  }
}
void Trajectory_4point(float tf, float x0, float y0, float z0, float xf, float yf, float zf, char colour)
{
  // 1. Di chuyển đến phía trên vị trí vật thể
  Trajectory_2point(tf-t_up_down, x0, y0, z0, xf, yf, -365);
  // delay(100);
  // 2. Hạ xuống nhặt vật
  Trajectory_2point(t_up_down, xf, yf, -365, xf, yf, -395);

  digitalWrite(namcham, HIGH);
  delay(500);
  // 3. Nâng vật lên
  Trajectory_2point(t_up_down, xf, yf, -395, xf, yf, -365);
  // 4. Di chuyển đến vị trí thả theo màu
  float dropX, dropY, dropZ;
  switch(colour) {
  case 'R': // Màu đỏ
    dropX = 78.31; dropY = -100;
    // dropZ = -404.03;
    break;
  case 'G': // Màu xanh
    dropX = 9.36; dropY = -115.28; 
    // dropZ = -418.81;
    break;
  case 'Y': // Màu vàng gold
    dropX = -60.72; dropY = -114.2; 
    // dropZ = -406.84;
    break;
  default:
    return; // Màu không hợp lệ
  }
  // Di chuyển cao hơn Z
  Trajectory_2point(tf, xf, yf, -365, dropX, dropY, -365);

  Trajectory_2point(t_up_down, dropX, dropY, -365, dropX, dropY, -395);

  digitalWrite(namcham, LOW);
  // delay(100);

  // 6. Nâng lên và cập nhật vị trí hiện tại
  Trajectory_2point(t_up_down, dropX, dropY, -395, dropX, dropY, -365);

    // Lưu vị trí hiện tại
  currentPosition[0] = dropX;
  currentPosition[1] = dropY;
  currentPosition[2] = -365;

  // Serial.print("X:");Serial.println(currentPosition[0]);
  // Serial.print("Y:");Serial.println(currentPosition[1]);
  // Serial.print("Z:");Serial.println(currentPosition[2]);
}
// Khi gọi hàm cho chu trình tiếp theo:
void nextCycle(float newXf, float newYf, float newZf, float tf, String newColour)
{
  // Serial.print("X:");Serial.println(currentPosition[0]);
  // Serial.print("Y:");Serial.println(currentPosition[1]);
  // Serial.print("Z:");Serial.println(currentPosition[2]);
  // if(currentPosition[2] == 0) currentPosition[2] = -307.38;
  // Sử dụng vị trí hiện tại làm điểm bắt đầu
  Trajectory_4point(tf, currentPosition[0], currentPosition[1], currentPosition[2], 
                   newXf, newYf, -395, newColour.charAt(0));
}