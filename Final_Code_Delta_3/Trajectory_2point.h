void waitUntilMotorsStop() {
  while ((motorRunning_1 || motorRunning_2 || motorRunning_3) && !emergencyStop) {
    if (emergencyStop) {
      StopAllMotors();
      break;
    }
    RunMotor_1();
    RunMotor_2();
    RunMotor_3();
    delay(1);
  }
}
void Trajectory_2point(float tf, float x0, float y0, float z0, float xf, float yf, float zf)
{
  for(int i = 0; i < 1; i++) {
      if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      if (input.equalsIgnoreCase("s")) {
        emergencyStop = true;
        StopAllMotors();
        break;
      }
    }
  }
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
void Trajectory_4point(float tf, float x0, float y0, float z0, float xf, float yf, float zf, char colour) 
{
  // Kiểm tra 1 lần duy nhất ở đầu hàm
  if(emergencyStop) return;

  const float high = -394.9;
  
  // Di chuyển tới phía trên vị trí vật
  if(!emergencyStop) Trajectory_2point(tf-t_up_down, x0, y0, z0, xf, yf, -365);
  
  // Hạ xuống nhặt vật
  if(!emergencyStop) Trajectory_2point(t_up_down, xf, yf, -365, xf, yf, high);
  
  // Bật nam châm nếu không dừng khẩn cấp
  if(!emergencyStop) {
    digitalWrite(namcham, HIGH);
    delay(500);
  }
  
  // Nâng vật lên
  if(!emergencyStop) Trajectory_2point(t_up_down, xf, yf, high, xf, yf, -365);
  
  // Xác định vị trí thả theo màu
  float dropX, dropY;
  switch(colour) {
    case 'R': dropX = 77.66; dropY = -113.09; break;
    case 'G': dropX = -22.54; dropY = -110.8; break;
    case 'Y': dropX = 24.26; dropY = 103.62; break;
    default: return;
  }
    
    // Di chuyển tới vị trí thả
  if (!emergencyStop) Trajectory_2point(tf, xf, yf, -365, dropX, dropY, -365);
  if (!emergencyStop) Trajectory_2point(t_up_down, dropX, dropY, -365, dropX, dropY, high);
  
  // Tắt nam châm và nâng lên
  if(!emergencyStop) {
    digitalWrite(namcham, LOW);
    Trajectory_2point(t_up_down, dropX, dropY, high, dropX, dropY, -365);
    
    // Cập nhật vị trí hiện tại
    currentPosition[0] = dropX;
    currentPosition[1] = dropY;
    currentPosition[2] = -365;
  }
}
void nextCycle(float newXf, float newYf, float newZf, float tf, String newColour)
{
  float high = -394.9;
  Trajectory_4point(tf, currentPosition[0], currentPosition[1], currentPosition[2], 
                   newXf, newYf, high, newColour.charAt(0));
}
