void waitUntilMotorsStop() {
  while ((motorRunning_1 || motorRunning_2 || motorRunning_3)) {
    if (emergencyStop) {
      StopAllMotors();
      break;
    }
    RunMotor_1();
    RunMotor_2();
    RunMotor_3();
    delay(1); // để tránh chiếm CPU quá mức
  }
}
void Trajectory_2point(float tf, float x0, float y0, float z0, float xf, float yf, float zf)
{
  if (emergencyStop) return;
  else
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
  // Kiểm tra 1 lần duy nhất ở đầu hàm
  if(emergencyStop) return;

  const float high = -390;
  
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
  float high = -390;
  Trajectory_4point(tf, currentPosition[0], currentPosition[1], currentPosition[2], 
                   newXf, newYf, high, newColour.charAt(0));
}
// void Trajectory_4point(float tf, float x0, float y0, float z0, float xf, float yf, float zf, char colour)
// {
//   if (emergencyStop) return;
//   float high = -390;
//   // 1. Di chuyển đến phía trên vị trí vật thể
//   Trajectory_2point(tf-t_up_down, x0, y0, z0, xf, yf, -365);
//   // delay(100);
//   // 2. Hạ xuống nhặt vật
//   Trajectory_2point(t_up_down, xf, yf, -365, xf, yf, high);

//   digitalWrite(namcham, HIGH);
//   delay(500);
//   // 3. Nâng vật lên
//   Trajectory_2point(t_up_down, xf, yf, high, xf, yf, -365);
//   // 4. Di chuyển đến vị trí thả theo màu
//   float dropX, dropY, dropZ;
//   switch(colour) {
//   case 'R': // Màu đỏ
//     dropX = 78.31; dropY = -100;
//     // dropZ = -404.03;
//     break;
//   case 'G': // Màu xanh
//     dropX = 9.36; dropY = -110; 
//     // dropZ = -418.81;
//     break;
//   case 'Y': // Màu vàng gold
//     dropX = 24.26; dropY = 103.62; 
//     // dropZ = -406.84;
//     break;
//   default:
//     return; // Màu không hợp lệ
//   }
//   // Di chuyển cao hơn Z
//   Trajectory_2point(tf, xf, yf, -365, dropX, dropY, -365);

//   Trajectory_2point(t_up_down, dropX, dropY, -365, dropX, dropY, high);

//   digitalWrite(namcham, LOW);
//   // delay(100);

//   // 6. Nâng lên và cập nhật vị trí hiện tại
//   Trajectory_2point(t_up_down, dropX, dropY, high, dropX, dropY, -365);

//     // Lưu vị trí hiện tại
//   currentPosition[0] = dropX;
//   currentPosition[1] = dropY;
//   currentPosition[2] = -365;

//   // Serial.print("X:");Serial.println(currentPosition[0]);
//   // Serial.print("Y:");Serial.println(currentPosition[1]);
//   // Serial.print("Z:");Serial.println(currentPosition[2]);
// }
// Khi gọi hàm cho chu trình tiếp theo:
