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
  trajectory_ready = false;
}

// void setupTrajectoryArrays(unsigned long p1, unsigned long p2, unsigned long p3) {
//   // Free existing arrays if they exist
//   if (delay_run_spd_traject_1) delete[] delay_run_spd_traject_1;
//   if (delay_run_spd_traject_2) delete[] delay_run_spd_traject_2;
//   if (delay_run_spd_traject_3) delete[] delay_run_spd_traject_3;

//   // Estimate required array sizes (you may need to calculate these properly)
//   // unsigned long estimatedPulses = 00; // Temporary value - calculate properly
  
//   delay_run_spd_traject_1 = new unsigned long[p1];
//   delay_run_spd_traject_2 = new unsigned long[p2]; 
//   delay_run_spd_traject_3 = new unsigned long[p3];

//   current_pulse_index_1 = 0;
//   current_pulse_index_2 = 0;
//   current_pulse_index_3 = 0;
// }

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
  
  // Tính toán góc quay trước
  float angles_start[3], angles_end[3];
  if (inverse_kinematic(x0, y0, z0, angles_start) && 
      inverse_kinematic(xf, yf, zf, angles_end)) {

    deg1_old = angles_end[0];
    deg2_old = angles_end[1];
    deg3_old = angles_end[2];
    
    float angle_deg_1 = angles_end[0] - angles_start[0];
    float angle_deg_2 = angles_end[1] - angles_start[1];
    float angle_deg_3 = angles_end[2] - angles_start[2];

    const float PULSE_PER_DEG = 1.0f / 0.225f; // Thêm vào Khaibao.h
    
    unsigned long p1 = abs(angle_deg_1) * PULSE_PER_DEG;
    unsigned long p2 = abs(angle_deg_2) * PULSE_PER_DEG;
    unsigned long p3 = abs(angle_deg_3) * PULSE_PER_DEG;

    // Serial.println("====== NEW MOVE DIAGNOSTICS ======");
    // Serial.print("Target Coordinates: (");
    // Serial.print(xf); Serial.print(", "); Serial.print(yf); Serial.print(", "); Serial.print(zf);
    // Serial.println(")");
    // Serial.print("Time (tf): "); Serial.println(tf);
    // Serial.print("Calculated Pulses (p1, p2, p3): ");
    // Serial.print(p1); Serial.print(", "); Serial.print(p2); Serial.print(", "); Serial.println(p3);

    if (p1 >= MAX_PULSES_PER_MOVE || p2 >= MAX_PULSES_PER_MOVE || p3 >= MAX_PULSES_PER_MOVE) {
      Serial.println("ERROR: Move requires too many pulses! Increase MAX_PULSES_PER_MOVE.");
      StopAllMotors();
      return;
    }
    
    // Cấp phát mảng có chiều dài ứng với từng số xung
    // setupTrajectoryArrays(p1, p2, p3);

    // Cập nhật delay cho từng động cơ
    updateMotorParameters(tf, angle_deg_1, angle_deg_2, angle_deg_3);
    
    // Serial.print("Trajectory Delays (Start, End) M1: ");
    // if (nPulse_1 > 0) {
    //   Serial.print(delay_run_spd_traject_1[0]); Serial.print(", "); Serial.print(delay_run_spd_traject_1[nPulse_1 - 1]);
    // } else {
    //   Serial.print("N/A");
    // }
    // Serial.println();

    // Serial.print("Trajectory Delays (Start, End) M2: ");
    // if (nPulse_2 > 0) {
    //   Serial.print(delay_run_spd_traject_2[0]); Serial.print(", "); Serial.print(delay_run_spd_traject_2[nPulse_2 - 1]);
    // } else {
    //   Serial.print("N/A");
    // }
    // Serial.println();

    // Serial.print("Trajectory Delays (Start, End) M3: ");
    // if (nPulse_3 > 0) {
    //   Serial.print(delay_run_spd_traject_3[0]); Serial.print(", "); Serial.print(delay_run_spd_traject_3[nPulse_3 - 1]);
    // } else {
    //   Serial.print("N/A");
    // }
    // Serial.println("====================================");
    // ==========================================

    // Khởi động động cơ
    motorRunning_1 = (nPulse_1 > 0);
    motorRunning_2 = (nPulse_2 > 0);
    motorRunning_3 = (nPulse_3 > 0);

    // **Reset các chỉ số xung TRƯỚC KHI bắt đầu chuyển động mới**
    current_pulse_index_1 = 0;
    current_pulse_index_2 = 0;
    current_pulse_index_3 = 0;
    waitUntilMotorsStop();
  }
}
void Trajectory_4point(float tf, float x0, float y0, float z0, float xf, float yf, float zf, char colour) 
{
  // Kiểm tra 1 lần duy nhất ở đầu hàm
  if(emergencyStop) return;

  const float high = -399.3;
  
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
    case 'R': dropX = 87.66; dropY = -113.09; break;
    case 'G': dropX = -22.54; dropY = -110.8; break;
    case 'Y': dropX = 34.26; dropY = 113.62; break;
    default: return;
  }
    
    // Di chuyển tới vị trí thả
  if (!emergencyStop) Trajectory_2point(tf, xf, yf, -365, dropX, dropY, -365);
  delay(1);
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
  float high = -399.3;
  Trajectory_4point(tf, currentPosition[0], currentPosition[1], currentPosition[2], 
                   newXf, newYf, high, newColour.charAt(0));
}
