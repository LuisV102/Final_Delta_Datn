void StopMotor_1() {
  motorRunning_1 = false;
  digitalWrite(PUL_PIN_1, LOW);
}

void StopMotor_2() {
  motorRunning_2 = false;
  digitalWrite(PUL_PIN_2, LOW);
}

void StopMotor_3() {
  motorRunning_3 = false;
  digitalWrite(PUL_PIN_3, LOW);
}

void StopAllMotors() {
  emergencyStop = true;  // Bật cờ dừng khẩn cấp

  motorRunning_1 = false;
  motorRunning_2 = false;
  motorRunning_3 = false;

  // Ngắt các xung đang phát
  digitalWrite(PUL_PIN_1, LOW);
  digitalWrite(PUL_PIN_2, LOW);
  digitalWrite(PUL_PIN_3, LOW);
  
  // Reset số xung còn lại
  nPulse_1 = 0;
  nPulse_2 = 0;
  nPulse_3 = 0;
  current_pulse_index_1 = 0;
  current_pulse_index_2 = 0;
  current_pulse_index_3 = 0;
  trajectory_ready = false;

    // Tắt tất cả thiết bị ngoại vi
  digitalWrite(namcham, LOW);
  digitalWrite(bangtai, LOW);
}

void Stop_toggle()
{
  if (digitalRead(limit_1) == LOW) {
    StopMotor_1();
  }
  if (digitalRead(limit_2) == LOW) {
    StopMotor_2();
  }
  if (digitalRead(limit_3) == LOW) {
    StopMotor_3();
  }
}
void calculateTrajectory(unsigned long delay_max, unsigned long xung_max, unsigned long* delay_array) {
  if (xung_max == 0) return;
  
  const int MIN_PULSES_FOR_CURVE = 15; 

  if (xung_max < MIN_PULSES_FOR_CURVE) {
    // Chạy với tốc độ không đổi
    for (unsigned long i = 0; i < xung_max; i++) {
      delay_array[i] = delay_max; // Gán cùng một giá trị delay cho tất cả các xung
    }
    return; // Kết thúc hàm tại đây
  }

  double mau_so = ((double)xung_max * xung_max) / 4.0 - (double)(3 * xung_max) / 2.0;
  if (abs(mau_so) < 1e-6) { // Kiểm tra nếu mẫu số quá gần 0
      // Trường hợp hiếm gặp, chạy tốc độ không đổi để phòng ngừa
      for (unsigned long i = 0; i < xung_max; i++) {
          delay_array[i] = delay_max;
      }
      return;
  } 
  // Calculate coefficients for quadratic speed profile
  float a = (delay_max / 2.0) / ((xung_max * xung_max) / 4.0 - 3 * xung_max / 2.0);
  float b = -a * (xung_max + 1);
  float c = delay_max + a * xung_max;
  
  // Calculate delay for each pulse
  // for (unsigned long xung_hien_tai = 0; xung_hien_tai < xung_max; xung_hien_tai++) {
  //   float delay = a * pow(xung_hien_tai, 2) + b * xung_hien_tai + c;
  //   delay_array[xung_hien_tai] = static_cast<unsigned long>(delay);
  // }
  for (unsigned long xung_hien_tai = 0; xung_hien_tai < xung_max; xung_hien_tai++) {
    double delay_double = a * pow(xung_hien_tai, 2) + b * xung_hien_tai + c;
    
    // Đảm bảo delay không âm hoặc quá nhỏ
    if (delay_double < 10.0) { // 10us là một giới hạn an toàn tối thiểu
        delay_double = 10.0;
    }
    delay_array[xung_hien_tai] = static_cast<unsigned long>(delay_double);
  }
}

void updateMotorParameters(float tf, float angle_deg_1, float angle_deg_2, float angle_deg_3) {
  const float DEG_PER_PULSE = 0.225f; // 0.225 độ/xung f là hậu tố float
  const float PULSE_PER_DEG = 1.0f / DEG_PER_PULSE;
  unsigned long delay_max_1 = 0, delay_max_2 = 0, delay_max_3 = 0;
  // Tính số xung cần thiết (dựa trên góc quay tuyệt đối)
  nPulse_1 = static_cast<unsigned long>(abs(angle_deg_1) * PULSE_PER_DEG); //static_cast là ép kiểu an toàn
  nPulse_2 = static_cast<unsigned long>(abs(angle_deg_2) * PULSE_PER_DEG);
  nPulse_3 = static_cast<unsigned long>(abs(angle_deg_3) * PULSE_PER_DEG);

  // Tính delay giữa các xung (đảm bảo hoàn thành trong thời gian tf)
  if (nPulse_1 > 0) {
    delay_max_1 = static_cast<unsigned long>((tf * 1e6) / (nPulse_1 * 2));
    calculateTrajectory(delay_max_1, nPulse_1, delay_run_spd_traject_1);
  }
  if (nPulse_2 > 0) {
    delay_max_2 = static_cast<unsigned long>((tf * 1e6) / (nPulse_2 * 2));
    calculateTrajectory(delay_max_2, nPulse_2, delay_run_spd_traject_2);
  }
  if (nPulse_3 > 0) {
    delay_max_3 = static_cast<unsigned long>((tf * 1e6) / (nPulse_3 * 2));
    calculateTrajectory(delay_max_3, nPulse_3, delay_run_spd_traject_3);
  }

  // Đặt hướng quay
  digitalWrite(DIR_PIN_1, angle_deg_1 >= 0 ? HIGH : LOW); //angle_deg_1 > 0 thì digitalWrite(DIR_PIN_1, HIGH) else LOW
  digitalWrite(DIR_PIN_2, angle_deg_2 >= 0 ? HIGH : LOW);
  digitalWrite(DIR_PIN_3, angle_deg_3 >= 0 ? HIGH : LOW);

  trajectory_ready = true;
//   Serial.print("Pulses: "); Serial.print(nPulse_1); Serial.print(", "); 
//   Serial.print(nPulse_2); Serial.print(", "); Serial.println(nPulse_3);
//   Serial.print("Delays (µs): "); Serial.print(delay_run_spd_1); Serial.print(", "); 
//   Serial.print(delay_run_spd_2); Serial.print(", "); Serial.println(delay_run_spd_3);
}



unsigned long lastPulseTime_1 = 0;
void RunMotor_1()
{
  if (emergencyStop) {
    StopMotor_1();
    return;
  }
  if (motorRunning_1 && nPulse_1 > 0 && !trajectory_ready) 
  {
    unsigned long now = micros();
    if (now - lastPulseTime_1 >= delay_run_spd_1) 
    { // mỗi 60us (30us high + 30us low)
      digitalWrite(PUL_PIN_1, !digitalRead(PUL_PIN_1)); // Toggle xung
      lastPulseTime_1 = now;

      if (digitalRead(PUL_PIN_1) == LOW) 
      { // mỗi chu kỳ LOW xong giảm 1 xung
        nPulse_1--;
        if (nPulse_1 <= 0) 
        {
          StopMotor_1();
        }
      }
    }
  }
  if (motorRunning_1 && nPulse_1 > 0 && trajectory_ready) {
    unsigned long now = micros();
    unsigned long current_delay = delay_run_spd_traject_1[current_pulse_index_1];
    
    if (now - lastPulseTime_1 >= current_delay) {
      digitalWrite(PUL_PIN_1, !digitalRead(PUL_PIN_1));
      lastPulseTime_1 = now;

      if (digitalRead(PUL_PIN_1) == LOW) {
        nPulse_1--;
        current_pulse_index_1++;
        if (nPulse_1 <= 0) StopMotor_1();
      }
    }
  }
}

unsigned long lastPulseTime_2 = 0;
void RunMotor_2()
{
  if (emergencyStop) {
    StopMotor_2();
    return;
  }
  if (motorRunning_2 && nPulse_2 > 0 && !trajectory_ready) 
  {
    unsigned long now = micros();
    if (now - lastPulseTime_2 >= delay_run_spd_2) 
    { // mỗi 60us (30us high + 30us low)
      digitalWrite(PUL_PIN_2, !digitalRead(PUL_PIN_2)); // Toggle xung
      lastPulseTime_2 = now;

      if (digitalRead(PUL_PIN_2) == LOW) 
      { // mỗi chu kỳ LOW xong giảm 1 xung
        nPulse_2--;
        if (nPulse_2 <= 0) 
        {
          StopMotor_2();
        }
      }
    }
  }
  if (motorRunning_2 && nPulse_2 > 0 && trajectory_ready) {
    unsigned long now = micros();
    unsigned long current_delay = delay_run_spd_traject_2[current_pulse_index_2];
    
    if (now - lastPulseTime_2 >= current_delay) {
      digitalWrite(PUL_PIN_2, !digitalRead(PUL_PIN_2));
      lastPulseTime_2 = now;

      if (digitalRead(PUL_PIN_2) == LOW) {
        nPulse_2--;
        current_pulse_index_2++;
        if (nPulse_2 <= 0) StopMotor_2();
      }
    }
  }
}

unsigned long lastPulseTime_3 = 0;
void RunMotor_3()
{
  if (emergencyStop) {
    StopMotor_3();
    return;
  }
  if (motorRunning_3 && nPulse_3 > 0 && !trajectory_ready) 
  {
    unsigned long now = micros();
    if (now - lastPulseTime_3 >= delay_run_spd_3) 
    { // mỗi 60us (30us high + 30us low)
      digitalWrite(PUL_PIN_3, !digitalRead(PUL_PIN_3)); // Toggle xung
      lastPulseTime_3 = now;

      if (digitalRead(PUL_PIN_3) == LOW) 
      { // mỗi chu kỳ LOW xong giảm 1 xung
        nPulse_3--;
        if (nPulse_3 <= 0) 
        {
          StopMotor_3();
        }
      }
    }
  }
  if (motorRunning_3 && nPulse_3 > 0 && trajectory_ready) {
    unsigned long now = micros();
    unsigned long current_delay = delay_run_spd_traject_3[current_pulse_index_3];
    
    if (now - lastPulseTime_3 >= current_delay) {
      digitalWrite(PUL_PIN_3, !digitalRead(PUL_PIN_3));
      lastPulseTime_3 = now;

      if (digitalRead(PUL_PIN_3) == LOW) {
        nPulse_3--;
        current_pulse_index_3++;
        if (nPulse_3 <= 0) StopMotor_3();
      }
    }
  }
}