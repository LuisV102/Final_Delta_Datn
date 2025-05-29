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

void updateMotorParameters(float tf, float angle_deg_1, float angle_deg_2, float angle_deg_3) {
  const float DEG_PER_PULSE = 0.225f; // 0.225 độ/xung f là hậu tố float
  const float PULSE_PER_DEG = 1.0f / DEG_PER_PULSE;

  // Tính số xung cần thiết (dựa trên góc quay tuyệt đối)
  nPulse_1 = static_cast<unsigned long>(abs(angle_deg_1) * PULSE_PER_DEG); //static_cast là ép kiểu an toàn
  nPulse_2 = static_cast<unsigned long>(abs(angle_deg_2) * PULSE_PER_DEG);
  nPulse_3 = static_cast<unsigned long>(abs(angle_deg_3) * PULSE_PER_DEG);

  // Tính delay giữa các xung (đảm bảo hoàn thành trong thời gian tf)
  if (nPulse_1 > 0) {
    delay_run_spd_1 = static_cast<unsigned long>((tf * 1e6) / (nPulse_1 * 2));
  }
  if (nPulse_2 > 0) {
    delay_run_spd_2 = static_cast<unsigned long>((tf * 1e6) / (nPulse_2 * 2));
  }
  if (nPulse_3 > 0) {
    delay_run_spd_3 = static_cast<unsigned long>((tf * 1e6) / (nPulse_3 * 2));
  }

  // Đặt hướng quay
  digitalWrite(DIR_PIN_1, angle_deg_1 >= 0 ? HIGH : LOW); //angle_deg_1 > 0 thì digitalWrite(DIR_PIN_1, HIGH) else LOW
  digitalWrite(DIR_PIN_2, angle_deg_2 >= 0 ? HIGH : LOW);
  digitalWrite(DIR_PIN_3, angle_deg_3 >= 0 ? HIGH : LOW);

//   Serial.print("Pulses: "); Serial.print(nPulse_1); Serial.print(", "); 
//   Serial.print(nPulse_2); Serial.print(", "); Serial.println(nPulse_3);
//   Serial.print("Delays (µs): "); Serial.print(delay_run_spd_1); Serial.print(", "); 
//   Serial.print(delay_run_spd_2); Serial.print(", "); Serial.println(delay_run_spd_3);
}

// void StopAllMotors() {
//   motorRunning_1 = false;
//   motorRunning_2 = false;
//   motorRunning_3 = false;
//   digitalWrite(PUL_PIN_1, LOW);
//   digitalWrite(PUL_PIN_2, LOW);
//   digitalWrite(PUL_PIN_3, LOW);
// }

unsigned long lastPulseTime_1 = 0;
void RunMotor_1()
{
  if (emergencyStop) {
    StopMotor_1();
    return;
  }
  if (motorRunning_1 && nPulse_1 > 0) 
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
}

unsigned long lastPulseTime_2 = 0;
void RunMotor_2()
{
  if (emergencyStop) {
    StopMotor_2();
    return;
  }
  if (motorRunning_2 && nPulse_2 > 0) 
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
}

unsigned long lastPulseTime_3 = 0;
void RunMotor_3()
{
  if (emergencyStop) {
    StopMotor_3();
    return;
  }
  if (motorRunning_3 && nPulse_3 > 0) 
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
}
