void SetHome()
{
  if (emergencyStop) return;
  digitalWrite(DIR_PIN_1,0);
  digitalWrite(DIR_PIN_2,0);
  digitalWrite(DIR_PIN_3,0);
  delayMicroseconds(20);
  bool xHomed = false; // Cờ để kiểm tra trục X đã về gốc
  bool yHomed = false; // Cờ để kiểm tra trục Y đã về gốc
  bool zHomed = false; // Cờ để kiểm tra trục Z đã về gốc

  while (!xHomed || !yHomed || !zHomed && !emergencyStop) 
  {
    // Set home cho motor 1
    if (!xHomed && !emergencyStop)
    {
      if (digitalRead(limit_1) == LOW)
      {
        delay(10); // chống dội
        if (digitalRead(limit_1) == LOW) // kiểm tra lại sau delay
        {
          StopMotor_1();
          delay(50);
          SetPosition_1();
          xHomed = true;
        }
      }
      else
      {
        digitalWrite(PUL_PIN_1, HIGH);
        delayMicroseconds(delay_home_spd);
        digitalWrite(PUL_PIN_1, LOW);
        delayMicroseconds(delay_home_spd);
      }
    }
    // Set home cho motor 2
    if (!yHomed && !emergencyStop)
    {
      if (digitalRead(limit_2) == LOW)
      {
        delay(10); // chống dội
        if (digitalRead(limit_2) == LOW) // kiểm tra lại sau delay
        {
          StopMotor_2();
          delay(50);
          SetPosition_2();
          yHomed = true;
        }
      }
      else
      {
        digitalWrite(PUL_PIN_2, HIGH);
        delayMicroseconds(delay_home_spd);
        digitalWrite(PUL_PIN_2, LOW);
        delayMicroseconds(delay_home_spd);
      }
    }
    // Set home cho motor 3
    if (!zHomed && !emergencyStop)
    {
      if (digitalRead(limit_3) == LOW)
      {
        delay(10); // chống dội
        if (digitalRead(limit_3) == LOW) // kiểm tra lại sau delay
        {
          StopMotor_3();
          delay(50);
          SetPosition_3();
          zHomed = true;
        }
      }
      else
      {
        digitalWrite(PUL_PIN_3, HIGH);
        delayMicroseconds(delay_home_spd);
        digitalWrite(PUL_PIN_3, LOW);
        delayMicroseconds(delay_home_spd);
      }
    }
  }
  if (emergencyStop) {
    StopAllMotors();
    return;
  }

  // Đặt lại delay_run_spd_x về giá trị mặc định cho các chuyển động độc lập (hoặc giá trị mong muốn sau homing)
  // Điều này đảm bảo các lệnh Degree_x sau đó sẽ dùng tốc độ này, không bị ảnh hưởng bởi update_delay_run_spd
  delay_run_spd_1 = 5000; // Hoặc một giá trị khác bạn muốn cho tốc độ di chuyển sau homing
  delay_run_spd_2 = 5000;
  delay_run_spd_3 = 5000;
  
  Degree_1new(7.2, 0);  // ~0.148 rad
  Degree_2new(9.3, 0);   // ~0.209 rad
  Degree_3new(6.8, 0);    // ~0.122 rad

  // Cho phép loop() chạy tiếp để điều khiển motor
  motorRunning_1 = true;
  motorRunning_2 = true;
  motorRunning_3 = true;

  // Chờ đến khi tất cả đã chạy xong
  while (motorRunning_1 || motorRunning_2 || motorRunning_3) 
  {
    RunMotor_1();
    RunMotor_2();
    RunMotor_3();
  }
  delay(800);
  SetPosition_1();
  SetPosition_2();
  SetPosition_3();
  
  currentPosition[0] = 0;
  currentPosition[1] = 0;
  currentPosition[2] = -307.38; // Độ cao home

  emergencyStop = false;
}