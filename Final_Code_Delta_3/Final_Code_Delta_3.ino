#include "Khaibao.h"
#include "Run_Stop_Mtor.h"
#include "encoder.h"
#include "Calculate.h"
#include "Set_Position.h"
#include "Set_Home.h"
#include "Trajectory_2point.h"

bool manualControlEnabled = false;  // Flag to track if manual control is enabled

void setup() {
  // Khởi tạo chân IO
  pinMode(PUL_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(PUL_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(PUL_PIN_3, OUTPUT);
  pinMode(DIR_PIN_3, OUTPUT);
  pinMode(namcham, OUTPUT);
  pinMode(bangtai, OUTPUT);

  // Khởi tạo encoder
  pinMode(pinA_1, INPUT_PULLUP);
  pinMode(pinB_1, INPUT_PULLUP);
  pinMode(pinA_2, INPUT_PULLUP);
  pinMode(pinB_2, INPUT_PULLUP);
  pinMode(pinA_3, INPUT_PULLUP);
  pinMode(pinB_3, INPUT_PULLUP);

  // Bật ngắt encoder
  attachInterrupt(digitalPinToInterrupt(pinA_1), handleA_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB_1), handleB_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinA_2), handleA_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB_2), handleB_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinA_3), handleA_3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB_3), handleB_3, CHANGE);

  // Cảm biến giới hạn
  pinMode(limit_1, INPUT_PULLUP);
  pinMode(limit_2, INPUT_PULLUP);
  pinMode(limit_3, INPUT_PULLUP);

  currentPosition[0] = 0;
  currentPosition[1] = 0;
  currentPosition[2] = -307.38; // Độ cao home

  emergencyStop = false; // Reset cờ dừng khẩn cấp khi khởi động

  Serial.begin(9600);
  Serial.println("System Ready. Send commands in format: P0:x,y,z;Pf:x,y,z;T:time");
}

void loop() {
  // Xử lý lệnh từ Serial
    // Khởi tạo vị trí hiện tại về home

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    processInputCommand(input);
  }
    if (millis() - lastPrintTime >= 2000) 
  { 
      Serial.print("Encoder 1: "); Serial.print(encoderPosition_1*45/9900);  Serial.print("  ");
      Serial.print("Encoder 2: "); Serial.print(encoderPosition_2*45/9900);  Serial.print("  ");
      Serial.print("Encoder 3: "); Serial.println(encoderPosition_3*45/9900); 
      // (long)encoderCalibrated_1 
      lastPrintTime = millis();
  }
  // Chạy các động cơ
  RunMotor_1();
  RunMotor_2();
  RunMotor_3();
}

void processInputCommand(String input) {
  // Các lệnh start/stop luôn hoạt động bất kể trạng thái manualControlEnabled
  // Các lệnh start/stop luôn hoạt động bất kể trạng thái manualControlEnabled
  if (input.equalsIgnoreCase("start")) {
    manualControlEnabled = true;
    emergencyStop = false;  // Reset trạng thái dừng khẩn cấp
    Serial.println("Manual control ENABLED");
    return;
  }
  else if (input.equalsIgnoreCase("stop") || input[0] == 's') {
    manualControlEnabled = false;
    emergencyStop = true;  // Kích hoạt dừng khẩn cấp
    StopAllMotors(); // Gọi hàm dừng khẩn cấp
    Serial.println("EMERGENCY STOP ACTIVATED");
    return;
  }

  // Nếu đang trong trạng thái dừng khẩn cấp, không xử lý bất kỳ lệnh nào khác
  if (emergencyStop) {
    Serial.println("System in EMERGENCY STOP state. Send 'start' to reset");
    return;
  }
  else if (input[0] == 'h') {
    SetHome();
  }
  else if (input[0] == 'u') {
    digitalWrite(namcham, HIGH);
    Serial.println("Magnet ON");
  }
  else if (input[0] == 'd') {
    digitalWrite(namcham, LOW);
    Serial.println("Magnet OFF");
  }
  else if (input[0] == 'r') {
    digitalWrite(bangtai, HIGH);
    Serial.println("Conveyor ON");
  }
  else if (input[0] == 'o') {
    digitalWrite(bangtai, LOW);
    Serial.println("Conveyor off");
  } 
  else if (input.startsWith("P0:") && input.indexOf("Pf:") > 0 && input.indexOf("T:") > 0) {
    processPositionCommand(input);
    // P0:0,0,-307.38;Pf:0,0,-395;T:1 -> No Colour
    // P0:0,0,-307.38;Pf:0,0,-395;T:1;C:R -> with Colour 
  }
  else if (input.startsWith("Next:")) {
    Serial.print("X:");Serial.println(currentPosition[0]);
    Serial.print("Y:");Serial.println(currentPosition[1]);
    Serial.print("Z:");Serial.println(currentPosition[2]);
    processNextCycleCommand(input);
    // Next:0,0,-395;T:1;C:R
  }
  else {
    processAngleCommand(input);
  }
}


void processPositionCommand(String input) {
  // Parse P0
  int p0_start = input.indexOf("P0:") + 3;
  int p0_end = input.indexOf(";");
  String p0_str = input.substring(p0_start, p0_end);
  
  // Parse Pf
  int pf_start = input.indexOf("Pf:") + 3;
  int pf_end = input.indexOf(";", p0_end + 1);
  String pf_str = input.substring(pf_start, pf_end);
  
  // Parse T
  int tf_start = input.indexOf("T:") + 2;
  int colour_start = input.indexOf("C:");
  String tf_str = (colour_start == -1) ? input.substring(tf_start) : input.substring(tf_start, colour_start);

  // Parse Colour (nếu có)
  char colour = ' '; // Mặc định không màu
  if(colour_start != -1) {
    colour = input.charAt(colour_start + 2);
  }

  // Convert to float values
  P0[0] = getValue(p0_str, ',', 0).toFloat();
  P0[1] = getValue(p0_str, ',', 1).toFloat();
  P0[2] = getValue(p0_str, ',', 2).toFloat();
  
  Pf[0] = getValue(pf_str, ',', 0).toFloat();
  Pf[1] = getValue(pf_str, ',', 1).toFloat();
  Pf[2] = getValue(pf_str, ',', 2).toFloat();
  
  tf = tf_str.toFloat();
    // Chọn hàm quy hoạch dựa trên có màu hay không
  if(colour == ' ') {
    Trajectory_2point(tf, P0[0], P0[1], P0[2], Pf[0], Pf[1], Pf[2]);
    Stop_toggle();
  } else {
    Trajectory_4point(tf, P0[0], P0[1], P0[2], Pf[0], Pf[1], Pf[2], colour);
    Stop_toggle();
  }
}

void processAngleCommand(String input) {
  if(!manualControlEnabled) return;  // Thay enable_manual bằng manualControlEnabled
  
  inString = "";
  for (int x = 0; x < input.length(); x++) {
    if ((input[x] == '-') || (input[x] == '.')) {
      inString += (char)input[x];
    }
    if (isDigit(input[x])) {
      inString += (char)input[x];
    }
    
    if (input[x] == 'A') {
      deg1 = inString.toFloat();
      if (abs(deg1 - deg1_old) > 0.1) {
        Degree_1new(deg1, deg1_old);
        deg1_old = deg1;
      }
      inString = "";
    } 
    else if (input[x] == 'B') {
      deg2 = inString.toFloat();
      if (abs(deg2 - deg2_old) > 0.1) {
        Degree_2new(deg2, deg2_old);
        deg2_old = deg2;
      }
      inString = "";
    }
    else if (input[x] == 'C') {
      deg3 = inString.toFloat();
      if (abs(deg3 - deg3_old) > 0.1) {
        Degree_3new(deg3, deg3_old);
        deg3_old = deg3;
      }
      inString = "";
    }
  }
}

void processNextCycleCommand(String input) {
  // Format: Next:x,y,z;T:time;C:R

  int coord_start = input.indexOf("Next:") + 5;
  int coord_end = input.indexOf(";");
  String coord_str = input.substring(coord_start, coord_end);

  float newXf = getValue(coord_str, ',', 0).toFloat();
  float newYf = getValue(coord_str, ',', 1).toFloat();
  float newZf = getValue(coord_str, ',', 2).toFloat();

    // Lấy tf
  int tf_start = input.indexOf("T:") + 2;
  int tf_end = input.indexOf(";", tf_start);
  String tf_str = input.substring(tf_start, tf_end);
  float tf_val = tf_str.toFloat();

  char colour = input.charAt(input.indexOf("C:") + 2);
  String colourStr = String(colour);

  nextCycle(newXf, newYf, newZf, tf_val, colourStr);
}

String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}