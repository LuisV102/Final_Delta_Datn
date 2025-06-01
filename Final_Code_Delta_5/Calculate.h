bool inverse_kinematic(float X_ee, float Y_ee, float Z_ee, float *J)
{
  const float R_Base = 60.0;
  const float R_platform = 42.62;
  const float r = R_Base - R_platform;
  const float re = 150.0;
  const float rf = 350.0;
  const float threshold = 0.001;
  const float alpha_deg[3] = {0, 120, 240};

  for (int i = 0; i < 3; i++) {
    float alpha = radians(alpha_deg[i]); // Chuyển alpha từ độ sang radian
    float cos_alpha = cos(alpha);
    float sin_alpha = sin(alpha);

    float A = -2.0 * re * (-r + X_ee * cos_alpha + Y_ee * sin_alpha);
    float B = -2.0 * re * Z_ee;
    float C = (X_ee * X_ee + Y_ee * Y_ee + Z_ee * Z_ee + r * r + re * re - rf * rf
              - 2 * r * (X_ee * cos_alpha + Y_ee * sin_alpha));

    float denominator = sqrt(A * A + B * B);
    if (denominator < 1e-6 || fabs(C / denominator) > 1.0) {
      return false;
    }

    float theta1 = atan2(B, A) + acos(-C / denominator);
    float theta2 = atan2(B, A) - acos(-C / denominator);

    		
    float theta;
    if (theta1 > theta2) {theta = theta2;}
    else {theta = theta1;}
    if (fabs(theta) < threshold) theta = 0;

    J[i] = -degrees(theta);  // Lưu vào mảng đầu ra
  }

  return true;
}

void Degree_1(float deg, float deg_old) {
  // Chỉ cập nhật hướng, không tính toán số xung ở đây nữa
  if (deg >= deg_old) {
    digitalWrite(DIR_PIN_1, HIGH);
    lastDir_1_encoder = 1;
  } else {
    digitalWrite(DIR_PIN_1, LOW);
    lastDir_1_encoder = -1;
  }
  deg1_old = deg; // Lưu góc mới
}
void Degree_2(float deg, float deg_old) {
  // Chỉ cập nhật hướng, không tính toán số xung ở đây nữa
  if (deg >= deg_old) {
    digitalWrite(DIR_PIN_2, HIGH);
    lastDir_2_encoder = 1;
  } else {
    digitalWrite(DIR_PIN_2, LOW);
    lastDir_2_encoder = -1;
  }
  deg2_old = deg; // Lưu góc mới
}
void Degree_3(float deg, float deg_old) {
  // Chỉ cập nhật hướng, không tính toán số xung ở đây nữa
  if (deg >= deg_old) {
    digitalWrite(DIR_PIN_3, HIGH);
    lastDir_3_encoder = 1;
  } else {
    digitalWrite(DIR_PIN_3, LOW);
    lastDir_3_encoder = -1;
  }
  deg3_old = deg; // Lưu góc mới
}
void Degree_1new(float deg, float deg_old) 
{
  if (deg_old >= deg) 
  {
    digitalWrite(DIR_PIN_1, 0);
    degree1 = abs(deg - deg_old);
    lastDir_1_encoder = -1;
  } 
  else
  {
    digitalWrite(DIR_PIN_1, 1);
    degree1 = abs(deg - deg_old);
    lastDir_1_encoder = 1;
  }
  nPulse_1 = degree1 * 8 * pulperrev / 360;
  if (nPulse_1 > 0) motorRunning_1 = true;
}
void Degree_2new(float deg, float deg_old) 
{
  if (deg_old >= deg) 
  {
    digitalWrite(DIR_PIN_2, 0);
    degree2 = abs(deg - deg_old);
    lastDir_2_encoder = -1;
  } 
  else
  {
    digitalWrite(DIR_PIN_2, 1);
    degree2 = abs(deg - deg_old);
    lastDir_2_encoder = 1;
  }
  nPulse_2 = degree2 * 8 * pulperrev / 360;
  if (nPulse_2 > 0) motorRunning_2 = true;
}
void Degree_3new(float deg, float deg_old) 
{
  if (deg_old >= deg) 
  {
    digitalWrite(DIR_PIN_3, 0);
    degree3 = abs(deg - deg_old);
    lastDir_3_encoder = -1;
  } 
  else
  {
    digitalWrite(DIR_PIN_3, 1);
    degree3 = abs(deg - deg_old);
    lastDir_3_encoder = 1;
  }
  nPulse_3 = degree3 * 8 * pulperrev / 360;
  if (nPulse_3 > 0) motorRunning_3 = true;
}
