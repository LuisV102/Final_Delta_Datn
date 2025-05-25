void SetPosition_1()
{
  detachInterrupt(digitalPinToInterrupt(pinA_1)); 
  detachInterrupt(digitalPinToInterrupt(pinB_1));
  deg1 = 0.0;
  deg1_old = 0.0;
  degree1 = 0.0;

  encoderRawLast_1 = 0;
  encoderRawNow_1 = 0;
  delta_1 = 0;
  
  encoderPosition_1 = 0;
  encoderCalibrated_1 = 0;
  nPulse_1 = 0; 
  delayMicroseconds(500);
  attachInterrupt(digitalPinToInterrupt(pinA_1), handleA_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB_1), handleB_1, CHANGE);
}
void SetPosition_2()
{
  deg2 = 0.0;
  deg2_old = 0.0;
  degree2 = 0.0;

  detachInterrupt(digitalPinToInterrupt(pinA_2)); 
  detachInterrupt(digitalPinToInterrupt(pinB_2));
  encoderPosition_2 = 0;
  nPulse_2 = 0; 
  delayMicroseconds(500);
  attachInterrupt(digitalPinToInterrupt(pinA_2), handleA_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB_2), handleB_2, CHANGE);
}
void SetPosition_3()
{
  deg3 = 0.0;
  deg3_old = 0.0;
  degree3 = 0.0;
  detachInterrupt(digitalPinToInterrupt(pinA_3)); 
  detachInterrupt(digitalPinToInterrupt(pinB_3));
  delayMicroseconds(100); // hoặc nhỏ hơn nếu cần

  encoderPosition_3 = 0;
  nPulse_3 = 0;
  delayMicroseconds(500);
  attachInterrupt(digitalPinToInterrupt(pinA_3), handleA_3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB_3), handleB_3, CHANGE);
}
