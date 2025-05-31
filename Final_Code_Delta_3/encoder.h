void handleA_1() {
  int delta = 0;
  if (READ_A_1) {
    if (READ_B_1) delta = -1;
    else delta = +1;
  } else {
    if (READ_B_1) delta = +1;
    else delta = -1;
  }
  encoderPosition_1 += delta;
}
void handleB_1() {
  int delta = 0;
  if (READ_B_1) {
    if (READ_A_1) delta = +1;
    else delta = -1;
  } else {
    if (READ_A_1) delta = -1;
    else delta = +1;
  }
  encoderPosition_1 += delta;
}
void handleA_2() {
  if (READ_A_2) {
    if (READ_B_2) encoderPosition_2 -= 1;
    else encoderPosition_2 += 1;
  } else {
    if (READ_B_2) encoderPosition_2 += 1;
    else encoderPosition_2 -= 1;
  }
}
void handleB_2() {
  if (READ_B_2) {
    if (READ_A_2) encoderPosition_2 += 1;
    else encoderPosition_2 -= 1;
  } else {
    if (READ_A_2) encoderPosition_2 -= 1;
    else encoderPosition_2 += 1;
  }
}

void handleA_3() {
  int delta = 0;
  if (READ_A_3) {
    if (READ_B_3) delta = -1;
    else delta = +1;
  } else {
    if (READ_B_3) delta = +1;
    else delta = -1;
  }
  encoderPosition_3 += delta;
}
void handleB_3() {
  int delta = 0;
  if (READ_B_3) {
    if (READ_A_3) delta = +1;
    else delta = -1;
  } else {
    if (READ_A_3) delta = -1;
    else delta = +1;
  }
  encoderPosition_3 += delta;
}