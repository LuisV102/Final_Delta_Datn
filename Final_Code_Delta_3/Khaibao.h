#include "WString.h"
// Khai báo chân
#define PUL_PIN_1 22
#define DIR_PIN_1 24
#define PUL_PIN_2 26
#define DIR_PIN_2 28
#define PUL_PIN_3 30
#define DIR_PIN_3 32

// Encoder pins
#define pinA_1 2
#define pinB_1 3
#define pinA_2 18
#define pinB_2 19
#define pinA_3 20
#define pinB_3 21

#define READ_A_1 digitalRead(pinA_1)
#define READ_B_1 digitalRead(pinB_1)
#define READ_A_2 digitalRead(pinA_2)
#define READ_B_2 digitalRead(pinB_2)
#define READ_A_3 digitalRead(pinA_3)
#define READ_B_3 digitalRead(pinB_3)

String inString = "";

// Limit switches
const int limit_1 = 34;
const int limit_2 = 36;
const int limit_3 = 38;
const int namcham = 40;
const int bangtai = 42;

// Biến toàn cục
volatile long encoderPosition_1 = 0;
volatile long encoderPosition_2 = 0;
volatile long encoderPosition_3 = 0;

volatile int lastDir_1_encoder = 1;  // 1 = thuận, -1 = nghịch
volatile int lastDir_2_encoder = 1;  // 1 = thuận, -1 = nghịch
volatile int lastDir_3_encoder = 1; // 1 = thuận, -1 = nghịch

long encoderRawLast_1 = 0;
long encoderRawNow_1 = 0;
volatile long encoderRaw_2 = 0;
volatile long encoderRaw_3 = 0;      // Encoder chưa hiệu chỉnh (raw, đọc trong ISR)
long lastEncoderRaw_3 = 0;           // Dùng để tính delta ngoài ISR

volatile float encoderCalibrated_1 = 0.0;

// Biến điều khiển động cơ
volatile long nPulse_1 = 0, nPulse_2 = 0, nPulse_3 = 0;
volatile bool motorRunning_1 = false, motorRunning_2 = false, motorRunning_3 = false;
unsigned long delay_run_spd_1 = 2000, delay_run_spd_2 = 2000, delay_run_spd_3 = 2000;
unsigned long delay_home_spd = 5000;
const unsigned long pulperrev = 200; // Số xung trên mỗi vòng

// Góc hiện tại và góc đích
float deg1 = 0.0, deg1_old = 0.0, degree1 = 0.0;
float deg2 = 0.0, deg2_old = 0.0, degree2 = 0.0;
float deg3 = 0.0, deg3_old = 0.0, degree3 = 0.0;

// Quy hoạch quỹ đạo
float P0[3] = {0, 0, 0}; // Vị trí ban đầu
float Pf[3] = {0, 0, 0}; // Vị trí đích
float tf = 0.0; // Thời gian thực hiện
float t_up_down = 0.2;


// Khai báo biến toàn cục để lưu vị trí hiện tại
float currentPosition[3] = {0, 0, -307.38}; // Khởi tạo vị trí ban đầu

long delta_1 = 0;
bool enable_manual = true;
bool emergencyStop = false;  // Thêm biến này ở phần khai báo toàn cục
unsigned long now,previous,tf_end;