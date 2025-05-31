// PID.h
#ifndef PID_H
#define PID_H

class SimplePID {
private:
    float kp, ki, kd;
    float integral = 0;
    float prevError = 0;
    unsigned long prevTime = 0;
    
public:
    SimplePID(float p, float i, float d) : kp(p), ki(i), kd(d) {}
    
    float compute(float setpoint, float input) {
        unsigned long now = micros();
        float dt = (now - prevTime) * 1e-6f; // Chuyển đổi sang giây
        if (dt <= 0) dt = 1e-3f; // Tránh chia cho 0
        
        float error = setpoint - input;
        
        // Tính thành phần P
        float proportional = kp * error;
        
        // Tính thành phần I
        integral += ki * error * dt;
        
        // Tính thành phần D
        float derivative = kd * (error - prevError) / dt;
        
        // Tổng hợp đầu ra
        float output = proportional + integral + derivative;
        
        // Cập nhật trạng thái
        prevError = error;
        prevTime = now;
        
        return output;
    }
    
    void reset() {
        integral = 0;
        prevError = 0;
        prevTime = micros();
    }
};

#endif