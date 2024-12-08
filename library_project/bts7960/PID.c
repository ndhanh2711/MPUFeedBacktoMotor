#include "pid.h"

/**
 * @brief Khởi tạo cấu trúc PID
 */
PID_Controller pid_init(float kp, float kd, float setpoint) {
    PID_Controller pid;
    pid.kp = kp;
    pid.kd = kd;
    pid.setpoint = setpoint;
    pid.previous_error = 0.0;
    return pid;
}

/**
 * @brief Tính toán đầu ra của PID
 */
float pid_compute(PID_Controller *pid, float current_value) {
    // Tính sai số hiện tại
    float error = pid->setpoint - current_value;

    // Thành phần Proportional (P)
    float proportional = pid->kp * error;

    // Thành phần Derivative (D)
    float derivative = pid->kd * (error - pid->previous_error);

    // Cập nhật sai số trước đó
    pid->previous_error = error;

    // Đầu ra PID
    return proportional + derivative;
}
