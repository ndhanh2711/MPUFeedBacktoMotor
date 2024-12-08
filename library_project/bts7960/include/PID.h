#ifndef PID_H
#define PID_H

#include <stdint.h>

// Cấu trúc PID
typedef struct {
    float kp;      // Hệ số P (Proportional)
    float kd;      // Hệ số D (Derivative)
    float setpoint; // Giá trị mục tiêu
    float previous_error; // Sai số ở lần đo trước
} PID_Controller;

/**
 * @brief Khởi tạo cấu trúc PID
 * 
 * @param kp Hệ số P
 * @param kd Hệ số D
 * @param setpoint Giá trị mục tiêu
 * @return PID_Controller Cấu trúc PID được khởi tạo
 */
PID_Controller pid_init(float kp, float kd, float setpoint);

/**
 * @brief Tính toán đầu ra của PID
 * 
 * @param pid Con trỏ đến cấu trúc PID
 * @param current_value Giá trị hiện tại từ cảm biến
 * @return float Giá trị điều khiển đầu ra
 */
float pid_compute(PID_Controller *pid, float current_value);

#endif // PID_H
