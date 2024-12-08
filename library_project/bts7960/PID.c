#include "pid.h"

// Khởi tạo PID với các tham số phù hợp
PID_Controller pid;

/**
 * @brief Hàm khởi tạo PID
 * 
 * @param kp Hệ số P của PID
 * @param kd Hệ số D của PID
 * @param setpoint Giá trị mục tiêu cho PID
 */
void pid_init_values(float kp, float kd, float setpoint) {
    pid = pid_init(kp, kd, setpoint);
}

/**
 * @brief Hàm điều khiển xi lanh bằng PID
 * 
 * @param current_position Vị trí hiện tại của xi lanh
 * @return Tốc độ PWM để điều khiển động cơ
 */
uint8_t control_cylinder_with_pid(float current_position) {
    // Tính toán đầu ra PID
    float pid_output = pid_compute(&pid, current_position);

    uint8_t speed = 0;
    if (pid_output > 0) {
        // Nếu PID output > 0, di chuyển xi lanh lên
        speed = (uint8_t)fmin(pid_output, 255); // Giới hạn tốc độ tối đa là 255
        lift_up(speed);
    } else if (pid_output < 0) {
        // Nếu PID output < 0, di chuyển xi lanh xuống
        speed = (uint8_t)fmin(-pid_output, 255); // Giới hạn tốc độ tối đa là 255
        lower_down(speed);
    } else {
        // Nếu PID output == 0, dừng xi lanh
        stop_cylinder();
    }

    return speed;
}


