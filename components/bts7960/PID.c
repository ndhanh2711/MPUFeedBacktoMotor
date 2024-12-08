#include "pid.h"

// Khởi tạo PID với các tham số phù hợp
PID_Controller pid;

/**
 * @brief Hàm khởi tạo PID
 *
 * @param kp Hệ số P của PID
 * @param kd Hệ số D của PID
 * @param curpoint Giá trị hiện tại cho PID
 */
void pid_init(float kp, float kd, float curpoint)
{ // Hàm này bạn chỉ để trong file .c nên idf nó không hiểu phải tìm ở đâu
    pid.kp = kp;
    pid.kd = kd; // Setpoint ở đây của bạn là giá trị thực actual value ?, cái này em làm nó là nâng hạ cho chảo thu sóng ạ, thì set point ỏ đây là góc độ cao, mà góc độ cao được xác định bằng một bạn khác nhưng chưa cho vào đây nên tạm để giả định ạ
    pid.curpoint = curpoint;
    pid.setpoint = curpoint;
}

void pid_set_target_value(float setpoint)
{
    pid.setpoint = setpoint;
}

float pid_compute(PID_Controller *pid)
{
    float output = 0.0f;
    float error = pid->setpoint - pid->curpoint;
    // Bạn bỏ thuật toán PD của bạn vào
    pid->previous_error = error;
    return output;
}

/**
 * @brief Hàm điều khiển xi lanh bằng PID
 *
 * @param current_position Vị trí hiện tại của xi lanh
 * @return Tốc độ PWM để điều khiển động cơ
 */
int16_t control_cylinder_with_pid(float current_position)
{
    // Tính toán đầu ra PID
    pid.curpoint = current_position;
    float pid_output = pid_compute(&pid);

   float speed = 0; // vậy để float ? vâng
    if (pid_output > 0.1f)
    {
        // Nếu PID output > 0, di chuyển xi lanh lên 
        speed = (pid_output > 255.0f ? 255.0f : pid_output); // Giới hạn tốc độ tối đa là 255
        // lift_up(speed); -> viết code
    }
    else if (pid_output < -0.1f)
    {
        // Nếu PID output < 0, di chuyển xi lanh xuống
        speed = (pid_output < -255.0f ? -255.0f : pid_output); // Giới hạn tốc độ tối đa là 255
        // lower_down(speed); -> viết code
    }
    else
    {
        // Nếu PID output == 0, dừng xi lanh
        // stop_cylinder(); -> viết code
    }

    return speed;
}
