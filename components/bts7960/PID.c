#include "pid.h"
#include "BTS7960.h"
// Khởi tạo PID với các tham số phù hợp
PID_Controller pid;

/**
 * @brief Hàm khởi tạo PID
 *
 * @param kp Hệ số P của PID
 * @param kd Hệ số D của PID
 * @param curpoint Giá trị hiện tại cho PID
 */
void pid_init(float kp, float kd, float dt, float curpoint)
{ 
    pid.kp = kp;
    pid.kd = kd; 
    pid.dt = dt;
    pid.curpoint = curpoint;
    pid.setpoint = curpoint;
    pid.previous_error = 0.0;
}

void pid_set_target_value(float setpoint)
{
    pid.setpoint = setpoint;
}

float pid_compute(PID_Controller *pid) // Đầu ra của hàm PID 
{
    float output = 0.0;
    float error = pid->setpoint - pid->curpoint;

        // Tính đạo hàm của sai số
    float derivative_error = (error - pid->previous_error) / pid->dt;


    // Tính output với PD
    output = (pid->kp * error) + (pid->kd * derivative_error);


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
    // Tính toán đầu ra PID, 
    pid.curpoint = current_position;
    float pid_output = pid_compute(&pid);

   float speed = 0; // 
    if (pid_output > 0.1f)
    {
        // Nếu PID output > 0, di chuyển xi lanh lên 
        speed = 255; //(pid_output > -0.0f ? -255.0f : pid_output); // Giới hạn tốc độ tối đa là 255
        // lift_up(speed); -> viết code
        lift_up(speed);
    }
    else if (pid_output < -0.1f)
    {
        // Nếu PID output < 0, di chuyển xi lanh xuống
        speed = 255; //(pid_output < -0.0f ? -255.0f : pid_output); // Giới hạn tốc độ tối đa là 255
        // lower_down(speed); -> viết code
        lower_down(speed);
    }
    else
    {
        // Nếu PID output == 0, dừng xi lanh
        // stop_cylinder(); -> viết code
        stop_cylinder();
    }

    return speed;
}
