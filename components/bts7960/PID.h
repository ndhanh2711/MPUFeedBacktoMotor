#ifndef PID_H
#define PID_H

#include <stdint.h>

// Cấu trúc PID
typedef struct
{                         // Ở đây chỉ có P và D thôi  ??ok ạ bạn không dùng i sao ?em được hướng dẫn chỉ dùng pd thôi ạ
    float kp;             // Hệ số P (Proportional) hồi xưa mình cũng không rành giải thuật PID lắm =)) nên không rõ vì sao phải dùng PD thôi
    float kd;             // Hệ số D (Derivative)
    float setpoint;       // Giá trị mục tiêu, tại thời điểm khởi động thì nên viết code theo kiểu sẽ init cho phânf giá trị hiện tại nghe sẽ hợp lý
                          // Ở đây bạn chưa có biến cho giá trị hiện tại ? hay bạn sẽ sử lý nó trực tiếp luôn ?, giá trị hiện tại em lấy từ cái mpu6050 ạ
    float curpoint;       // Bạn có thể bỏ nó vào để đây để check, khi cần truyền thì bạn cập nhật giá trị này luôn
    float previous_error; // Sai số ở lần đo trước
} PID_Controller;

/**
 * @brief Khởi tạo cấu trúc PID
 *
 * @param kp Hệ số P
 * @param kd Hệ số D
 * @param curpoint Giá trị hiện tại cho PID
 * @return PID_Controller Cấu trúc PID được khởi tạo
 */
void pid_init(float kp, float kd, float curpoint);

void pid_set_target_value(float setpoint);

/**
 * @brief Tính toán đầu ra của PID
 *
 * @param pid Con trỏ đến cấu trúc PID
 * @return float Giá trị điều khiển đầu ra
 */
float pid_compute(PID_Controller *pid);

int16_t control_cylinder_with_pid(float current_position);
// Chờ mình tí

#endif // PID_H
