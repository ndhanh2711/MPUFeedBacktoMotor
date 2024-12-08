#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

// Định nghĩa chân kết nối với BTS7960
#define REN_PIN    GPIO_NUM_2   // Chân điều khiển chiều nâng
#define LEN_PIN    GPIO_NUM_4   // Chân điều khiển chiều hạ
#define RPWM_PIN   GPIO_NUM_18  // Chân PWM cho chiều nâng
#define LPWM_PIN   GPIO_NUM_19  // Chân PWM cho chiều hạ

// Định nghĩa các tham số PWM
#define PWM_FREQUENCY 5000         // Tần số PWM
#define PWM_RESOLUTION LEDC_TIMER_8_BIT  // Độ phân giải PWM (0-255)
#define PWM_MAX_DUTY 255           // Giá trị PWM tối đa

/**
 * @brief Khởi tạo PWM
 * 
 * Cấu hình các kênh và timer PWM để điều khiển BTS7960.
 */
void pwm_init(void);

/**
 * @brief Khởi tạo các chân điều khiển xi lanh
 * 
 * Thiết lập các chân điều khiển (REN, LEN) ở chế độ OUTPUT.
 */
void motor_direction_init(void);

/**
 * @brief Nâng xi lanh
 * 
 * @param speed Giá trị tốc độ PWM (0-255)
 * 
 * Kích hoạt chiều nâng xi lanh với tốc độ điều chỉnh qua PWM.
 */
void lift_up(uint8_t speed);

/**
 * @brief Hạ xi lanh
 * 
 * @param speed Giá trị tốc độ PWM (0-255)
 * 
 * Kích hoạt chiều hạ xi lanh với tốc độ điều chỉnh qua PWM.
 */
void lower_down(uint8_t speed);

/**
 * @brief Dừng xi lanh
 * 
 * Dừng cả hai chiều (nâng và hạ) và tắt tín hiệu PWM.
 */
void stop_cylinder(void);

#endif // MOTOR_CONTROL_H
