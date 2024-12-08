#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Định nghĩa các chân kết nối với BTS7960
#define REN_PIN    GPIO_NUM_2   // Chân điều khiển chiều nâng
#define LEN_PIN    GPIO_NUM_4   // Chân điều khiển chiều hạ
#define RPWM_PIN   GPIO_NUM_18  // Chân PWM cho chiều nâng
#define LPWM_PIN   GPIO_NUM_19  // Chân PWM cho chiều hạ

// Các biến điều khiển PWM
#define PWM_FREQUENCY 5000    // Tần số PWM
#define PWM_RESOLUTION LEDC_TIMER_8_BIT  // Độ phân giải PWM (0-255)
#define PWM_MAX_DUTY 255     // Giá trị PWM tối đa

/**
 * @brief Hàm khởi tạo PWM
 * 
 * Thiết lập các kênh PWM để điều khiển BTS7960 ở hai chiều
 * (nâng và hạ) qua RPWM và LPWM.
 */
void pwm_init() {
    // Cấu hình timer PWM
    ledc_timer_config_t timer_config = {
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
    };
    ledc_timer_config(&timer_config);

    // Cấu hình kênh PWM cho RPWM (chiều nâng xi lanh)
    ledc_channel_config_t channel_config_rpwm = {
        .channel = LEDC_CHANNEL_0,
        .gpio_num = RPWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,  // Bắt đầu với giá trị PWM là 0
        .hpoint = 0,
    };
    ledc_channel_config(&channel_config_rpwm);

    // Cấu hình kênh PWM cho LPWM (chiều hạ xi lanh)
    ledc_channel_config_t channel_config_lpwm = {
        .channel = LEDC_CHANNEL_1,
        .gpio_num = LPWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,  // Bắt đầu với giá trị PWM là 0
        .hpoint = 0,
    };
    ledc_channel_config(&channel_config_lpwm);
}

/**
 * @brief Hàm khởi tạo chân điều khiển
 * 
 * Đặt REN và LEN ở chế độ OUTPUT và thiết lập mức logic ban đầu.
 */
void motor_direction_init() {
    gpio_set_direction(REN_PIN, GPIO_MODE_OUTPUT); // Chân điều khiển chiều nâng
    gpio_set_direction(LEN_PIN, GPIO_MODE_OUTPUT); // Chân điều khiển chiều hạ

    // Đặt mức thấp ban đầu để đảm bảo xi lanh không hoạt động
    gpio_set_level(REN_PIN, 0);
    gpio_set_level(LEN_PIN, 0);
}

/**
 * @brief Hàm nâng xi lanh
 * 
 * @param speed Giá trị tốc độ PWM (0-255)
 * 
 * Kích hoạt chân nâng (REN) và điều chỉnh tốc độ bằng PWM qua RPWM.
 */
void lift_up(uint8_t speed) {
    gpio_set_level(REN_PIN, 1);  // Kích hoạt chiều nâng
    gpio_set_level(LEN_PIN, 0);  // Đảm bảo chiều hạ tắt

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, speed);  // PWM cho nâng
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);  // Đảm bảo chiều hạ dừng
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

/**
 * @brief Hàm hạ xi lanh
 * 
 * @param speed Giá trị tốc độ PWM (0-255)
 * 
 * Kích hoạt chân hạ (LEN) và điều chỉnh tốc độ bằng PWM qua LPWM.
 */
void lower_down(uint8_t speed) {
    gpio_set_level(REN_PIN, 0);  // Đảm bảo chiều nâng tắt
    gpio_set_level(LEN_PIN, 1);  // Kích hoạt chiều hạ

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);  // Đảm bảo chiều nâng dừng
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, speed);  // PWM cho hạ
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

/**
 * @brief Hàm dừng xi lanh
 * 
 * Đặt cả hai chân điều khiển về mức thấp và dừng PWM.
 */
void stop_cylinder() {
    gpio_set_level(REN_PIN, 0);  // Dừng chiều nâng
    gpio_set_level(LEN_PIN, 0);  // Dừng chiều hạ

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);  // Dừng PWM cho nâng
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);  // Dừng PWM cho hạ
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}
