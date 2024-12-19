#include <stdio.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/gpio.h"
// Định nghĩa các chân kết nối với BTS7960
#define R_IS_GPIO 25
#define R_EN_GPIO 2
#define R_PWM_CHANNEL LEDC_CHANNEL_0
#define R_PWM_PIN GPIO_NUM_18

#define L_IS_GPIO 26
#define L_EN_GPIO 4
#define L_PWM_CHANNEL LEDC_CHANNEL_1
#define L_PWM_PIN GPIO_NUM_19
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
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL<<R_IS_GPIO) | (1ULL<<R_EN_GPIO) | (1ULL<<R_PWM_PIN) |
                        (1ULL<<L_IS_GPIO) | (1ULL<<L_EN_GPIO) | (1ULL<<L_PWM_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_PIN_INTR_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);

    gpio_set_level(R_IS_GPIO, 0);
    gpio_set_level(L_IS_GPIO, 0);
    gpio_set_level(R_EN_GPIO, 1);
    gpio_set_level(L_EN_GPIO, 1);

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t pwm_channel = {
        .gpio_num = R_PWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = R_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&pwm_channel);

    pwm_channel.gpio_num = L_PWM_PIN;
    pwm_channel.channel = L_PWM_CHANNEL;
    ledc_channel_config(&pwm_channel);
}

/**
 * @brief Hàm khởi tạo chân điều khiển
 * 
 * Đặt REN và LEN ở chế độ OUTPUT và thiết lập mức logic ban đầu.
 */
/**
 * @brief Hàm nâng xi lanh
 * 
 * @param speed Giá trị tốc độ PWM (0-255)
 * 
 * Kích hoạt chân nâng (REN) và điều chỉnh tốc độ bằng PWM qua RPWM.
 */
void lift_up(uint8_t speed) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, R_PWM_CHANNEL, speed);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, R_PWM_CHANNEL);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, L_PWM_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, L_PWM_CHANNEL);
}

/**
 * @brief Hàm hạ xi lanh
 * 
 * @param speed Giá trị tốc độ PWM (0-255)
 * 
 * Kích hoạt chân hạ (LEN) và điều chỉnh tốc độ bằng PWM qua LPWM.
 */
void lower_down(uint8_t speed) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, R_PWM_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, R_PWM_CHANNEL);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, L_PWM_CHANNEL, speed);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, L_PWM_CHANNEL);
}

/**
 * @brief Hàm dừng xi lanh
 * 
 * Đặt cả hai chân điều khiển về mức thấp và dừng PWM.
 */
void stop_cylinder() {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, R_PWM_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, R_PWM_CHANNEL);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, L_PWM_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, L_PWM_CHANNEL);
}
