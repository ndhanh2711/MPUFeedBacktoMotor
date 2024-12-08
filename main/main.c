#include <stdio.h>
#include "BTS7960.h"
#include "PID.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main() {
    pwm_init();
    motor_direction_init();

    float setpoint = 100.0;   // Giá trị mong muốn
    float current_value = 50.0;  // Giá trị hiện tại
    float Kp = 1.0, Kd = 0.1;

    while (1) {
        current_value = read_sensor_value(); // Cập nhật giá trị cảm biến
        float pid_output = pd_update(setpoint, current_value, Kp, Kd);

        // Giới hạn giá trị đầu ra
        if (pid_output > PWM_MAX_DUTY) pid_output = PWM_MAX_DUTY;
        if (pid_output < -PWM_MAX_DUTY) pid_output = -PWM_MAX_DUTY;

        if (pid_output > 0) {
            move_forward((uint8_t)pid_output);
        } else if (pid_output < 0) {
            turn_left((uint8_t)(-pid_output));
        } else {
            stop_motors();
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
