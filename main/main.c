#include <stdio.h>
#include "mpu6050.h"
#include "I2Cdev.h"
#include "PID.h"
#include "BTS7960.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void task_xylanh_mpu6050(void *pvParameters) {
	//khởi tạo xy lanh
// Khởi tạo các chân GPIO và PWM
    pwm_init();
    motor_direction_init();


    // Giả sử bạn đã lấy được giá trị setpoint và current_position từ cảm biến QMC5883
    float setpoint = 45.00;  // Ví dụ setpoint là 100


     pid_init_values(1.0, 0.1, setpoint);  // kp=1.0, kd=0.1
	//khởi tao mpu6050
	MPU6050 mpu;
	MPU6050_init(&mpu, MPU6050_DEFAULT_ADDRESS);

	uint8_t devid = MPU6050_getDeviceID(&mpu);

	uint8_t rate = MPU6050_getRate(&mpu);
	if (rate != 0) MPU6050_setRate(&mpu, 0);

	uint8_t ExternalFrameSync = MPU6050_getExternalFrameSync(&mpu);
	if (ExternalFrameSync != 0) MPU6050_setExternalFrameSync(&mpu, 0);;

	uint8_t DLPFMode = MPU6050_getDLPFMode(&mpu);
	if (DLPFMode != 6) MPU6050_setDLPFMode(&mpu, 6);

	uint8_t FullScaleAccelRange = MPU6050_getFullScaleGyroRange(&mpu);
	if (FullScaleAccelRange != 0) MPU6050_setFullScaleGyroRange(&mpu, 0); 
	float accel_sensitivity = 16384.0; // g

	uint8_t FullScaleGyroRange = MPU6050_getFullScaleAccelRange(&mpu);
	if (FullScaleGyroRange != 0) MPU6050_setFullScaleAccelRange(&mpu, 0);
	float gyro_sensitivity = 131.0; // Deg/Sec

	MPU6050_setI2CBypassEnabled(&mpu, true);

	double ax, ay, az;
    double gx, gy, gz;
    float Elevation;
    float prevElevation = 0; // Lưu trữ giá trị Elevation trước đó

    while (1) {
        MPU6050_getMotion6(&mpu, &ax, &ay, &az, &gx, &gy, &gz, accel_sensitivity, gyro_sensitivity);
        Elevation = MPU6050_getElevation(&mpu, ax, ay, az);
	//elevation là current_value
	 float speed = pid_compute(&pid, Elevation);
	//code điều khiển xylanh
    // Gọi hàm điều khiển xi lanh sử dụng PID
        uint8_t speed = control_cylinder_with_pid(current_position);
        // Kiểm tra thay đổi góc
        if (fabs(Elevation - prevElevation) >= 5.0) {
            printf(" Elevation = %f\n", Elevation);
            prevElevation = Elevation; // Cập nhật giá trị trước đó
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void start_i2c(void) {
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = (gpio_num_t)CONFIG_GPIO_SDA;
	conf.scl_io_num = (gpio_num_t)CONFIG_GPIO_SCL;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 400000;
	conf.clk_flags = 0;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

void app_main(void)
{
	start_i2c();

	xTaskCreate(task_mpu6050, "task_xylanh_mpu6050", 1024*8, NULL, 1, NULL);

	vTaskDelay(100);
}
