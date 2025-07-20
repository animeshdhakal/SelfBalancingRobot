#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "hal/i2c_types.h"
#include "soc/clk_tree_defs.h"
#include <math.h>
#include <stdint.h>
#include "pid.h"

#define TAG "Main"

#define ACC_X_OFFSET 0.0f
#define ACC_Y_OFFSET 0.0f
#define ACC_Z_OFFSET 0.0f

#define GYRO_X_OFFSET 0.0f
#define GYRO_Y_OFFSET 0.0f
#define GYRO_Z_OFFSET 0.0f

static void i2c_master_init(i2c_master_bus_handle_t *bus_handle,
                            i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .scl_io_num = 21,
        .sda_io_num = 22,
        .glitch_ignore_cnt = 7,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true
    };

    i2c_device_config_t mpu_config = {
        .device_address = 0x68,
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = 100000,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &mpu_config, dev_handle));
}


static void initalize_imu(i2c_master_dev_handle_t* dev_handle)  {
    uint8_t write_buf[] = {0x6B, 0};
    i2c_master_transmit(*dev_handle, write_buf, sizeof(write_buf), 1000);
}


static void get_imu_data(i2c_master_dev_handle_t* dev_handle, float* imu_data)  {
    uint8_t reg;
    uint8_t mpu_buf[14];

    reg = 0x3B; // ACCEL_XOUT_H 

    i2c_master_transmit_receive(*dev_handle, &reg, sizeof(reg), mpu_buf, sizeof(mpu_buf), 1000);

    int16_t a_x = mpu_buf[0] << 8 | mpu_buf[1];
    int16_t a_y = mpu_buf[2] << 8 | mpu_buf[3];
    int16_t a_z = mpu_buf[4] << 8 | mpu_buf[5];

    int16_t temp = mpu_buf[6] << 8 | mpu_buf[7];

    int16_t g_x = mpu_buf[8] << 8 | mpu_buf[9];
    int16_t g_y = mpu_buf[10] << 8 | mpu_buf[11];
    int16_t g_z = mpu_buf[12] << 8 | mpu_buf[13];
    
    imu_data[0] = (a_x / 16384.0f) + ACC_X_OFFSET;
    imu_data[1] = (a_y / 16384.0f) + ACC_Y_OFFSET;
    imu_data[2] = (a_z / 16384.0f) + ACC_Z_OFFSET;

    imu_data[3] = (g_x / 131.0f) + GYRO_Y_OFFSET;
    imu_data[4] = (g_y / 131.0f) + GYRO_Y_OFFSET;
    imu_data[5] = (g_z / 131.0f) + GYRO_Y_OFFSET;
}


void print_imu_error(i2c_master_dev_handle_t* dev_handle, int no_of_iterations)  {
    float acc_x=0, acc_y=0, acc_z=0, gyro_x=0, gyro_y=0, gyro_z=0;
    float imu_data[6];

    for(int i = 0; i < no_of_iterations; i++) {
        get_imu_data(dev_handle, imu_data);

        acc_x += imu_data[0];
        acc_y += imu_data[1];
        acc_z += imu_data[2];

        gyro_x += imu_data[3];
        gyro_y += imu_data[4];
        gyro_z += imu_data[5];
    }

    acc_x = acc_x / no_of_iterations;
    acc_y = acc_y / no_of_iterations;
    acc_z = acc_z / no_of_iterations;
    gyro_x = gyro_x / no_of_iterations;
    gyro_y = gyro_y / no_of_iterations;
    gyro_z = gyro_z / no_of_iterations;

    printf("IMU Offsets: \n");
    printf("%f %f %f \n", acc_x, acc_y, acc_z);
    printf("%f %f %f \n", gyro_x, gyro_y, gyro_z);
}

void app_main()
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t mpu_handle;

    i2c_master_init(&bus_handle, &mpu_handle);
    initalize_imu(&mpu_handle);

    ESP_LOGI(TAG, "MPU6050 Initialized Successfully");

    float imu_data[6];

    // print_imu_error(&mpu_handle, 200);

    pid_t pitch_pid;

    initialize_pid(&pitch_pid, 2.0f, 2.0f, 1.0f, 0.0f);

    while (1) {
        get_imu_data(&mpu_handle, imu_data);

        float roll = atan(imu_data[1] / sqrt(imu_data[0]*imu_data[0] + imu_data[2]*imu_data[2])) * (180 / M_PI);
        float pitch = atan(-imu_data[0] / sqrt(imu_data[1]*imu_data[1] + imu_data[2]*imu_data[2])) * (180 / M_PI);

        float pitch_response = calculate_pid_response(&pitch_pid, pitch);

        printf("%f %f\n", pitch, pitch_response);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
