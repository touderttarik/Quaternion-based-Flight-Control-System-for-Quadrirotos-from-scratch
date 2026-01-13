#include "sensors.h"

#include <stddef.h>

#define MPU6050_ADDR          0x68
#define MPU6050_REG_PWR_MGMT1 0x6B
#define MPU6050_REG_GYRO_CFG  0x1B
#define MPU6050_REG_ACCEL_CFG 0x1C
#define MPU6050_REG_DATA      0x3B

#define MPU6050_GYRO_1000DPS  0x10
#define MPU6050_ACCEL_8G      0x10

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;

static void mpu6050_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    i2c_master_transmit(s_dev, buf, sizeof(buf), 1000);
}

static void mpu6050_read_regs(uint8_t reg, uint8_t *data, size_t len) {
    i2c_master_transmit_receive(s_dev, &reg, 1, data, len, 1000);
}

void mpu6050_init(int scl_gpio, int sda_gpio, uint32_t freq_hz) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .scl_io_num = scl_gpio,
        .sda_io_num = sda_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };

    i2c_new_master_bus(&bus_cfg, &s_bus);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = freq_hz,
        .scl_wait_us = 10
    };

    i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);

    mpu6050_write_reg(MPU6050_REG_PWR_MGMT1, 0x00);
    mpu6050_write_reg(MPU6050_REG_ACCEL_CFG, MPU6050_ACCEL_8G);
    mpu6050_write_reg(MPU6050_REG_GYRO_CFG, MPU6050_GYRO_1000DPS);
}

void read_sensors(sensor_data_t *out) {
    uint8_t data[14] = {0};
    mpu6050_read_regs(MPU6050_REG_DATA, data, sizeof(data));

    int16_t ax = (int16_t)((data[0] << 8) | data[1]);
    int16_t ay = (int16_t)((data[2] << 8) | data[3]);
    int16_t az = (int16_t)((data[4] << 8) | data[5]);

    int16_t gx = (int16_t)((data[8] << 8) | data[9]);
    int16_t gy = (int16_t)((data[10] << 8) | data[11]);
    int16_t gz = (int16_t)((data[12] << 8) | data[13]);

    const float accel_scale = 1.0f / 4096.0f;        // +/- 8g
    const float gyro_scale = 0.01745329252f / 32.8f; // +/- 1000 deg/s -> rad/s

    out->accel.x = ax * accel_scale;
    out->accel.y = ay * accel_scale;
    out->accel.z = az * accel_scale;

    out->gyro.x = gx * gyro_scale;
    out->gyro.y = gy * gyro_scale;
    out->gyro.z = gz * gyro_scale;
}
