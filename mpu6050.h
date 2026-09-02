/*
 * E.Manganyi
 */

#pragma once
#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define MPU6050_I2C_ADDR_68   (0x68 << 1)
#define MPU6050_I2C_ADDR_70   (0x70 << 1)

#define MPU6050_REG_WHO_AM_I  0x75
#define MPU6050_REG_PWR_MGMT1 0x6B
#define MPU6050_REG_SMPLRTDIV 0x19
#define MPU6050_REG_CONFIG    0x1A
#define MPU6050_REG_GYROCFG   0x1B
#define MPU6050_REG_ACCCFG    0x1C
#define MPU6050_REG_INT_EN    0x38
#define MPU6050_REG_INT_CFG   0x37
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

typedef struct {
  int16_t ax, ay, az;
  int16_t temp;
  int16_t gx, gy, gz;
} mpu6050_raw_t;

typedef struct {
  float ax_g, ay_g, az_g;
  float gx_dps, gy_dps, gz_dps;
  float temp_c;
} mpu6050_si_t;

bool  mpu6050_init(I2C_HandleTypeDef *hi2c, uint16_t addr);
bool  mpu6050_read_raw(I2C_HandleTypeDef *hi2c, uint16_t addr, mpu6050_raw_t *r);
void  mpu6050_convert(const mpu6050_raw_t *r, mpu6050_si_t *o,
                      float acc_lsb_per_g, float gyro_lsb_per_dps);

// Simple complementary filter for pitch/roll (°)
void  mpu6050_complementary(const mpu6050_si_t *m, float dt_s,
                            float *pitch_deg, float *roll_deg, float alpha);
