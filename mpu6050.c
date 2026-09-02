#include "mpu6050.h"
#include <math.h>

static HAL_StatusTypeDef wr(I2C_HandleTypeDef *h, uint16_t addr,
                            uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(h, addr, reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1U, 10U);
}

static HAL_StatusTypeDef rd(I2C_HandleTypeDef *h, uint16_t addr,
                            uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(h, addr, reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, 10U);
}

bool mpu6050_init(I2C_HandleTypeDef *hi2c, uint16_t addr)
{
    uint8_t who = 0U;

    if (rd(hi2c, addr, MPU6050_REG_WHO_AM_I, &who, 1U) != HAL_OK)
        return false;

    if ((who != 0x68U) && (who != 0x69U) && (who != 0x70U))
        return false;

    if (wr(hi2c, addr, MPU6050_REG_PWR_MGMT1, 0x01U) != HAL_OK) return false;
    if (wr(hi2c, addr, MPU6050_REG_CONFIG,    0x03U) != HAL_OK) return false;
    if (wr(hi2c, addr, MPU6050_REG_SMPLRTDIV, 0x04U) != HAL_OK) return false;
    if (wr(hi2c, addr, MPU6050_REG_GYROCFG,   0x00U) != HAL_OK) return false;
    if (wr(hi2c, addr, MPU6050_REG_ACCCFG,    0x00U) != HAL_OK) return false;
    if (wr(hi2c, addr, MPU6050_REG_INT_CFG,   0x10U) != HAL_OK) return false;
    if (wr(hi2c, addr, MPU6050_REG_INT_EN,    0x01U) != HAL_OK) return false;

    return true;
}

bool mpu6050_read_raw(I2C_HandleTypeDef *hi2c, uint16_t addr,
                      mpu6050_raw_t *r)
{
    uint8_t b[14];

    if (rd(hi2c, addr, MPU6050_REG_ACCEL_XOUT_H, b, 14U) != HAL_OK)
        return false;

    r->ax   = (int16_t)(((uint16_t)b[0]  << 8) | b[1]);
    r->ay   = (int16_t)(((uint16_t)b[2]  << 8) | b[3]);
    r->az   = (int16_t)(((uint16_t)b[4]  << 8) | b[5]);
    r->temp = (int16_t)(((uint16_t)b[6]  << 8) | b[7]);
    r->gx   = (int16_t)(((uint16_t)b[8]  << 8) | b[9]);
    r->gy   = (int16_t)(((uint16_t)b[10] << 8) | b[11]);
    r->gz   = (int16_t)(((uint16_t)b[12] << 8) | b[13]);

    return true;
}

void mpu6050_convert(const mpu6050_raw_t *r, mpu6050_si_t *o,
                     float acc_lsb_per_g, float gyro_lsb_per_dps)
{
    o->ax_g = (float)r->ax / acc_lsb_per_g;
    o->ay_g = (float)r->ay / acc_lsb_per_g;
    o->az_g = (float)r->az / acc_lsb_per_g;

    o->gx_dps = (float)r->gx / gyro_lsb_per_dps;
    o->gy_dps = (float)r->gy / gyro_lsb_per_dps;
    o->gz_dps = (float)r->gz / gyro_lsb_per_dps;

    o->temp_c = (float)r->temp / 340.0f + 36.53f;
}

void mpu6050_complementary(const mpu6050_si_t *m, float dt_s,
                           float *pitch_deg, float *roll_deg, float alpha)
{
    float roll_acc =
        atan2f(m->ay_g, m->az_g) * 57.29578f;

    float pitch_acc =
        atan2f(-m->ax_g,
               sqrtf((m->ay_g * m->ay_g) + (m->az_g * m->az_g))) *
        57.29578f;

    *roll_deg =
        alpha * (*roll_deg + (m->gx_dps * dt_s)) +
        (1.0f - alpha) * roll_acc;

    *pitch_deg =
        alpha * (*pitch_deg + (m->gy_dps * dt_s)) +
        (1.0f - alpha) * pitch_acc;
}
