/*
 * mpu6050.h
 *
 *  Created on: Jun 23, 2026
 *      Author: rejin
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f4xx_hal.h"

/* ── I2C Address ───────────────────────────────────────
 * AD0 pin LOW  → 0x68 → 8-bit address = 0xD0
 * AD0 pin HIGH → 0x69 → 8-bit address = 0xD2
 * HAL uses 8-bit addresses so we shift left by 1
 */
#define MPU6050_I2C_ADDR        (0x68 << 1)   // = 0xD0

/* ── Register Map ──────────────────────────────────────
 * These are the register addresses from the MPU6050
 * datasheet. Writing/reading these controls the chip.
 */
#define MPU6050_REG_SMPLRT_DIV   0x19  // Sample rate divider
#define MPU6050_REG_CONFIG       0x1A  // DLPF config
#define MPU6050_REG_GYRO_CONFIG  0x1B  // Gyro full-scale range
#define MPU6050_REG_ACCEL_CONFIG 0x1C  // Accel full-scale range
#define MPU6050_REG_INT_ENABLE   0x38  // Interrupt enable
#define MPU6050_REG_ACCEL_XOUT_H 0x3B  // First data register
#define MPU6050_REG_TEMP_OUT_H   0x41  // Temperature high byte
#define MPU6050_REG_GYRO_XOUT_H  0x43  // Gyro X high byte
#define MPU6050_REG_PWR_MGMT_1   0x6B  // Power management
#define MPU6050_REG_WHO_AM_I     0x75  // Device ID (returns 0x68)

/* ── Sensitivity Scales ────────────────────────────────
 * These come from the datasheet sensitivity table.
 * Dividing raw value by these gives physical units.
 */
#define MPU6050_ACCEL_SENS_2G    16384.0f  // LSB/g  at ±2g
#define MPU6050_ACCEL_SENS_4G    8192.0f   // LSB/g  at ±4g
#define MPU6050_GYRO_SENS_250    131.0f    // LSB/°/s at ±250°/s
#define MPU6050_GYRO_SENS_500    65.5f     // LSB/°/s at ±500°/s

/* ── Data Structure ────────────────────────────────────
 * This struct holds one complete reading from the sensor.
 * Pass a pointer to this into the read function.
 */
typedef struct {
    float accel_x;      // acceleration X axis in g
    float accel_y;      // acceleration Y axis in g
    float accel_z;      // acceleration Z axis in g
    float gyro_x;       // angular velocity X in deg/s
    float gyro_y;       // angular velocity Y in deg/s
    float gyro_z;       // angular velocity Z in deg/s
    float temperature;  // temperature in Celsius
} MPU6050_Data_t;

/* ── Return Status ─────────────────────────────────────
 * Functions return this so caller knows if it worked.
 */
typedef enum {
    MPU6050_OK    = 0,
    MPU6050_ERROR = 1
} MPU6050_Status_t;

/* ── Public API ────────────────────────────────────────
 * These are the only functions you call from outside
 * this driver. Everything else is internal.
 */
MPU6050_Status_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
MPU6050_Status_t MPU6050_ReadAll(I2C_HandleTypeDef *hi2c,
                                  MPU6050_Data_t *data);
MPU6050_Status_t MPU6050_ReadAccel(I2C_HandleTypeDef *hi2c,
                                    MPU6050_Data_t *data);
MPU6050_Status_t MPU6050_ReadGyro(I2C_HandleTypeDef *hi2c,
                                   MPU6050_Data_t *data);
MPU6050_Status_t MPU6050_ReadTemp(I2C_HandleTypeDef *hi2c,
                                   MPU6050_Data_t *data);

#endif /* INC_MPU6050_H_ */

