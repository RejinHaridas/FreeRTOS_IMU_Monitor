/*
 * mpu6050.c
 *
 *  Created on: Jun 23, 2026
 *      Author: rejin
 */


#include "mpu6050.h"

/* ════════════════════════════════════════════════════════
 * INTERNAL HELPER FUNCTIONS
 * These are static — not visible outside this file.
 * ════════════════════════════════════════════════════════ */

/* Write a single byte to a register */
static MPU6050_Status_t MPU6050_WriteReg(I2C_HandleTypeDef *hi2c,
                                          uint8_t reg,
                                          uint8_t value) {
    HAL_StatusTypeDef result = HAL_I2C_Mem_Write(
        hi2c,
        MPU6050_I2C_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        HAL_MAX_DELAY
    );
    return (result == HAL_OK) ? MPU6050_OK : MPU6050_ERROR;
}

/* Read multiple bytes starting from a register */
static MPU6050_Status_t MPU6050_ReadRegs(I2C_HandleTypeDef *hi2c,
                                          uint8_t reg,
                                          uint8_t *buf,
                                          uint8_t len) {
    HAL_StatusTypeDef result = HAL_I2C_Mem_Read(
        hi2c,
        MPU6050_I2C_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        buf,
        len,
        HAL_MAX_DELAY
    );
    return (result == HAL_OK) ? MPU6050_OK : MPU6050_ERROR;
}

/* Combine two bytes into a signed 16-bit integer
 *
 * MPU6050 sends data as two separate bytes:
 * HIGH byte first, then LOW byte.
 * We combine them like this:
 *
 * raw = (HIGH << 8) | LOW
 *
 * Cast to int16_t so negative values work correctly
 * (two's complement)
 */
static int16_t CombineBytes(uint8_t high, uint8_t low) {
    return (int16_t)((high << 8) | low);
}


/* ════════════════════════════════════════════════════════
 * MPU6050_Init
 *
 * Call this once at startup before reading any data.
 * It:
 *   1. Checks the device is actually there (WHO_AM_I)
 *   2. Wakes the chip up (it starts in sleep mode)
 *   3. Configures sample rate, accel range, gyro range
 * ════════════════════════════════════════════════════════ */
MPU6050_Status_t MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t who_am_i = 0;

    /* Step 1 — Verify device identity
     * WHO_AM_I register always returns 0x68
     * If it returns anything else, wiring is wrong
     * or device is not responding
     */
    if (MPU6050_ReadRegs(hi2c, MPU6050_REG_WHO_AM_I,
                         &who_am_i, 1) != MPU6050_OK) {
        return MPU6050_ERROR;  // I2C transaction failed
    }
    if (who_am_i != 0x68) {
        return MPU6050_ERROR;  // Wrong device or wrong address
    }

    /* Step 2 — Wake up the chip
     * PWR_MGMT_1 register controls power state.
     * Bit 6 (SLEEP) is set by default on power-on.
     * Writing 0x00 clears all bits → chip wakes up,
     * uses internal 8MHz oscillator as clock source.
     */
    if (MPU6050_WriteReg(hi2c, MPU6050_REG_PWR_MGMT_1,
                         0x00) != MPU6050_OK) {
        return MPU6050_ERROR;
    }

    /* Small delay after wakeup — let oscillator stabilize */
    HAL_Delay(100);

    /* Step 3 — Set sample rate
     * Formula: Sample Rate = Gyro Rate / (1 + SMPLRT_DIV)
     * Gyro rate = 1kHz (when DLPF enabled)
     * SMPLRT_DIV = 7 → Sample Rate = 1000 / 8 = 125 Hz
     * Meaning: sensor data updates 125 times per second
     */
    if (MPU6050_WriteReg(hi2c, MPU6050_REG_SMPLRT_DIV,
                         0x07) != MPU6050_OK) {
        return MPU6050_ERROR;
    }

    /* Step 4 — Configure Digital Low Pass Filter (DLPF)
     * CONFIG register bits [2:0] set DLPF bandwidth.
     * 0x01 → 184Hz bandwidth, 2ms delay
     * This filters out high-frequency noise from readings.
     * Gyro output rate becomes 1kHz when DLPF is enabled.
     */
    if (MPU6050_WriteReg(hi2c, MPU6050_REG_CONFIG,
                         0x01) != MPU6050_OK) {
        return MPU6050_ERROR;
    }

    /* Step 5 — Set accelerometer range
     * ACCEL_CONFIG bits [4:3] (FS_SEL) set full-scale range:
     * 00 → ±2g  (sensitivity: 16384 LSB/g)  ← we use this
     * 01 → ±4g  (sensitivity:  8192 LSB/g)
     * 10 → ±8g  (sensitivity:  4096 LSB/g)
     * 11 → ±16g (sensitivity:  2048 LSB/g)
     * 0x00 → FS_SEL = 00 → ±2g range
     */
    if (MPU6050_WriteReg(hi2c, MPU6050_REG_ACCEL_CONFIG,
                         0x00) != MPU6050_OK) {
        return MPU6050_ERROR;
    }

    /* Step 6 — Set gyroscope range
     * GYRO_CONFIG bits [4:3] (FS_SEL):
     * 00 → ±250  deg/s (sensitivity: 131   LSB/deg/s) ← we use
     * 01 → ±500  deg/s (sensitivity:  65.5 LSB/deg/s)
     * 10 → ±1000 deg/s (sensitivity:  32.8 LSB/deg/s)
     * 11 → ±2000 deg/s (sensitivity:  16.4 LSB/deg/s)
     * 0x00 → FS_SEL = 00 → ±250 deg/s
     */
    if (MPU6050_WriteReg(hi2c, MPU6050_REG_GYRO_CONFIG,
                         0x00) != MPU6050_OK) {
        return MPU6050_ERROR;
    }

    return MPU6050_OK;  // All init steps passed
}


/* ════════════════════════════════════════════════════════
 * MPU6050_ReadAll
 *
 * Reads all sensor data in one burst I2C transaction.
 * This is efficient — one START/STOP for 14 bytes
 * instead of separate transactions for each axis.
 *
 * Register map starting at 0x3B:
 * 0x3B ACCEL_XOUT_H  ─┐
 * 0x3C ACCEL_XOUT_L  ─┘ → combine → raw_ax
 * 0x3D ACCEL_YOUT_H  ─┐
 * 0x3E ACCEL_YOUT_L  ─┘ → combine → raw_ay
 * 0x3F ACCEL_ZOUT_H  ─┐
 * 0x40 ACCEL_ZOUT_L  ─┘ → combine → raw_az
 * 0x41 TEMP_OUT_H    ─┐
 * 0x42 TEMP_OUT_L    ─┘ → combine → raw_temp
 * 0x43 GYRO_XOUT_H   ─┐
 * 0x44 GYRO_XOUT_L   ─┘ → combine → raw_gx
 * 0x45 GYRO_YOUT_H   ─┐
 * 0x46 GYRO_YOUT_L   ─┘ → combine → raw_gy
 * 0x47 GYRO_ZOUT_H   ─┐
 * 0x48 GYRO_ZOUT_L   ─┘ → combine → raw_gz
 * ════════════════════════════════════════════════════════ */
MPU6050_Status_t MPU6050_ReadAll(I2C_HandleTypeDef *hi2c,
                                  MPU6050_Data_t *data) {
    uint8_t buf[14];
    int16_t raw_ax, raw_ay, raw_az;
    int16_t raw_gx, raw_gy, raw_gz;
    int16_t raw_temp;

    /* Read 14 bytes starting from ACCEL_XOUT_H (0x3B)
     * MPU6050 auto-increments register address,
     * so one read gets all sensor data at once
     */
    if (MPU6050_ReadRegs(hi2c, MPU6050_REG_ACCEL_XOUT_H,
                         buf, 14) != MPU6050_OK) {
        return MPU6050_ERROR;
    }

    /* Combine high and low bytes into raw 16-bit values */
    raw_ax   = CombineBytes(buf[0],  buf[1]);
    raw_ay   = CombineBytes(buf[2],  buf[3]);
    raw_az   = CombineBytes(buf[4],  buf[5]);
    raw_temp = CombineBytes(buf[6],  buf[7]);
    raw_gx   = CombineBytes(buf[8],  buf[9]);
    raw_gy   = CombineBytes(buf[10], buf[11]);
    raw_gz   = CombineBytes(buf[12], buf[13]);

    /* Convert raw values to physical units
     *
     * Accelerometer: divide by sensitivity (16384 at ±2g)
     * Result is in g (1g = 9.81 m/s²)
     * When flat on table: az ≈ 1.0g (gravity), ax=ay≈0
     */
    data->accel_x = raw_ax / MPU6050_ACCEL_SENS_2G;
    data->accel_y = raw_ay / MPU6050_ACCEL_SENS_2G;
    data->accel_z = raw_az / MPU6050_ACCEL_SENS_2G;

    /* Gyroscope: divide by sensitivity (131 at ±250 deg/s)
     * Result is in degrees per second
     * When stationary: all values ≈ 0 (with some noise)
     */
    data->gyro_x = raw_gx / MPU6050_GYRO_SENS_250;
    data->gyro_y = raw_gy / MPU6050_GYRO_SENS_250;
    data->gyro_z = raw_gz / MPU6050_GYRO_SENS_250;

    /* Temperature: formula straight from datasheet
     * Section 4.18: Temp_degC = TEMP_OUT / 340 + 36.53
     */
    data->temperature = (raw_temp / 340.0f) + 36.53f;

    return MPU6050_OK;
}


/* ════════════════════════════════════════════════════════
 * MPU6050_ReadAccel
 * Read only accelerometer — 6 bytes from 0x3B
 * ════════════════════════════════════════════════════════ */
MPU6050_Status_t MPU6050_ReadAccel(I2C_HandleTypeDef *hi2c,
                                    MPU6050_Data_t *data) {
    uint8_t buf[6];

    if (MPU6050_ReadRegs(hi2c, MPU6050_REG_ACCEL_XOUT_H,
                         buf, 6) != MPU6050_OK) {
        return MPU6050_ERROR;
    }

    data->accel_x = CombineBytes(buf[0], buf[1]) / MPU6050_ACCEL_SENS_2G;
    data->accel_y = CombineBytes(buf[2], buf[3]) / MPU6050_ACCEL_SENS_2G;
    data->accel_z = CombineBytes(buf[4], buf[5]) / MPU6050_ACCEL_SENS_2G;

    return MPU6050_OK;
}


/* ════════════════════════════════════════════════════════
 * MPU6050_ReadGyro
 * Read only gyroscope — 6 bytes from 0x43
 * ════════════════════════════════════════════════════════ */
MPU6050_Status_t MPU6050_ReadGyro(I2C_HandleTypeDef *hi2c,
                                   MPU6050_Data_t *data) {
    uint8_t buf[6];

    if (MPU6050_ReadRegs(hi2c, MPU6050_REG_GYRO_XOUT_H,
                         buf, 6) != MPU6050_OK) {
        return MPU6050_ERROR;
    }

    data->gyro_x = CombineBytes(buf[0], buf[1]) / MPU6050_GYRO_SENS_250;
    data->gyro_y = CombineBytes(buf[2], buf[3]) / MPU6050_GYRO_SENS_250;
    data->gyro_z = CombineBytes(buf[4], buf[5]) / MPU6050_GYRO_SENS_250;

    return MPU6050_OK;
}


/* ════════════════════════════════════════════════════════
 * MPU6050_ReadTemp
 * Read only temperature — 2 bytes from 0x41
 * ════════════════════════════════════════════════════════ */
MPU6050_Status_t MPU6050_ReadTemp(I2C_HandleTypeDef *hi2c,
                                   MPU6050_Data_t *data) {
    uint8_t buf[2];

    if (MPU6050_ReadRegs(hi2c, MPU6050_REG_TEMP_OUT_H,
                         buf, 2) != MPU6050_OK) {
        return MPU6050_ERROR;
    }

    int16_t raw_temp = CombineBytes(buf[0], buf[1]);
    data->temperature = (raw_temp / 340.0f) + 36.53f;

    return MPU6050_OK;
}
