/*
 * mpu6050.h
 *
 *  Created on: Nov 20, 2024
 *      Author: ImadF
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"
#include <stdio.h>

//MPU6050 REGISTER ADDRESSES
#define MPU6050_REG_WHOAMI					(uint8_t)117UL
#define MPU6050_REG_PWR_MGMT_1				(uint8_t)107UL
#define MPU6050_REG_ACCEL_BASE				(uint8_t)59UL
#define MPU6050_REG_CONFIG					(uint8_t)26UL
#define MPU6050_REG_INT_ENABLE				(uint8_t)56UL
#define MPU6050_REG_INT_PIN_CONFIG			(uint8_t)55UL


typedef enum
{
	MPU6050_OK, MPU6050_ERROR
}mpu6050status_t;

typedef enum {
	DLPF_CFG_260HZ = 0,
	DLPF_CFG_184HZ = 1,
	DLPF_CFG_94HZ = 2,
	DLPF_CFG_44HZ = 3,
	DLPF_CFG_21HZ = 4,
	DLPF_CFG_10HZ = 5,
	DLPF_CFG_5HZ = 6,

}mpu6050_dlpf_config_t;


typedef enum
{
	INT_LEVEL_ACTIVE_HIGH = 0,
	INT_LEVEL_ACTIVE_LOW,
}mpu6050_interrupt_config_t;


typedef enum
{
	RAW_RDY_INT = 0,
	I2C_MST_INT = 0x08,
	FIFO_OVRFLOW_INT = 0x10,
	MOT_INT = 0x40,
	ALL_INT = 0xFF
}mpu6050_interrupt;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}mpu6050_accel_data;

mpu6050status_t mpu6050_init(I2C_HandleTypeDef* hi2c1,uint16_t dev_addr);
mpu6050status_t mpu6050_read_accelerometer_data(I2C_HandleTypeDef* hi2c1,uint16_t dev_addr, mpu6050_accel_data* data_buffer);
mpu6050_accel_data mpu6050_accel_calib(mpu6050_accel_data* raw_data, const mpu6050_accel_data* offset);
mpu6050status_t mpu6050_configure_low_pass_filter(I2C_HandleTypeDef *hi2c, mpu6050_dlpf_config_t dlpf);
mpu6050status_t mpu6050_interrupt_config(I2C_HandleTypeDef* hi2c1,mpu6050_interrupt_config_t value);
mpu6050status_t mpu6050_enable_interrupt(I2C_HandleTypeDef* hi2c1, mpu6050_interrupt interrupt);
mpu6050status_t mpu6050_disable_interrupt(I2C_HandleTypeDef* hi2c1, mpu6050_interrupt interrupt);


#endif /* INC_MPU6050_H_ */
