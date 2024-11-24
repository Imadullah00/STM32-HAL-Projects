/*
 * mpu6050.c
 *
 *  Created on: Nov 20, 2024
 *      Author: ImadF
 */

#include "mpu6050.h"

#define TIMEOUT		500

static uint8_t mpu6050_dev_addr;

mpu6050status_t mpu6050_read_byte(I2C_HandleTypeDef* hi2c1,uint16_t dev_addr,uint8_t reg_addr,uint8_t* pdata)
{
	if(HAL_I2C_Mem_Read(hi2c1, mpu6050_dev_addr<<1, reg_addr, 1, pdata, 1, TIMEOUT) == HAL_OK)
	{
		return MPU6050_OK;
	}
	else
		return MPU6050_ERROR;
}

mpu6050status_t mpu6050_read(I2C_HandleTypeDef* hi2c1,uint16_t dev_addr,uint8_t reg_base_addr,uint8_t* buffer, uint16_t nbytes)
{
	if(HAL_I2C_Mem_Read(hi2c1, mpu6050_dev_addr<<1, reg_base_addr, 1, buffer, nbytes, TIMEOUT) == HAL_OK)
	{
		return MPU6050_OK;
	}
	else
		return MPU6050_ERROR;
}

mpu6050status_t mpu6050_write_byte(I2C_HandleTypeDef* hi2c1,uint16_t dev_addr,uint8_t reg_addr,uint8_t* pdata)
{
	if(HAL_I2C_Mem_Write(hi2c1, dev_addr, reg_addr, 1, pdata, 1, TIMEOUT) == HAL_OK)
	{
		return MPU6050_OK;
	}
	else
		return MPU6050_ERROR;
}

mpu6050status_t mpu6050_init(I2C_HandleTypeDef* hi2c1,uint16_t dev_addr)
{


	uint8_t data = 0;
	mpu6050_dev_addr = dev_addr;

	if(mpu6050_read_byte(hi2c1, dev_addr, MPU6050_REG_WHOAMI, &data) != MPU6050_OK)
	{
		return MPU6050_ERROR;
	}

	if (data == 0x68 || data == 0x98)
	{
		printf("Valid mpu6050 found at %X", mpu6050_dev_addr);
	}
	else
	{
		printf("Invalid device found at %X", mpu6050_dev_addr);
		return MPU6050_ERROR;
	}

	data = 0x00;

	if(mpu6050_write_byte(hi2c1,dev_addr,MPU6050_REG_PWR_MGMT_1,&data)!= MPU6050_OK)
	{
		return MPU6050_ERROR;
	}

	return MPU6050_OK;
}



mpu6050status_t mpu6050_read_accelerometer_data(I2C_HandleTypeDef* hi2c1,uint16_t dev_addr, mpu6050_accel_data* data_buffer )
{
	uint8_t raw_buffer[6];

	if(mpu6050_read(hi2c1, dev_addr, MPU6050_REG_ACCEL_BASE, raw_buffer, sizeof(raw_buffer)) != MPU6050_OK)
		return MPU6050_ERROR;

	else
	{
		data_buffer->x = (uint16_t)raw_buffer[0]<<8 | raw_buffer[1];
		data_buffer->y = (uint16_t)raw_buffer[2]<<8 | raw_buffer[3];
		data_buffer->z = (uint16_t)raw_buffer[4]<<8 | raw_buffer[5];
	}

	return MPU6050_OK;

}

mpu6050_accel_data mpu6050_accel_calib(mpu6050_accel_data* raw_data, const mpu6050_accel_data* offset)
{
	mpu6050_accel_data calibrated_data;

	calibrated_data.x = raw_data->x - offset->x;
	calibrated_data.y = raw_data->y - offset->y;
	calibrated_data.z = raw_data->x - offset->z;

	return calibrated_data;

}



mpu6050status_t mpu6050_interrupt_config(I2C_HandleTypeDef* hi2c1,mpu6050_interrupt_config_t value)
{
	uint8_t read_value = 0x00;

	if(mpu6050_read_byte(hi2c1, mpu6050_dev_addr, MPU6050_REG_INT_PIN_CONFIG, &read_value) != MPU6050_OK)
	{
		return MPU6050_ERROR;
	}

	if(value == INT_LEVEL_ACTIVE_HIGH)
	{
		read_value &= ~(1 << 3);
	}

	else
	{
		read_value |= (1<<3);
	}

	if(mpu6050_write_byte(hi2c1, mpu6050_dev_addr, MPU6050_REG_INT_PIN_CONFIG, (uint8_t*)&read_value)!= MPU6050_OK)
	{
		return MPU6050_ERROR;
	}

	return MPU6050_OK;


}

mpu6050status_t mpu6050_enable_interrupt(I2C_HandleTypeDef* hi2c1, mpu6050_interrupt interrupt)
{
	uint8_t read_value = 0x00;

	if(mpu6050_read_byte(hi2c1, mpu6050_dev_addr, MPU6050_REG_INT_ENABLE, &read_value) != MPU6050_OK)
	{
		return MPU6050_ERROR;
	}

	read_value |= (uint8_t)interrupt;

	if(mpu6050_write_byte(hi2c1, mpu6050_dev_addr, MPU6050_REG_INT_ENABLE, (uint8_t*)&read_value)!= MPU6050_OK)
	{
		return MPU6050_ERROR;
	}

	return MPU6050_OK;
}

mpu6050status_t mpu6050_disable_interrupt(I2C_HandleTypeDef* hi2c1, mpu6050_interrupt interrupt)
{
	uint8_t read_value = 0x00;

	if(mpu6050_read_byte(hi2c1, mpu6050_dev_addr, MPU6050_REG_INT_ENABLE, &read_value) != MPU6050_OK)
	{
		return MPU6050_ERROR;
	}

	read_value &= ~(interrupt);

	if(mpu6050_write_byte(hi2c1, mpu6050_dev_addr, MPU6050_REG_INT_ENABLE, (uint8_t*)&read_value)!= MPU6050_OK)
	{
		return MPU6050_ERROR;
	}

	return MPU6050_OK;

}
mpu6050status_t mpu6050_configure_low_pass_filter(I2C_HandleTypeDef *hi2c1, mpu6050_dlpf_config_t dlpf)
{

	uint8_t value = 0;

	if (mpu6050_read_byte(hi2c1,mpu6050_dev_addr<<1, MPU6050_REG_CONFIG, &value) != MPU6050_OK)
	{
		return MPU6050_ERROR;
	}

	value &= ~(0x7);
	value |= (uint8_t)dlpf;
	if (mpu6050_write_byte(hi2c1,mpu6050_dev_addr<<1, MPU6050_REG_CONFIG, &value) != MPU6050_OK)
	{
		return MPU6050_ERROR;
	}

	return MPU6050_OK;
}
