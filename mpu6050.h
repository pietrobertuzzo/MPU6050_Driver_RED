/*
 * mpu6050.h
 *
 *  Created on: Mar 9, 2021
 *      Author: Pietro Bertuzzo
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f1xx_hal.h"
/* Register defines*/

#define MPU6050_I2C_ADDRESS (0x68<<1)

#define SELF_TEST_X_GYRO	0x00
#define SELF_TEST_Y_GYRO	0x01
#define SELF_TEST_Z_GYRO	0x02
#define SELF_TEST_X_ACCEL	0x0D
#define SELF_TEST_Y_ACCEL	0x0E
#define SELF_TEST_Z_ACCEL	0x0F

#define SMPLRT_DIV			0x19
#define CONFIG				0x1A
#define GYRO_CONFIG			0x1B
#define ACCEL_CONFIG		0x1C
#define ACCEL_CONFIG_2		0x1D

#define INT_PIN_CFG 		0x37
#define INT_ENABLE 			0x38
#define INT_STATUS			0x3A
#define ACCEL_XOUT_H		0x3B
#define ACCEL_XOUT_L		0x3C
#define ACCEL_YOUT_H		0x3D
#define ACCEL_YOUT_L		0x3E
#define	ACCEL_ZOUT_H		0x3F
#define	ACCEL_ZOUT_L		0x40
#define TEMP_OUT_H			0x41
#define	TEMP_OUT_L			0x42
#define GYRO_XOUT_H			0x43
#define GYRO_XOUT_L			0x44
#define GYRO_YOUT_H			0x45
#define GYRO_YOUT_L			0x46
#define GYRO_ZOUT_H			0x47
#define GYRO_ZOUT_L			0x48
#define	PWR_MGMT_1			0x6B
#define WHO_AM_I			0x75

typedef struct {

	/* I2C */
	I2C_HandleTypeDef	*i2cHandle;

	/* DMA */
	uint8_t readingAcc;
	uint8_t readingGyr;
	uint8_t readingTemp;

	volatile uint8_t accRxBuf[8];
	volatile uint8_t gyrRxBuf[8];
	volatile uint8_t temRxBuf[8];

	/* Measurement mode */
	uint8_t gyrMode; // 2 bit value
	uint8_t accMode; // 2 bit value
	uint8_t accDLPFConfig; // Low pass filter config (check datasheet)

	/* Measurement vectors (x-y-z when applicable) */
	float accMeasurement[3];
	float gyrMeasurement[3];
	float temMeasurement;

	float accRes;
	float gyrRes;
	float tempRes;
} MPU6050;

/*
 *
 * Initialization
 *
 */

 uint8_t MPU6050_Init(MPU6050 *imu, I2C_HandleTypeDef *i2cHAndle);

/*
 *
 * Config Update
 *
 */

 uint8_t MPU6050_Update(MPU6050 *imu);

/*
 *
 * Low-lever register functions
 *
 */

 uint8_t MPU6050_ReadRegister(MPU6050 *imu, uint8_t regAddr, uint8_t *data);
 uint8_t MPU6050_WriteRegister(MPU6050 *imu, uint8_t regAddr, uint8_t data);


 /*
  *
  * Polling
  *
  */

 uint8_t MPU6050_ReadAccelerometer(MPU6050 *imu);
 uint8_t MPU6050_ReadGyroscope(MPU6050 *imu);
 uint8_t MPU6050_ReadTemperature(MPU6050 *imu);

 /*
  *
  * DMA
  *
  */

 uint8_t MPU6050_ReadAccelerometerDMA(MPU6050 *imu);
 void MPU6050_ReadAccelerometerDMA_Complete(MPU6050 *imu);

 uint8_t MPU6050_ReadGyroscopeDMA(MPU6050 *imu);
 void MPU6050_ReadGyroscopeDMA_Complete(MPU6050 *imu);

 uint8_t MPU6050_ReadTemperatureDMA(MPU6050 *imu);
 void MPU6050_ReadTemperatureDMA_Complete(MPU6050 *imu);

#endif
