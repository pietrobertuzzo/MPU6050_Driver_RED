/*
 * mpu6050.c
 *
 *  Created on: Mar 9, 2021
 *      Author: Pietro Bertuzzo
 */

#include "mpu6050.h"

/*
 *
 * Initialization
 *
 */

uint8_t MPU6050_Init(MPU6050 *imu, I2C_HandleTypeDef *i2cHandle) {

	/* Store interface parameter in a MPU6050 struct */
	imu->i2cHandle = i2cHandle;

	/* Clear DMA flags */

	imu->readingAcc = 0;
	imu->readingGyr = 0;
	imu->readingTemp = 0;

	/* Measurement config init */

	imu->accMode = 0x00;

	/* Accelerometer full scale select options:
	 * 2g(00), 4g(01), 8g(10), 16g(11). Default: 2g(00)
	 */

	imu->gyrMode = 0x00;

	/* Gyroscope full scale select options:
	 * 250dps (00), 500dps(01), 1000dps(10), 2000dps(11). Default: 250dps(00)
	 */

	imu->accDLPFConfig = 0x00; // Digital low pass filter configuration. Check datasheet.

	imu-> accRes = 0.000061035f;
	imu-> gyrRes = 0.0076f;

	/* Perform who am I check (expected value 0x70) */
	uint8_t whoAmI;
	uint8_t status = 0;
	HAL_I2C_Mem_Read (imu->i2cHandle, MPU6050_I2C_ADDRESS, WHO_AM_I,1, &whoAmI, 1, 1000);
	if (whoAmI != 0x68) {
		status = 1;
	}

	// power management register 0X6B we should write all 0's to wake the sensor up
	uint8_t Data = 0;
	HAL_I2C_Mem_Write(imu->i2cHandle, MPU6050_I2C_ADDRESS, PWR_MGMT_1, 1,&Data, 1, 1000);

	// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
	Data = 0x07;
	HAL_I2C_Mem_Write(imu->i2cHandle, MPU6050_I2C_ADDRESS, SMPLRT_DIV, 1, &Data, 1, 1000);
	HAL_Delay(10);
	// Enable data ready interrupt
	Data = 0x01;
	HAL_I2C_Mem_Write(imu->i2cHandle, MPU6050_I2C_ADDRESS, INT_ENABLE, 1, &Data, 1, 1000);
	HAL_Delay(10);

	Data = 0xD0;
	HAL_I2C_Mem_Write(imu->i2cHandle, MPU6050_I2C_ADDRESS, INT_PIN_CFG, 1, &Data, 1, 1000);
	HAL_Delay(10);

	return status;
}

uint8_t MPU6050_Update(MPU6050 *imu) {
	if (imu->accMode > 0x03) {
		return 0;
	}
	if (imu->gyrMode > 0x03) {
		return 0;
	}
	uint8_t txBuf = imu->accMode<<3;
	MPU6050_WriteRegister(imu, ACCEL_CONFIG, txBuf);
	txBuf = imu->gyrMode<<3;
	MPU6050_WriteRegister(imu, GYRO_CONFIG, txBuf);
	return 1;
}

uint8_t MPU6050_ReadRegister(MPU6050 *imu, uint8_t regAddr, uint8_t *data) {
	uint8_t rxBuf = 0;

	uint8_t status = (HAL_I2C_Mem_Read(imu->i2cHandle, MPU6050_I2C_ADDRESS, regAddr, 1, &rxBuf, 1, HAL_MAX_DELAY) == HAL_OK);

	if (status) {
		*data = rxBuf;
	}
	return status;
}

uint8_t MPU6050_WriteRegister(MPU6050 *imu, uint8_t regAddr, uint8_t data) {
	uint8_t txBuf =0 ;

	uint8_t status = (HAL_I2C_Mem_Write(imu->i2cHandle, MPU6050_I2C_ADDRESS, regAddr, 1, &txBuf, 1, HAL_MAX_DELAY) == HAL_OK);

	return status;
}

uint8_t MPU6050_ReadAccelerometer(MPU6050 *imu) {
	uint8_t rxBuf[6];

	uint8_t status = (HAL_I2C_Mem_Read(imu->i2cHandle, MPU6050_I2C_ADDRESS, ACCEL_XOUT_H, 6, rxBuf, 6, HAL_MAX_DELAY) == HAL_OK);

	if(status) {
		int16_t accelX = (int8_t) rxBuf[2] | ((int8_t) rxBuf[1]<<8);
		int16_t accelY = (int8_t) rxBuf[4] | ((int8_t) rxBuf[3]<<8);
		int16_t accelZ = (int8_t) rxBuf[6] | ((int8_t) rxBuf[5]<<8);
		imu->accMeasurement[1] = (float) accelX * imu->accRes;
		imu->accMeasurement[2] = (float) accelY * imu->accRes;
		imu->accMeasurement[3] = (float) accelZ * imu->accRes;
	}
	return status;
}

uint8_t MPU6050_ReadGyroscope(MPU6050 *imu) {
	uint8_t rxBuf[6];

	uint8_t status = (HAL_I2C_Mem_Read(imu->i2cHandle, MPU6050_I2C_ADDRESS, GYRO_XOUT_H, 6, rxBuf, 6, HAL_MAX_DELAY) == HAL_OK);

	if(status) {
		int16_t gyroX = (int8_t) rxBuf[2] | ((int8_t) rxBuf[1]<<8);
		int16_t gyroY = (int8_t) rxBuf[4] | ((int8_t) rxBuf[3]<<8);
		int16_t gyroZ = (int8_t) rxBuf[6] | ((int8_t) rxBuf[5]<<8);
		imu->gyrMeasurement[0] = (float) gyroX * imu->gyrRes;
		imu->gyrMeasurement[1] = (float) gyroY * imu->gyrRes;
		imu->gyrMeasurement[2] = (float) gyroZ * imu->gyrRes;
	}
	return status;
}

uint8_t MPU6050_ReadAccelerometerDMA(MPU6050 *imu) {
	if (HAL_I2C_Mem_Read_DMA(imu->i2cHandle, MPU6050_I2C_ADDRESS, ACCEL_XOUT_H, 6, (uint8_t *) imu->accRxBuf, 6)) {
		imu->readingAcc = 1;
		return 1;
	}
	else return 0;
}

void MPU6050_ReadAccelerometerDMA_Complete(MPU6050 *imu) {
	imu->readingAcc = 0;

	/* Forming 16 bit signed integers from H and L bytes */
	int16_t accelX = (int8_t) imu->accRxBuf[2] | ((int8_t) imu->accRxBuf[1]<<8);
	int16_t accelY = (int8_t) imu->accRxBuf[4] | ((int8_t) imu->accRxBuf[3]<<8);
	int16_t accelZ = (int8_t) imu->accRxBuf[6] | ((int8_t) imu->accRxBuf[5]<<8);

	/* Converting to values */
	imu->accMeasurement[1] = (float) accelX * imu->accRes;
	imu->accMeasurement[2] = (float) accelY * imu->accRes;
	imu->accMeasurement[3] = (float) accelZ * imu->accRes;
}

uint8_t MPU6050_ReadGyroscopeDMA(MPU6050 *imu) {
	if(HAL_I2C_Mem_Read_DMA(imu->i2cHandle, MPU6050_I2C_ADDRESS, GYRO_XOUT_H, 6, (uint8_t *) imu->gyrRxBuf, 6)) {
		imu->readingGyr = 1;
		return 1;
	}
	else return 0;
}

void MPU6050_ReadGyroscopeDMA_Complete(MPU6050 *imu) {
	imu->readingGyr = 0;

	/* Forming 16 bit signed integers from H and L bytes */
	int16_t gyroX = (int8_t) imu->gyrRxBuf[2] | ((int8_t) imu->accRxBuf[1]<<8);
	int16_t gyroY = (int8_t) imu->gyrRxBuf[4] | ((int8_t) imu->accRxBuf[3]<<8);
	int16_t gyroZ = (int8_t) imu->gyrRxBuf[6] | ((int8_t) imu->accRxBuf[5]<<8);

	/* Converting to values */
	imu->gyrMeasurement[1] = (float) gyroX * imu->gyrRes;
	imu->gyrMeasurement[2] = (float) gyroY * imu->gyrRes;
	imu->gyrMeasurement[3] = (float) gyroZ * imu->gyrRes;

 }
