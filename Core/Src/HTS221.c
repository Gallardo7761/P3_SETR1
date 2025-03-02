#include "stm32l4xx_hal.h"
#include "HTS221.h"

extern I2C_HandleTypeDef hi2c2;

//Estructura para almacenar la calibración
typedef struct{
	//Valores de los registros
	float H0_rH_x2;
	float H1_rH_x2;

	float T0_degC_x8;
	float T1_degC_x8;

	int16_t H0_T0_OUT;
	int16_t H1_T0_OUT;

	int16_t T0_OUT;
	int16_t T1_OUT;

	//Rectas de calibración para humedad y temperatura
	float ha, hb;

	float ta, tb;

} HTS_Calibration;

//Instancia de la calibración
HTS_Calibration hts_cal;

// Direcciones de los registros
#define I2C_TH 0xBE
#define HTS_H0_rH_x2 0x30
#define HTS_H1_rH_x2 0x31
#define HTS_T0_degC_x8	 0x32
#define HTS_T1_degC_x8	 0x33

#define HTS_T1_T0_MSB	 0x35
#define HTS_H0_T0_OUT_LSB 0x36
#define HTS_H0_T0_OUT_MSB 0x37

#define HTS_H1_T0_OUT_LSB 0x3A
#define HTS_H1_T0_OUT_MSB 0x3B

#define HTS_T0_OUT_LSB 0x3C
#define HTS_T0_OUT_MSB 0x3D

#define HTS_T1_OUT_LSB 0x3E
#define HTS_T1_OUT_MSB 0x3F


void HTS221_UpdateCalibration(){
	uint8_t buffer;
	uint8_t tempMSB;


	HAL_I2C_Mem_Read(&hi2c2, I2C_TH, HTS_H0_rH_x2, I2C_MEMADD_SIZE_8BIT, &buffer, 1, 1000);
	hts_cal.H0_rH_x2 = buffer / 2.0f;

	HAL_I2C_Mem_Read(&hi2c2, I2C_TH, HTS_H1_rH_x2, I2C_MEMADD_SIZE_8BIT, &buffer, 1, 1000);
	hts_cal.H1_rH_x2 = buffer / 2.0f;

	HAL_I2C_Mem_Read(&hi2c2, I2C_TH, HTS_T1_T0_MSB, I2C_MEMADD_SIZE_8BIT, &tempMSB, 1, 1000);

	HAL_I2C_Mem_Read(&hi2c2, I2C_TH, HTS_T0_degC_x8, I2C_MEMADD_SIZE_8BIT, &buffer, 1, 1000);
	hts_cal.T0_degC_x8 = (((tempMSB & 0x03) <<8) |  buffer) / 8.0f ;

	HAL_I2C_Mem_Read(&hi2c2, I2C_TH, HTS_T1_degC_x8, I2C_MEMADD_SIZE_8BIT, &buffer, 1, 1000);
	hts_cal.T1_degC_x8 =  (((tempMSB & 0x0C) <<6) |  buffer) / 8.0f ;

	HAL_I2C_Mem_Read(&hi2c2, I2C_TH, HTS_H0_T0_OUT_LSB, I2C_MEMADD_SIZE_8BIT, &buffer, 1, 1000);
	HAL_I2C_Mem_Read(&hi2c2, I2C_TH, HTS_H0_T0_OUT_MSB, I2C_MEMADD_SIZE_8BIT, &tempMSB, 1, 1000);
	hts_cal.H0_T0_OUT = (tempMSB <<8) |  buffer;

	HAL_I2C_Mem_Read(&hi2c2, I2C_TH, HTS_H1_T0_OUT_LSB, I2C_MEMADD_SIZE_8BIT, &buffer, 1, 1000);
	HAL_I2C_Mem_Read(&hi2c2, I2C_TH, HTS_H1_T0_OUT_MSB, I2C_MEMADD_SIZE_8BIT, &tempMSB, 1, 1000);
	hts_cal.H1_T0_OUT = (tempMSB <<8) |  buffer;

	HAL_I2C_Mem_Read(&hi2c2, I2C_TH, HTS_T0_OUT_LSB, I2C_MEMADD_SIZE_8BIT, &buffer, 1, 1000);
	HAL_I2C_Mem_Read(&hi2c2, I2C_TH, HTS_T0_OUT_MSB, I2C_MEMADD_SIZE_8BIT, &tempMSB, 1, 1000);
	hts_cal.T0_OUT = (tempMSB <<8) |  buffer;

	HAL_I2C_Mem_Read(&hi2c2, I2C_TH, HTS_T1_OUT_LSB, I2C_MEMADD_SIZE_8BIT, &buffer, 1, 1000);
	HAL_I2C_Mem_Read(&hi2c2, I2C_TH, HTS_T1_OUT_MSB, I2C_MEMADD_SIZE_8BIT, &tempMSB, 1, 1000);
	hts_cal.T1_OUT = (tempMSB <<8) |  buffer;


	hts_cal.ha = (hts_cal.H1_rH_x2 - hts_cal.H0_rH_x2) / (hts_cal.H1_T0_OUT - hts_cal.H0_T0_OUT);

	hts_cal.hb = hts_cal.H0_rH_x2 - hts_cal.ha*hts_cal.H0_T0_OUT;

	hts_cal.ta = (hts_cal.T1_degC_x8 - hts_cal.T0_degC_x8) / (hts_cal.T1_OUT - hts_cal.T0_OUT);

	hts_cal.tb = hts_cal.T0_degC_x8 - hts_cal.ha*hts_cal.T0_OUT;

}

void HTS221_Init() {
	uint8_t buffer[1];
	buffer[0] = 0x87;
	HAL_I2C_Mem_Write(&hi2c2, 0xBE, 0x20, I2C_MEMADD_SIZE_8BIT, buffer, 1, 1000);
	HTS221_UpdateCalibration();
}

THSample HTS221_Read() {
	THSample ths;

	uint8_t buffer[4];
	HAL_I2C_Mem_Read(&hi2c2, 0xBE, 0x80 | 0x28, I2C_MEMADD_SIZE_8BIT, buffer, 4, 1000);

	int16_t hum_raw;
	int16_t temp_raw;

	hum_raw = (buffer[1] << 8) | buffer[0];
	temp_raw = (buffer[3] << 8) | buffer[2];

	ths.hum = hts_cal.ha * hum_raw + hts_cal.hb;
	ths.temp = hts_cal.ta * temp_raw + hts_cal.tb;

	return ths;

	return ths;
}
