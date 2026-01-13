#ifndef __MS5611_H__
#define __MS5611_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define MS5611_SPI_CHANNEL		SPI3

#define MS5611_CHIP_SELECT(MS5611)		LL_GPIO_ResetOutputPin( GPIOB, LL_GPIO_PIN_6)
#define MS5611_CHIP_DESELECT(MS5611)	LL_GPIO_SetOutputPin(  GPIOB, LL_GPIO_PIN_6)

/* Register Map */
#define MS5611_REG_RESET			UINT8_C(0x1E)
#define MS5611_REG_PRESURE_D1		UINT8_C(0x40)
#define MS5611_REG_TEMPERATURE_D2 	UINT8_C(0x50)
#define MS5611_REG_ADC_REG			UINT8_C(0x00)
#define MS5611_REG_PROM_REG			UINT8_C(0xA0)

#define OSR_256  					UINT8_C(0x00)
#define OSR_512  					UINT8_C(0x02)
#define OSR_1024 					UINT8_C(0x04)
#define OSR_2048 					UINT8_C(0x06)
#define OSR_4096 					UINT8_C(0x08)

#define ADC_SIZE					3
#define PROM_SIZE					16
#define SEA_LEVEL_PRESSURE 1013.25f // mbar

/* Structure Definition */
typedef struct _MS5611
{
	float 			pressure;
	float 			temperature;
	float			altitude;
	float			filtered_altitude;
	uint32_t		raw_pressure;
	uint32_t		raw_temperature;
	float			altitude_setpoint;   // điểm setpoint
	float			vertical_speed;
	double     last_altitude;
	float filtered_pressure;
	float pressure_setpoint;
	float pressure_filtered;  // <<-- giá trị đã filter mượt, dùng cho PID
	// lưu lần đọc trước
	    // tính ra dt động
	    uint8_t    updated;
	// PROM data
	uint16_t		FACTORY_DATA;
	uint16_t		COEF1;
	uint16_t		COEF2;
	uint16_t		COEF3;
	uint16_t		COEF4;
	uint16_t		COEF5;
	uint16_t		COEF6;
	uint16_t		CRC_SERIAL_CODE;
	// Compensation variables
	int32_t			dT;
	int64_t			T2;
	int64_t			OFFSET;
	int64_t			OFFSET2;
	int64_t			SENS;
	int64_t			SENS2;
	uint8_t			ADC_DATA[ADC_SIZE];

	// --- Non-blocking state machine ---
	uint8_t			state;       // 0: start D1, 1: wait D1, 2: start D2, 3: wait D2
	uint32_t		timestamp;   // HAL_GetTick() timestamp
}Struct_MS5611;

extern Struct_MS5611 MS5611;

/* Core SPI read/write functions */
uint8_t MS5611_Readbyte(uint8_t reg_addr);
void 	MS5611_Read_Buffer(unsigned char reg_addr, unsigned char len, unsigned char* data);
void 	MS5611_Writebyte(uint8_t reg_addr, uint8_t val);
void 	MS5611_Write_Buffer(unsigned char reg_addr, unsigned char len, unsigned char* data);

/* Initialization */
void MS5611_SPI_GPIO_Initialization(void);
int  MS5611_Initialization(void);
void MS5611_Read_Factory_Calibrated_Data(uint16_t* data);

/* Old blocking functions (optional) */
void MS5611_Get_Raw_Pressure(uint32_t* pressure);
void MS5611_Get_Raw_Temperature(uint32_t* temperature);
void MS5611_Calculate_Temperature(Struct_MS5611* MS5611);
void MS5611_Calculate_Temperature_Compensated_Pressure(Struct_MS5611* MS5611);
void MS5611_Get_Altitude(Struct_MS5611 * MS5611);

/* --- Non-blocking functions --- */
void MS5611_Init_NonBlocking(void);  // init coefficients & state machine
void MS5611_Tick(void);              // call in main loop, non-blocking update

#ifdef __cplusplus
}
#endif

#endif /* __MS5611_H__ */
