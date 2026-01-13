//
//#include "MS5611.h"
//#include <math.h>
//
//Struct_MS5611 MS5611;
//
//
//void MS5611_SPI_GPIO_Initialization(void)
//{
//	LL_SPI_InitTypeDef SPI_InitStruct = {0};
//
//	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
//	/* Peripheral clock enable */
//	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
//
//	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
//	/**SPI3 GPIO Configuration
//	PB3   ------> SPI3_SCK
//	PB4   ------> SPI3_MISO
//	PB5   ------> SPI3_MOSI
//	*/
//	GPIO_InitStruct.Pin = LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5;
//	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//	GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
//	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//	SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
//	SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
//	SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
//	SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
//	SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
//	SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
//	SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
//	SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
//	SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
//	SPI_InitStruct.CRCPoly = 10;
//	LL_SPI_Init(MS5611_SPI_CHANNEL, &SPI_InitStruct);
//	LL_SPI_SetStandard(MS5611_SPI_CHANNEL, LL_SPI_PROTOCOL_MOTOROLA);
//
//	/**MS5611 GPIO Control Configuration
//	 * PB6  ------> MS5611_SPI_CS_PIN (output)
//	 * PB7  ------> MS5611_INT_PIN (input)
//	 */
//	/**/
//	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);
//
//	/**/
//	GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
//	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
//	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//	/**/
//
//	LL_SPI_Enable(MS5611_SPI_CHANNEL);
//
//	MS5611_CHIP_DESELECT(MS5611);
//}
//
//
//static unsigned char MS5611_Transmit_Receive(unsigned char data)
//{
//	LL_SPI_TransmitData8(MS5611_SPI_CHANNEL, data);
//	while(LL_SPI_IsActiveFlag_TXE(MS5611_SPI_CHANNEL)==RESET);
//
//	while(LL_SPI_IsActiveFlag_RXNE(MS5611_SPI_CHANNEL)==RESET);
//	return LL_SPI_ReceiveData8(MS5611_SPI_CHANNEL);
//}
//
///**
// * @brief  Read 1 byte of given register
// * @retval Received data
// * @param  reg_addr		Register address to read from
// */
//uint8_t MS5611_Readbyte(uint8_t reg_addr)
//{
//	uint8_t val;
//
//	MS5611_CHIP_SELECT(MS5611);
//	MS5611_Transmit_Receive(reg_addr | 0x80);
//	val = MS5611_Transmit_Receive(0x00); 					//Send DUMMY to read data
//	MS5611_CHIP_DESELECT(MS5611);
//
//	return val;
//}
//
///**
// * @brief  Read n bytes starting from given register address
// * @retval No return value
// * @param  reg_addr		Register address to start read from
// * @param  len			Number of byte to read
// * @param  data			Pointer to data
// */
//void MS5611_Read_Buffer(unsigned char reg_addr, unsigned char len, unsigned char* data)
//{
//	unsigned int i = 0;
//
//	MS5611_CHIP_SELECT(MS5611);
//	MS5611_Transmit_Receive(reg_addr | 0x80);
//	while(i < len)
//	{
//		data[i++] = MS5611_Transmit_Receive(0x00);			//Send DUMMY to read data
//	}
//	MS5611_CHIP_DESELECT(MS5611);
//}
//
///**
// * @brief  Write 1 bytes in given register address
// * @retval No return value
// * @param  reg_addr		Register address to write
// * @param  val			Value to be written
// */
//void MS5611_Writebyte(uint8_t reg_addr, uint8_t val)
//{
//	MS5611_CHIP_SELECT(MS5611);
//	MS5611_Transmit_Receive(reg_addr & 0x7F);
//	MS5611_Transmit_Receive(val);	 						//Send Data to write
//	MS5611_CHIP_DESELECT(MS5611);
//}
//
///**
// * @brief  Write n bytes starting from given register address
// * @retval No return value
// * @param  reg_addr		Register address to start write
// * @param  len			Number of byte to read
// * @param  data			Pointer of data to be written
// */
//void MS5611_Write_Buffer(unsigned char reg_addr, unsigned char len, unsigned char* data)
//{
//	unsigned int i = 0;
//	MS5611_CHIP_SELECT(MS5611);
//	MS5611_Transmit_Receive(reg_addr & 0x7F);
//	while(i < len)
//	{
//		MS5611_Transmit_Receive(data[i++]); 				//Send Data to write
//	}
//	MS5611_CHIP_DESELECT(MS5611);
//}
//
///**
// * @brief  Read factory data and coefficients
// * @retval No return value
// */
//void MS5611_Read_Factory_Calibrated_Data(uint16_t* data)
//{
//	  uint8_t prom[2];
//
//	    // C1..C6
//	    MS5611_Read_Buffer(MS5611_REG_PROM_REG+2, 2, prom); MS5611.COEF1 = (prom[0]<<8)|prom[1];
//	    MS5611_Read_Buffer(MS5611_REG_PROM_REG+4, 2, prom); MS5611.COEF2 = (prom[0]<<8)|prom[1];
//	    MS5611_Read_Buffer(MS5611_REG_PROM_REG+6, 2, prom); MS5611.COEF3 = (prom[0]<<8)|prom[1];
//	    MS5611_Read_Buffer(MS5611_REG_PROM_REG+8, 2, prom); MS5611.COEF4 = (prom[0]<<8)|prom[1];
//	    MS5611_Read_Buffer(MS5611_REG_PROM_REG+10, 2, prom); MS5611.COEF5 = (prom[0]<<8)|prom[1];
//	    MS5611_Read_Buffer(MS5611_REG_PROM_REG+12, 2, prom); MS5611.COEF6 = (prom[0]<<8)|prom[1];
//
//	    // CRC (không quan trọng cho đo hiện tại)
//	    MS5611_Read_Buffer(MS5611_REG_PROM_REG+14, 2, prom); MS5611.CRC_SERIAL_CODE = (prom[0]<<8)|prom[1];
//}
//
///**
// * @brief  Initialize the device in reset form
// * @retval No return value
// */
//int MS5611_Initialization(void)
//{
//	MS5611_SPI_GPIO_Initialization();
//	MS5611_Read_Factory_Calibrated_Data(&MS5611.FACTORY_DATA) ;
//	// MS5611_Writebyte( MS5611_PRESURE_D1 | OSR_1024,  0x01);
//	return 0;
//}
//
///**
// * @brief  Get raw pressure data
// * @retval No return value
// * @param  pressure			Pointer to store pressure data
// */
//void MS5611_Get_Raw_Pressure(uint32_t* pressure)
//{
//    uint8_t data[3];
//
//    MS5611_Writebyte(MS5611_REG_PRESURE_D1 | OSR_1024, 0x01);
//    HAL_Delay(5);  // tăng delay
//
//    MS5611_CHIP_SELECT(MS5611);
//    MS5611_Transmit_Receive(0x00); // lệnh đọc ADC
//    for(int i=0;i<3;i++)
//        data[i] = MS5611_Transmit_Receive(0x00);
//    MS5611_CHIP_DESELECT(MS5611);
//
//    *pressure = ((uint32_t)data[0]<<16) | ((uint32_t)data[1]<<8) | data[2];
//}
//
//void MS5611_Get_Raw_Temperature(uint32_t* temperature)
//{
//    uint8_t data[3];
//
//    MS5611_Writebyte(MS5611_REG_TEMPERATURE_D2 | OSR_1024, 0x01);
//    HAL_Delay(5);  // tăng delay
//
//    MS5611_CHIP_SELECT(MS5611);
//    MS5611_Transmit_Receive(0x00); // lệnh đọc ADC
//    for(int i=0;i<3;i++)
//        data[i] = MS5611_Transmit_Receive(0x00);
//    MS5611_CHIP_DESELECT(MS5611);
//
//    *temperature = ((uint32_t)data[0]<<16) | ((uint32_t)data[1]<<8) | data[2];
//}
//
///**
// * @brief  Get raw pressure temperature data
// * @retval No return value
// * @param  MS5611			Pointer to Structure
// */
//void MS5611_Calculate_Temperature(Struct_MS5611* MS5611)
//{
//	int32_t TEMP;
//	MS5611_Get_Raw_Temperature(&MS5611->raw_temperature);
//	// Difference between actual and reference temperature
//	// dT = D2 - T_REF = D2 - C5 * 2^8
//	MS5611->dT=(int32_t)MS5611->raw_temperature - ((int32_t)MS5611->COEF5<<8);
//	// Actual temperature (-40…85°C with 0.01°C resolution)
//	// TEMP = 20°C + dT * TEMPSENS = 2000 + dT * C6 / 2^23
//	TEMP = 2000 + ((int64_t)MS5611->dT * (int64_t)MS5611->COEF6>>23) ;
//	// Second order temperature compensation
//	if (TEMP<2000)
//	{
//		// T2 = dT2 / 2^31
//		MS5611->T2=((int64_t)MS5611->dT*(int64_t)MS5611->dT)>>31;
//		// OFFSET2 = 5 * (TEMP – 2000)^2 / 2^1
//		MS5611->OFFSET2 = (5 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000)) >> 1 ;
//		// SENS2   = 5 * (TEMP – 2000)^2 / 2^2
//		MS5611->SENS2 = (5 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000)) >> 2 ;
//
//		// Low temperature
//		if( TEMP < -1500 )
//		{
//			// OFFSET2 = OFFSET2 + 7 * (TEMP + 1500)^2
//			MS5611->OFFSET2 += 7 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
//			// SENS2 = SENS2 + 11 * (TEMP + 1500)^2/ 2^1
//			MS5611->SENS2 += 11 * ((((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500)) >> 1) ;
//		}
//	}
//	else
//	{
//		// High temperature
//		MS5611->T2 = 0 ;
//		MS5611->OFFSET2 = 0 ;
//		MS5611->SENS2 = 0 ;
//	}
//	MS5611->temperature = ( (float)TEMP - MS5611->T2 ) / 100;
//}
//
///**
// * @brief  Get raw pressure temperature data
// * @retval No return value
// * @param  MS5611			Pointer to Structure
// */
//void MS5611_Calculate_Temperature_Compensated_Pressure(Struct_MS5611* MS5611)
//{
//	int64_t P;
//
//	MS5611_Get_Raw_Temperature(&MS5611->raw_temperature);
//	MS5611_Get_Raw_Pressure(&MS5611->raw_pressure);
//
//	// OFFSET = OFF_T1 + TCO * dT = C2 * 2^16 + (C4 * dT ) / 2^7
//	MS5611->OFFSET = ( (int64_t)(MS5611->COEF2) << 16 ) + ( ( (int64_t)(MS5611->COEF4) * MS5611->dT ) >> 7 ) ;
//
//	// Sensitivity at actual temperature
//	// SENS = SENS_T1 + TCS * dT = C1 * 2^15 + (C3 * dT ) / 2^8
//	MS5611->SENS = ( (int64_t)MS5611->COEF1 << 15 ) + ( ((int64_t)MS5611->COEF3 * MS5611->dT) >> 8 ) ;
//
//	MS5611->OFFSET -= MS5611->OFFSET2 ;
//	MS5611->SENS -= MS5611->SENS2 ;
//
//	// Temperature compensated pressure (10…1200mbar with 0.01mbar resolution)
//	// P = D1 * SENS - OFFSET = (D1 * SENS / 2^21 - OFFSET) / 2^15
//	P = ( ( (MS5611->raw_pressure * MS5611->SENS) >> 21 ) - MS5611->OFFSET ) >> 14 ;
//	MS5611->pressure = (float) P / 100;		// mbar
//}
//
////#define Y 0.90f
//
///**
// * @brief  Get raw pressure temperature data
// * @retval No return value
// * @param  MS5611			Pointer to Structure
// */
//void MS5611_Get_Altitude(Struct_MS5611 * MS5611)
//{
//	MS5611->altitude = 44330.0f * (1.0f - powf(MS5611->pressure / SEA_LEVEL_PRESSURE, 0.1903f));
//
//	//	MS5611->filtered_altitude = MS5611->filtered_altitude * Y + MS5611->altitude * (1.0f - Y);
//}

#include "MS5611.h"
#include <math.h>
#include "main.h"

Struct_MS5611 MS5611;

/* ========================= SPI & GPIO ========================= */
void MS5611_SPI_GPIO_Initialization(void)
{
    LL_SPI_InitTypeDef SPI_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

    /* SPI3 GPIO Configuration PB3=SCK, PB4=MISO, PB5=MOSI */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_3 | LL_GPIO_PIN_4 | LL_GPIO_PIN_5;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_6;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
    SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
    SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
    SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
    SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
    SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
    SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
    SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
    SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
    SPI_InitStruct.CRCPoly = 10;
    LL_SPI_Init(MS5611_SPI_CHANNEL, &SPI_InitStruct);
    LL_SPI_SetStandard(MS5611_SPI_CHANNEL, LL_SPI_PROTOCOL_MOTOROLA);
    LL_SPI_Enable(MS5611_SPI_CHANNEL);

    /* CS GPIO PB6 */
    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_6);
    GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    MS5611_CHIP_DESELECT(MS5611);
}

static uint8_t MS5611_Transmit_Receive(uint8_t data)
{
    LL_SPI_TransmitData8(MS5611_SPI_CHANNEL, data);
    while(!LL_SPI_IsActiveFlag_TXE(MS5611_SPI_CHANNEL));
    while(!LL_SPI_IsActiveFlag_RXNE(MS5611_SPI_CHANNEL));
    return LL_SPI_ReceiveData8(MS5611_SPI_CHANNEL);
}

/* ========================= Read / Write ========================= */
uint8_t MS5611_Readbyte(uint8_t reg_addr)
{
    uint8_t val;
    MS5611_CHIP_SELECT(MS5611);
    MS5611_Transmit_Receive(reg_addr | 0x80);
    val = MS5611_Transmit_Receive(0x00);
    MS5611_CHIP_DESELECT(MS5611);
    return val;
}

void MS5611_Read_Buffer(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
    uint8_t i = 0;
    MS5611_CHIP_SELECT(MS5611);
    MS5611_Transmit_Receive(reg_addr | 0x80);
    while(i < len) data[i++] = MS5611_Transmit_Receive(0x00);
    MS5611_CHIP_DESELECT(MS5611);
}

void MS5611_Writebyte(uint8_t reg_addr, uint8_t val)
{
    MS5611_CHIP_SELECT(MS5611);
    MS5611_Transmit_Receive(reg_addr & 0x7F);
    MS5611_Transmit_Receive(val);
    MS5611_CHIP_DESELECT(MS5611);
}

void MS5611_Write_Buffer(uint8_t reg_addr, uint8_t len, uint8_t* data)
{
    uint8_t i=0;
    MS5611_CHIP_SELECT(MS5611);
    MS5611_Transmit_Receive(reg_addr & 0x7F);
    while(i<len) MS5611_Transmit_Receive(data[i++]);
    MS5611_CHIP_DESELECT(MS5611);
}

/* ========================= Initialization ========================= */
void MS5611_Read_Factory_Calibrated_Data(uint16_t* data)
{
    uint8_t prom[2];
    MS5611_Read_Buffer(MS5611_REG_PROM_REG+2, 2, prom); MS5611.COEF1 = (prom[0]<<8)|prom[1];
    MS5611_Read_Buffer(MS5611_REG_PROM_REG+4, 2, prom); MS5611.COEF2 = (prom[0]<<8)|prom[1];
    MS5611_Read_Buffer(MS5611_REG_PROM_REG+6, 2, prom); MS5611.COEF3 = (prom[0]<<8)|prom[1];
    MS5611_Read_Buffer(MS5611_REG_PROM_REG+8, 2, prom); MS5611.COEF4 = (prom[0]<<8)|prom[1];
    MS5611_Read_Buffer(MS5611_REG_PROM_REG+10,2,prom); MS5611.COEF5 = (prom[0]<<8)|prom[1];
    MS5611_Read_Buffer(MS5611_REG_PROM_REG+12,2,prom); MS5611.COEF6 = (prom[0]<<8)|prom[1];
    MS5611_Read_Buffer(MS5611_REG_PROM_REG+14,2,prom); MS5611.CRC_SERIAL_CODE = (prom[0]<<8)|prom[1];
}

int MS5611_Initialization(void)
{
    MS5611_SPI_GPIO_Initialization();
    MS5611_Read_Factory_Calibrated_Data(&MS5611.FACTORY_DATA);
    MS5611.state = 0;
    MS5611.timestamp = HAL_GetTick();
    return 0;
}

/* ========================= Non-blocking Tick ========================= */
void MS5611_Tick(void)
{
    uint32_t now = HAL_GetTick();

    switch(MS5611.state)
    {
        case 0: // start D1 (pressure)
            MS5611_Writebyte(MS5611_REG_PRESURE_D1 | OSR_1024, 0x01);
            MS5611.timestamp = now;
            MS5611.state = 1;
            break;

        case 1: // wait D1
            if(now - MS5611.timestamp >= 5) // conversion time ~5ms
            {
                uint8_t data[3];
                MS5611_CHIP_SELECT(MS5611);
                MS5611_Transmit_Receive(0x00);
                for(int i=0;i<3;i++) data[i]=MS5611_Transmit_Receive(0x00);
                MS5611_CHIP_DESELECT(MS5611);
                MS5611.raw_pressure = ((uint32_t)data[0]<<16) | ((uint32_t)data[1]<<8) | data[2];
                MS5611.state = 2;
            }
            break;

        case 2: // start D2 (temperature)
            MS5611_Writebyte(MS5611_REG_TEMPERATURE_D2 | OSR_1024, 0x01);
            MS5611.timestamp = now;
            MS5611.state = 3;
            break;

        case 3: // wait D2
            if(now - MS5611.timestamp >= 5)
            {
                uint8_t data[3];
                MS5611_CHIP_SELECT(MS5611);
                MS5611_Transmit_Receive(0x00);
                for(int i=0;i<3;i++) data[i]=MS5611_Transmit_Receive(0x00);
                MS5611_CHIP_DESELECT(MS5611);
                MS5611.raw_temperature = ((uint32_t)data[0]<<16) | ((uint32_t)data[1]<<8) | data[2];

                // tính toán nhiệt độ và áp suất
                int32_t TEMP;
                MS5611.dT = (int32_t)MS5611.raw_temperature - ((int32_t)MS5611.COEF5 << 8);
                TEMP = 2000 + ((int64_t)MS5611.dT * (int64_t)MS5611.COEF6 >> 23);

                // second order
                if(TEMP<2000)
                {
                    MS5611.T2=((int64_t)MS5611.dT*(int64_t)MS5611.dT)>>31;
                    MS5611.OFFSET2 = (5*((int64_t)TEMP-2000)*((int64_t)TEMP-2000))>>1;
                    MS5611.SENS2   = (5*((int64_t)TEMP-2000)*((int64_t)TEMP-2000))>>2;
                    if(TEMP<-1500)
                    {
                        MS5611.OFFSET2 += 7*((int64_t)TEMP+1500)*((int64_t)TEMP+1500);
                        MS5611.SENS2   += 11*(((int64_t)TEMP+1500)*((int64_t)TEMP+1500)>>1);
                    }
                } else { MS5611.T2=MS5611.OFFSET2=MS5611.SENS2=0; }

                MS5611.temperature = ((float)TEMP - MS5611.T2)/100;

                MS5611.OFFSET = ((int64_t)MS5611.COEF2<<16) + (((int64_t)MS5611.COEF4*MS5611.dT)>>7) - MS5611.OFFSET2;
                MS5611.SENS   = ((int64_t)MS5611.COEF1<<15) + (((int64_t)MS5611.COEF3*MS5611.dT)>>8) - MS5611.SENS2;
                int64_t P = (((MS5611.raw_pressure * MS5611.SENS)>>21) - MS5611.OFFSET)>>14;
                MS5611.pressure = (float)P/100;

                // Altitude
                MS5611.altitude = 44330.0f * (1.0f - powf(MS5611.pressure/SEA_LEVEL_PRESSURE,0.1903f));

                MS5611.state = 0; // vòng lại D1
            }
            break;
    }
}
