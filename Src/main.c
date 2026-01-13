/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "adc.h"
//#include "dma.h"
//#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BNO080.h"
#include "Quaternion.h"
#include "ICM20602.h"
//#include "lps22hh.h"
#include "M8N.h"
#include "FS-iA6B.h"
//#include "AT24C08.h"
#include "MS5611.h"
#include <string.h>
#include "PID control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int _write(int file, char* p, int len)
{
	for(int i=0;i<len;i++)
	{
		while(!LL_USART_IsActiveFlag_TXE(USART6));
		LL_USART_TransmitData8(USART6, *(p+i));
	}
	return len;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint8_t uart6_rx_flag;
extern uint8_t uart6_rx_data;

 extern uint8_t m8n_rx_buf[36];
 extern uint8_t m8n_rx_cplt_flag;

extern uint8_t ibus_rx_buf[32];
extern uint8_t ibus_rx_cplt_flag;

extern uint8_t uart1_rx_data;

extern uint8_t tim7_1ms_flag;
extern uint8_t tim7_10ms_flag;
extern uint8_t tim7_20ms_flag;
extern uint8_t tim7_100ms_flag;
extern uint8_t tim7_200ms_flag;
extern uint8_t tim7_1000ms_flag;
volatile float loop_1ms_time_us = 0.0f;
 uint8_t telemetry_tx_buf[40];
// uint8_t telemetry_rx_buf[20];
// uint8_t telemetry_rx_cplt_flag;
// float batVolt;

unsigned char failsafe_flag = 0;
//unsigned char low_bat_flag = 0;
/* =============== THÊM CHO POSITION HOLD =============== */
uint8_t position_hold_flag = 0;          // Cờ bật/tắt Position Hold
int32_t setpoint_lat = 0;                // Setpoint latitude (raw từ GPS)
int32_t setpoint_lon = 0;                // Setpoint longitude (raw từ GPS)

float current_lat_deg = 0.0f;            // Latitude hiện tại (độ)
float current_lon_deg = 0.0f;            // Longitude hiện tại (độ)
float yaw_deg = 0.0f;                    // Yaw từ BNO080 (độ)

/* PID Position đã có sẵn từ file PID control.c */
extern float Position_pitch;             // Setpoint pitch từ position hold
extern float Position_roll;              // Setpoint roll từ position hold
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int Is_iBus_Throttle_Min(void);
void ESC_Calibration(void);
int Is_iBus_Received(void);
void BNO080_Calibration(void);
void Encode_Msg_AHRS(unsigned char* telemetry_tx_buf);
// void Encode_Msg_GPS(unsigned char* telemetry_tx_buf);
// void Encode_Msg_PID_Gain(unsigned char* telemetry_tx_buf, unsigned char id, float p, float i, float d);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	float q[4];
	float quatRadianAccuracy;


	short gyro_x_offset = 10, gyro_y_offset = 15, gyro_z_offset = -1;
	unsigned char motor_arming_flag = 0;
	unsigned short iBus_SwA_Prev = 0;
	unsigned char iBus_rx_cnt = 0;
	unsigned short ccr1, ccr2, ccr3, ccr4;

	unsigned char altitude_hold_flag = 0;

	float yaw_heading_reference;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
//  void DWT_Init(void)
//  {
//      CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
//      DWT->CYCCNT = 0;
//      DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
//  }
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
//  DWT_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
 // MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_TIM5_Init();
 // MX_I2C1_Init();
 // MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  LL_TIM_EnableCounter(TIM3); //Buzzer

  LL_USART_EnableIT_RXNE(USART6); //Debug UART
  LL_USART_EnableIT_RXNE(UART4); //GPS
  LL_USART_EnableIT_RXNE(UART5); //FS-iA6B

  LL_TIM_EnableCounter(TIM5); //Motor PWM
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH4);

  LL_TIM_EnableCounter(TIM7); //10Hz, 50Hz, 1kHz loop
  LL_TIM_EnableIT_UPDATE(TIM7);


  TIM3->PSC = 1000;
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
  HAL_Delay(60);

  printf("Checking sensor connection..\n");


  if(BNO080_Initialization() != 0)
  {


	  printf("\nBNO080 failed. Program shutting down...\n");
	  while(1)
	  {

	  }
  }
  //BNO080_enableRotationVector(2500);
  BNO080_enableGameRotationVector(2500);

  if(ICM20602_Initialization() != 0)
  {
	  printf("\nICM-20602 failed. Program shutting down...\n");
	  while(1)
	  {
	  }
  }

  MS5611_Initialization();


  printf("All sensors OK!\n\n");

  M8N_Initialization();



  ICM20602_Writebyte(0x13, (gyro_x_offset*-2)>>8);
  ICM20602_Writebyte(0x14, (gyro_x_offset*-2));

  ICM20602_Writebyte(0x15, (gyro_y_offset*-2)>>8);
  ICM20602_Writebyte(0x16, (gyro_y_offset*-2));

  ICM20602_Writebyte(0x17, (gyro_z_offset*-2)>>8);
  ICM20602_Writebyte(0x18, (gyro_z_offset*-2));

  printf("Loading PID Gain...\n");
	PID_Init();
	printf("\nAll gains OK!\n\n");





  while(Is_iBus_Received() == 0)
  {
	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);

	  TIM3->PSC = 3000;
	  HAL_Delay(200);
	  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  HAL_Delay(200);
  }

  if(iBus.SwC == 2000)
  {
	  ESC_Calibration();
	  while(iBus.SwC != 1000)
	  {
		  Is_iBus_Received();
	  }
  }


  while(Is_iBus_Throttle_Min() == 0 || iBus.SwA == 2000)
  {
	  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);

	  TIM3->PSC = 1000;
	  HAL_Delay(70);

  }

  printf("Start\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
//	  uint32_t start = DWT->CYCCNT;
    /* USER CODE BEGIN 3 */
	  MS5611_Tick();
	  MS5611.filtered_pressure = (float)( MS5611.filtered_pressure * 0.9f + MS5611.pressure * 0.1f);
//	  printf("pressure : %f\t temperature :%f\t altitude : %f\t\n", (float)(MS5611.pressure), (float)(MS5611.temperature), (float)(MS5611.altitude));
//  	  uint32_t end = DWT->CYCCNT;
//  		    loop_1ms_time_us =
//  		        (end - start) / (SystemCoreClock / 1000000.0f);
//  		    printf("1ms loop time: %.2f us\r\n", loop_1ms_time_us);

	  if(tim7_1ms_flag == 1)
	  {
		  tim7_1ms_flag = 0;
			if(iBus.SwC == 1500 )
			{
				if(altitude_hold_flag == 0 && position_hold_flag == 0)
				{
					MS5611.pressure_setpoint = MS5611.filtered_pressure;
				    Reset_All_PID_Integrator();
				    altitude_hold_flag = 1;
				    position_hold_flag = 1;
				    setpoint_lat = posllh.lat;
				    setpoint_lon = posllh.lon;
				}

				Position_Hold_Calculation((float)setpoint_lat / 10000000.0f, (float)setpoint_lon / 10000000.0f, current_lat_deg, current_lon_deg,BNO080_Yaw, ICM20602.gyro_z );
				Double_Roll_Pitch_PID_Calculation(&pitch, (iBus.RV - 1500) * 0.1f + Position_pitch, BNO080_Pitch, ICM20602.gyro_x);
				Double_Roll_Pitch_PID_Calculation(&roll, (iBus.RH - 1500) * 0.1f + Position_roll, BNO080_Roll , ICM20602.gyro_y);
				Single_Altitude_PID_Calculation(&altitude, MS5611.pressure_setpoint,  MS5611.filtered_pressure);

		  if(iBus.LV < 1030 || motor_arming_flag == 0)
		  {
			  Reset_All_PID_Integrator();
		  }

		  if(iBus.LH < 1485 || iBus.LH > 1515)
		  {
			  yaw_heading_reference = BNO080_Yaw;
			  Single_Yaw_Rate_PID_Calculation(&yaw_rate, (iBus.LH - 1501), ICM20602.gyro_z);

			  ccr1 = 10500 + 500 + (iBus.LV - 1000) * 10 - pitch.in.pid_result + roll.in.pid_result - yaw_rate.pid_result + altitude.out.pid_result;
			  ccr2 = 10500 + 500 + (iBus.LV - 1000) * 10 + pitch.in.pid_result + roll.in.pid_result + yaw_rate.pid_result + altitude.out.pid_result;
			  ccr3 = 10500 + 500 + (iBus.LV - 1000) * 10 + pitch.in.pid_result - roll.in.pid_result - yaw_rate.pid_result + altitude.out.pid_result;
			  ccr4 = 10500 + 500 + (iBus.LV - 1000) * 10 - pitch.in.pid_result - roll.in.pid_result + yaw_rate.pid_result + altitude.out.pid_result;
		  }
		  else
		  {
			  Single_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_heading_reference, BNO080_Yaw, ICM20602.gyro_z);

			  ccr1 = 10500 + 500 + (iBus.LV - 1000) * 10 - pitch.in.pid_result + roll.in.pid_result - yaw_heading.pid_result + altitude.out.pid_result;
			  ccr2 = 10500 + 500 + (iBus.LV - 1000) * 10 + pitch.in.pid_result + roll.in.pid_result + yaw_heading.pid_result + altitude.out.pid_result;
			  ccr3 = 10500 + 500 + (iBus.LV - 1000) * 10 + pitch.in.pid_result - roll.in.pid_result - yaw_heading.pid_result + altitude.out.pid_result;
			  ccr4 = 10500 + 500 + (iBus.LV - 1000) * 10 - pitch.in.pid_result - roll.in.pid_result + yaw_heading.pid_result + altitude.out.pid_result;
		  }
//		  printf("%f\t%f\n", BNO080_Pitch, ICM20602.gyro_x);
//		  printf("%f\t%f\n", BNO080_Roll, ICM20602.gyro_y);
//		  printf("%f\t%f\n", BNO080_Yaw, ICM20602.gyro_z);
	  }
			else
			{
				altitude_hold_flag = 0;
				position_hold_flag = 0;
				  Double_Roll_Pitch_PID_Calculation(&pitch, (iBus.RV - 1500) * 0.1f, BNO080_Pitch, ICM20602.gyro_x);
				  Double_Roll_Pitch_PID_Calculation(&roll, (iBus.RH - 1500) * 0.1f, BNO080_Roll , ICM20602.gyro_y);

				  if(iBus.LV < 1030 || motor_arming_flag == 0)
				  {
					  Reset_All_PID_Integrator();
				  }

				  if(iBus.LH < 1485 || iBus.LH > 1515)
				  {
					  yaw_heading_reference = BNO080_Yaw;
					  Single_Yaw_Rate_PID_Calculation(&yaw_rate, (iBus.LH - 1500), ICM20602.gyro_z);

					  ccr1 = 10500 + 500 + (iBus.LV - 1000) * 10 - pitch.in.pid_result + roll.in.pid_result - yaw_rate.pid_result ;
					  ccr2 = 10500 + 500 + (iBus.LV - 1000) * 10 + pitch.in.pid_result + roll.in.pid_result + yaw_rate.pid_result ;
					  ccr3 = 10500 + 500 + (iBus.LV - 1000) * 10 + pitch.in.pid_result - roll.in.pid_result - yaw_rate.pid_result ;
					  ccr4 = 10500 + 500 + (iBus.LV - 1000) * 10 - pitch.in.pid_result - roll.in.pid_result + yaw_rate.pid_result ;
				  }
				  else
				  {
					  Single_Yaw_Heading_PID_Calculation(&yaw_heading, yaw_heading_reference, BNO080_Yaw, ICM20602.gyro_z);

					  ccr1 = 10500 + 500 + (iBus.LV - 1000) * 10 - pitch.in.pid_result + roll.in.pid_result - yaw_heading.pid_result ;
					  ccr2 = 10500 + 500 + (iBus.LV - 1000) * 10 + pitch.in.pid_result + roll.in.pid_result + yaw_heading.pid_result ;
					  ccr3 = 10500 + 500 + (iBus.LV - 1000) * 10 + pitch.in.pid_result - roll.in.pid_result - yaw_heading.pid_result ;
					  ccr4 = 10500 + 500 + (iBus.LV - 1000) * 10 - pitch.in.pid_result - roll.in.pid_result + yaw_heading.pid_result ;
				  }
			}
	  }
	  if(iBus.SwA == 2000 && iBus_SwA_Prev != 2000)
	  {
		  if(iBus.LV < 1010)
		  {
			  motor_arming_flag = 1;
			  yaw_heading_reference = BNO080_Yaw;
		  }
		  else
		  {
			  while(Is_iBus_Throttle_Min() == 0 || iBus.SwA == 2000)
			  {
				  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);

				  TIM3->PSC = 1000;
				  HAL_Delay(70);
				  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
				  HAL_Delay(70);
			  }
		  }
	  }
	  iBus_SwA_Prev = iBus.SwA;

	  if(iBus.SwA != 2000)
	  {
		  motor_arming_flag = 0;
	  }

	  if(motor_arming_flag == 1)
	  {
		  if(failsafe_flag == 0)
		  {
			  if(iBus.LV > 1030)
			  {
				  TIM5->CCR1 = ccr1 > 21000 ? 21000 : ccr1 < 11000 ? 11000 : ccr1;
				  TIM5->CCR2 = ccr2 > 21000 ? 21000 : ccr2 < 11000 ? 11000 : ccr2;
				  TIM5->CCR3 = ccr3 > 21000 ? 21000 : ccr3 < 11000 ? 11000 : ccr3;
				  TIM5->CCR4 = ccr4 > 21000 ? 21000 : ccr4 < 11000 ? 11000 : ccr4;
			  }
			  else
			  {
				  TIM5->CCR1 = 11000;
				  TIM5->CCR2 = 11000;
				  TIM5->CCR3 = 11000;
				  TIM5->CCR4 = 11000;
			  }
		  }
		  else
		  {
			  TIM5->CCR1 = 10500;
			  TIM5->CCR2 = 10500;
			  TIM5->CCR3 = 10500;
			  TIM5->CCR4 = 10500;
		  }
	  }
	  else
	  {
		  TIM5->CCR1 = 10500;
		  TIM5->CCR2 = 10500;
		  TIM5->CCR3 = 10500;
		  TIM5->CCR4 = 10500;
	  }

	     if(tim7_20ms_flag == 1 )
	  	   {
	  	 	  tim7_20ms_flag = 0;
	  	 		  	 	  Encode_Msg_AHRS(&telemetry_tx_buf[0]);

	  	 		  	 	  HAL_UART_Transmit_IT(&huart1, &telemetry_tx_buf[0], 20);


	}
	  if(BNO080_dataAvailable() == 1)
	  {
		  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_0);

		  q[0] = BNO080_getQuatI();
		  q[1] = BNO080_getQuatJ();
		  q[2] = BNO080_getQuatK();
		  q[3] = BNO080_getQuatReal();
		  quatRadianAccuracy = BNO080_getQuatRadianAccuracy();

		  Quaternion_Update(&q[0]);

		  BNO080_Roll = BNO080_Roll;
		  BNO080_Pitch = BNO080_Pitch;

		//  printf("%.2f\t%.2f\n", BNO080_Roll, BNO080_Pitch);
		 // printf("%.2f\n", BNO080_Yaw);
	  }

	  if(ICM20602_DataReady() == 1)
	  {
		  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_1);

		  ICM20602_Get3AxisGyroRawData(&ICM20602.gyro_x_raw);

		  ICM20602.gyro_x = ICM20602.gyro_x_raw * 2000.f / 32768.f;
		  ICM20602.gyro_y = ICM20602.gyro_y_raw * 2000.f / 32768.f;
		  ICM20602.gyro_z = ICM20602.gyro_z_raw * 2000.f / 32768.f;

		  ICM20602.gyro_x = ICM20602.gyro_x;
		  ICM20602.gyro_y = -ICM20602.gyro_y;
		  ICM20602.gyro_z = -ICM20602.gyro_z;

//		 printf("%d,%d,%d\n", ICM20602.gyro_x_raw, ICM20602.gyro_y_raw, ICM20602.gyro_z_raw);
//		 printf("%d,%d,%d\n", (int)(ICM20602.gyro_x*100), (int)(ICM20602.gyro_y*100), (int)(ICM20602.gyro_z*100));
	  }
		/* neo m8n gps -------------------------------------------------------------------------------*/
	  if(m8n_rx_cplt_flag == 1)
		  {
			  m8n_rx_cplt_flag = 0;

			  if(M8N_UBX_CHKSUM_Check(&m8n_rx_buf[0], 36) == 1)
			      {
			          LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_2);
			          M8N_UBX_NAV_POSLLH_Parsing(&m8n_rx_buf[0], &posllh);

			          // Chuyển sang độ để dùng trong Position_Hold_Calculation
			          current_lat_deg = (float)posllh.lat / 10000000.0f;
			          current_lon_deg = (float)posllh.lon / 10000000.0f;

			     //     printf("LAT: %.7f\tLON: %.7f\tHeight: %ld\n", current_lat_deg, current_lon_deg, posllh.height);
			      }
		  }
//	  if(tim7_100ms_flag == 1)
//	  {
//		  tim7_100ms_flag = 0;
//
//	  }



	  if(ibus_rx_cplt_flag == 1)
	  {
		  ibus_rx_cplt_flag = 0;
		  if(iBus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1)
		  {
			  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_2);

			  iBus_Parsing(&ibus_rx_buf[0], &iBus);
			  iBus_rx_cnt++;
			  if(iBus_isActiveFailsafe(&iBus) == 1)
			  {
				  failsafe_flag = 1;
			  }
			  else
			  {
				  failsafe_flag = 0;
			  }
//		 printf("%d\t%d\t%d\t%d\t%d\t%d\n",
//					  iBus.RH, iBus.RV, iBus.LV, iBus.LH, iBus.SwA, iBus.SwC);
//			  HAL_Delay(100);
		  }
	  }

	  if(tim7_1000ms_flag == 1)
	  {
		  tim7_1000ms_flag = 0;
		  if(iBus_rx_cnt == 0)
		  {
			  failsafe_flag = 2;
		  }
		  iBus_rx_cnt = 0;
	  }

	  if(failsafe_flag == 1 || failsafe_flag == 2 ||  iBus.SwC == 2000) //low_bat_flag == 1 ||
	  {
		  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  }
	  else
	  {
		  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH4);
	  }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int Is_iBus_Throttle_Min(void)
{
	if(ibus_rx_cplt_flag == 1)
	{
		ibus_rx_cplt_flag = 0;
		if(iBus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1)
		{
			iBus_Parsing(&ibus_rx_buf[0], &iBus);
			if(iBus.LV < 1010) return 1;
		}
	}

	return 0;
}

void ESC_Calibration(void)
{
	  TIM5->CCR1 = 21000;
	  TIM5->CCR2 = 21000;
	  TIM5->CCR3 = 21000;
	  TIM5->CCR4 = 21000;
	  HAL_Delay(7000);
	  TIM5->CCR1 = 10500;
	  TIM5->CCR2 = 10500;
	  TIM5->CCR3 = 10500;
	  TIM5->CCR4 = 10500;
	  HAL_Delay(8000);
}

int Is_iBus_Received(void)
{
	if(ibus_rx_cplt_flag == 1)
	{
		ibus_rx_cplt_flag = 0;
		if(iBus_Check_CHKSUM(&ibus_rx_buf[0], 32) == 1)
		{
			iBus_Parsing(&ibus_rx_buf[0], &iBus);
			return 1;
		}
	}

	return 0;
}



void Encode_Msg_AHRS(unsigned char* telemetry_tx_buf)
 {
 	  telemetry_tx_buf[0] = 0x46;
 	  telemetry_tx_buf[1] = 0x43;

 	  telemetry_tx_buf[2] = 0x10;

 	  telemetry_tx_buf[3] = (short)(BNO080_Roll*100);
 	  telemetry_tx_buf[4] = ((short)(BNO080_Roll*100))>>8;

 	  telemetry_tx_buf[5] = (short)(BNO080_Pitch*100);
 	  telemetry_tx_buf[6] = ((short)(BNO080_Pitch*100))>>8;

 	  telemetry_tx_buf[5] = (short)(ICM20602.gyro_x*100);
 	  telemetry_tx_buf[6] = ((short)(ICM20602.gyro_x*100))>>8;

 	  telemetry_tx_buf[7] = (unsigned short)(BNO080_Yaw*100);
 	  telemetry_tx_buf[8] = ((unsigned short)(BNO080_Yaw*100))>>8;

	  telemetry_tx_buf[9] = (short)(MS5611.altitude*10);
	  telemetry_tx_buf[10] = ((short)(MS5611.altitude*10))>>8;

 	  telemetry_tx_buf[11] = posllh.lat;
 	  telemetry_tx_buf[12] = posllh.lat>>8;
 	  telemetry_tx_buf[13] = posllh.lat>>16;
 	  telemetry_tx_buf[14] = posllh.lat>>24;

	  telemetry_tx_buf[15] = posllh.lon;
 	  telemetry_tx_buf[16] = posllh.lon>>8;
 	  telemetry_tx_buf[17] = posllh.lon>>16;
 	  telemetry_tx_buf[18] = posllh.lon>>24;

 	  telemetry_tx_buf[19] = 0xff;

 	  for(int i=0;i<19;i++) telemetry_tx_buf[19] = telemetry_tx_buf[19] - telemetry_tx_buf[i];
 }


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
