/*
User Tasks for FreeRTOS

*/


// ------------- INCLUDES -----------------------------
#include <stdint.h>
#include <stdio.h>
#include <string.h>
//#include "stdio.h"
#include "cmsis_os.h"
#include "main.h"
#include "main_module.h"
#include "global_var.h"
//#include "stm32f4xx_hal_uart.h"
#include "user_tasks.h"

// -------------- Defines -----------------------------

// --------------- Global variable --------------------
extern uint8_t NumberUartConsole;
extern _RTC ds3231_rtc;
extern uint8_t buff_ds3231[8];
extern T_BIN_OUTPUTS binout;
extern T_BOILER_STATE boiler;
extern T_BOILER_STATE last_boiler;
extern T_MENU_TEMPERATURE temperatures;
extern T_MENU_TIME times;
extern T_MENU_SEASON season;
extern T_EVENT on_off;
extern T_EVENT ext_event;
extern T_EVENT last_event;

 uint8_t buffer_display[16] = {0,};
// ----------------------------------------------------
// Прототипы функций
// ----------------------------------------------------
void STDOUT_Flush(void);
uint16_t ConvertNumberToPin(uint8_t number);
GPIO_TypeDef* ConvertNumberToPort(uint8_t number);
uint8_t ConvertPinToNumber(uint16_t pin);
// ----------------------------------------------------
void ParseCommandForUART(uint8_t *buff, uint8_t len)
{
	uint8_t xx;
	xx = (buff[0] - 0x30)*10 + (buff[1]-0x30);	
											switch (xx){
												case 0: {
if (binout.HopperAuger.enable != 0) HAL_GPIO_WritePin(binout.HopperAuger.GPIOx, binout.HopperAuger.GPIO_Pin, GPIO_PIN_RESET);
if (binout.BurnerAuger.enable != 0)	HAL_GPIO_WritePin(binout.BurnerAuger.GPIOx, binout.BurnerAuger.GPIO_Pin, GPIO_PIN_RESET);
if (binout.Aux1.enable != 0) HAL_GPIO_WritePin(binout.Aux1.GPIOx, binout.Aux1.GPIO_Pin, GPIO_PIN_RESET);													
if (binout.HotWaterPump.enable != 0) HAL_GPIO_WritePin(binout.HotWaterPump.GPIOx, binout.HotWaterPump.GPIO_Pin, GPIO_PIN_RESET);
if (binout.Ignition.enable != 0 )	HAL_GPIO_WritePin(binout.Ignition.GPIOx, binout.Ignition.GPIO_Pin, GPIO_PIN_RESET);
if (binout.AutoClean.enable != 0)	HAL_GPIO_WritePin(binout.AutoClean.GPIOx, binout.AutoClean.GPIO_Pin, GPIO_PIN_RESET);
if (binout.AshRemove.enable != 0)	HAL_GPIO_WritePin(binout.AshRemove.GPIOx, binout.AshRemove.GPIO_Pin, GPIO_PIN_RESET);		
if (binout.CirclePump.enable != 0) HAL_GPIO_WritePin(binout.CirclePump.GPIOx, binout.CirclePump.GPIO_Pin, GPIO_PIN_RESET);
if (binout.FireSafetyValve.enable != 0)	HAL_GPIO_WritePin(binout.FireSafetyValve.GPIOx, binout.FireSafetyValve.GPIO_Pin, GPIO_PIN_RESET);
if (binout.AlarmDry.enable != 0)	HAL_GPIO_WritePin(binout.AlarmDry.GPIOx, binout.AlarmDry.GPIO_Pin, GPIO_PIN_RESET);														
													break;		
																}
												case 1: {
if (binout.HopperAuger.enable != 0) HAL_GPIO_WritePin(binout.HopperAuger.GPIOx,binout.HopperAuger.GPIO_Pin, GPIO_PIN_SET);
													break;
																}
												case 2: {
if (binout.BurnerAuger.enable != 0)	HAL_GPIO_WritePin(binout.BurnerAuger.GPIOx,binout.BurnerAuger.GPIO_Pin, GPIO_PIN_SET);
													break;
																}
												case 3: {
if (binout.Aux1.enable != 0) HAL_GPIO_WritePin(binout.Aux1.GPIOx, binout.Aux1.GPIO_Pin, GPIO_PIN_SET);
													break;
																}
												case 4: {
if (binout.HotWaterPump.enable != 0) HAL_GPIO_WritePin(binout.HotWaterPump.GPIOx, binout.HotWaterPump.GPIO_Pin, GPIO_PIN_SET);
													break;
																}
												case 5: {
if (binout.Ignition.enable != 0 )	HAL_GPIO_WritePin(binout.Ignition.GPIOx, binout.Ignition.GPIO_Pin, GPIO_PIN_SET);
													break;
																}
												case 6: {
if (binout.AutoClean.enable != 0)	HAL_GPIO_WritePin(binout.AutoClean.GPIOx, binout.AutoClean.GPIO_Pin, GPIO_PIN_SET);
													break;
																}
												case 7: {
if (binout.AshRemove.enable != 0)	HAL_GPIO_WritePin(binout.AshRemove.GPIOx, binout.AshRemove.GPIO_Pin, GPIO_PIN_SET);											
													break;
																}
												case 8: {
if (binout.CirclePump.enable != 0)	HAL_GPIO_WritePin(binout.CirclePump.GPIOx, binout.CirclePump.GPIO_Pin, GPIO_PIN_SET);				
													break;
																}
												case 9: {
if (binout.FireSafetyValve.enable != 0)	HAL_GPIO_WritePin(binout.FireSafetyValve.GPIOx, binout.FireSafetyValve.GPIO_Pin, GPIO_PIN_SET);	
													break;
																}
												case 10: {
if (binout.AlarmDry.enable != 0)	HAL_GPIO_WritePin(binout.AlarmDry.GPIOx, binout.AlarmDry.GPIO_Pin, binout.AlarmDry.PinStateActive);			
													break;
																}
												default: break;
																	}
	
}	

// --------------------------------------------------------------------------------------
void ParseCommandForDisplay(uint8_t *buff, uint8_t len)
{
	uint16_t cnt_pwm1, cnt_pwm2, cnt_pwm3, cnt_pwm4;
		uint8_t id_display;
if (buff[0] == 0x71) 
	{		
		id_display = buff[1] + buff[2]*10;

											switch (id_display){
												case 1: {
if (binout.HopperAuger.enable != 0) HAL_GPIO_WritePin(binout.HopperAuger.GPIOx, binout.HopperAuger.GPIO_Pin, GPIO_PIN_RESET);
if (binout.BurnerAuger.enable != 0)	HAL_GPIO_WritePin(binout.BurnerAuger.GPIOx, binout.BurnerAuger.GPIO_Pin, GPIO_PIN_RESET);
if (binout.Aux1.enable != 0) HAL_GPIO_WritePin(binout.Aux1.GPIOx, binout.Aux1.GPIO_Pin, GPIO_PIN_RESET);													
if (binout.HotWaterPump.enable != 0) HAL_GPIO_WritePin(binout.HotWaterPump.GPIOx, binout.HotWaterPump.GPIO_Pin, GPIO_PIN_RESET);
if (binout.Ignition.enable != 0 )	HAL_GPIO_WritePin(binout.Ignition.GPIOx, binout.Ignition.GPIO_Pin, GPIO_PIN_RESET);
if (binout.AutoClean.enable != 0)	HAL_GPIO_WritePin(binout.AutoClean.GPIOx, binout.AutoClean.GPIO_Pin, GPIO_PIN_RESET);
if (binout.AshRemove.enable != 0)	HAL_GPIO_WritePin(binout.AshRemove.GPIOx, binout.AshRemove.GPIO_Pin, GPIO_PIN_RESET);		
if (binout.CirclePump.enable != 0) HAL_GPIO_WritePin(binout.CirclePump.GPIOx, binout.CirclePump.GPIO_Pin, GPIO_PIN_RESET);
if (binout.FireSafetyValve.enable != 0)	HAL_GPIO_WritePin(binout.FireSafetyValve.GPIOx, binout.FireSafetyValve.GPIO_Pin, GPIO_PIN_RESET);
if (binout.AlarmDry.enable != 0)	HAL_GPIO_WritePin(binout.AlarmDry.GPIOx, binout.AlarmDry.GPIO_Pin, GPIO_PIN_RESET);		
													break;		
																}
												case 2: {
if (binout.HopperAuger.enable != 0) HAL_GPIO_WritePin(binout.HopperAuger.GPIOx, binout.HopperAuger.GPIO_Pin, GPIO_PIN_SET);
													break;
																}
												case 3: {
if (binout.BurnerAuger.enable != 0)	HAL_GPIO_WritePin(binout.BurnerAuger.GPIOx, binout.BurnerAuger.GPIO_Pin, GPIO_PIN_SET);
													break;
																}
												case 4: {
if (binout.Aux1.enable != 0) HAL_GPIO_WritePin(binout.Aux1.GPIOx, binout.Aux1.GPIO_Pin, GPIO_PIN_SET);													
													break;
																}
												case 5: {
if (binout.HotWaterPump.enable != 0) HAL_GPIO_WritePin(binout.HotWaterPump.GPIOx, binout.HotWaterPump.GPIO_Pin, GPIO_PIN_SET);
													break;
																}
												case 6: {
if (binout.Ignition.enable != 0 )	HAL_GPIO_WritePin(binout.Ignition.GPIOx, binout.Ignition.GPIO_Pin, GPIO_PIN_SET);
													break;
																}
												case 7: {
if (binout.AutoClean.enable != 0)	HAL_GPIO_WritePin(binout.AutoClean.GPIOx, binout.AutoClean.GPIO_Pin, GPIO_PIN_SET);
													break;
																}
												case 8: {
if (binout.AshRemove.enable != 0)	HAL_GPIO_WritePin(binout.AshRemove.GPIOx, binout.AshRemove.GPIO_Pin, GPIO_PIN_SET);		
													break;
																}
												case 33: {
if (binout.CirclePump.enable != 0) HAL_GPIO_WritePin(binout.CirclePump.GPIOx, binout.CirclePump.GPIO_Pin, GPIO_PIN_SET);
													break;
																}
												case 9: {
													cnt_pwm1 = (buff[9] + buff[10]*256)<<3;
													TIM4->CCR1= cnt_pwm1; //0x18F + cnt_pwm1;
														break;
																}
												case 10: {
													cnt_pwm2 = (buff[9]+ buff[10]*256)<<3;
													TIM4->CCR2=cnt_pwm2;
														break;
																}
												case 11: {
												cnt_pwm3 = (buff[9] + buff[10]*256)<<3;
												TIM4->CCR3=cnt_pwm3;
													break;
																}
												case 12: {
												cnt_pwm4 = (buff[9] + buff[10]*256)<<3;
												TIM4->CCR4=cnt_pwm4;
													break;
																}
												default: break;
														}	
												
	}	
	if (buff[0] == 0x77) 
	{
		binout.HopperAuger.enable = buff[1];
		binout.HopperAuger.GPIOx = ConvertNumberToPort(buff[2]);
		binout.HopperAuger.GPIO_Pin = ConvertNumberToPin(buff[2]);
		
		binout.BurnerAuger.enable = buff[3];
		binout.BurnerAuger.GPIOx = ConvertNumberToPort(buff[4]);
		binout.BurnerAuger.GPIO_Pin = ConvertNumberToPin(buff[4]);
		
		binout.AutoClean.enable = buff[5];
		binout.AutoClean.GPIOx = ConvertNumberToPort(buff[6]);
		binout.AutoClean.GPIO_Pin = ConvertNumberToPin(buff[6]);
		
		binout.AshRemove.enable = buff[7];
		binout.AshRemove.GPIOx = ConvertNumberToPort(buff[8]);
		binout.AshRemove.GPIO_Pin = ConvertNumberToPin(buff[8]);
		
		binout.CirclePump.enable = buff[9];
		binout.CirclePump.GPIOx = ConvertNumberToPort(buff[10]);
		binout.CirclePump.GPIO_Pin = ConvertNumberToPin(buff[10]);
		
		binout.HotWaterPump.enable = buff[11];
		binout.HotWaterPump.GPIOx = ConvertNumberToPort(buff[12]);
		binout.HotWaterPump.GPIO_Pin = ConvertNumberToPin(buff[12]);
		
		binout.Ignition.enable = buff[13];
		binout.Ignition.GPIOx = ConvertNumberToPort(buff[14]);
		binout.Ignition.GPIO_Pin = ConvertNumberToPin(buff[14]);
		
		binout.FireSafetyValve.enable = buff[15];
		binout.FireSafetyValve.GPIOx = ConvertNumberToPort(buff[16]);
		binout.FireSafetyValve.GPIO_Pin = ConvertNumberToPin(buff[16]);
		
		binout.AlarmDry.enable = buff[17];
		binout.AlarmDry.GPIOx = ConvertNumberToPort(buff[18]);
		binout.AlarmDry.GPIO_Pin = ConvertNumberToPin(buff[18]);
		
		binout.Aux1.enable = buff[19];
		binout.Aux1.GPIOx = ConvertNumberToPort(buff[20]);
		binout.Aux1.GPIO_Pin = ConvertNumberToPin(buff[20]);
//	}	
//	if (buff[0] == 0x72)
//	{
		NumberUartConsole=2;
  	printf("outassign.HopperAugerC.val=%u%c%c%c", binout.HopperAuger.enable,0xFF,0xFF,0xFF);
		printf("outassign.HopperAugerN.val=%u%c%c%c", ConvertPinToNumber(binout.HopperAuger.GPIO_Pin),0xFF,0xFF,0xFF);
		
		printf("outassign.BurnerAugerC.val=%u%c%c%c", binout.BurnerAuger.enable,0xFF,0xFF,0xFF);
		printf("outassign.BurnerAugerN.val=%u%c%c%c", ConvertPinToNumber(binout.BurnerAuger.GPIO_Pin),0xFF,0xFF,0xFF);
		
		printf("outassign.AutoCleanC.val=%u%c%c%c", binout.AutoClean.enable,0xFF,0xFF,0xFF);
		printf("outassign.AutoCleanN.val=%u%c%c%c", ConvertPinToNumber(binout.AutoClean.GPIO_Pin),0xFF,0xFF,0xFF);
		
		printf("outassign.AshRemoveC.val=%u%c%c%c", binout.AshRemove.enable,0xFF,0xFF,0xFF);
		printf("outassign.AshRemoveN.val=%u%c%c%c", ConvertPinToNumber(binout.AshRemove.GPIO_Pin),0xFF,0xFF,0xFF);
		
		printf("outassign.CirclePumpC.val=%u%c%c%c", binout.CirclePump.enable,0xFF,0xFF,0xFF);
		printf("outassign.CirclePumpN.val=%u%c%c%c", ConvertPinToNumber(binout.CirclePump.GPIO_Pin),0xFF,0xFF,0xFF);
		
		printf("outassign.HotWaterPumpC.val=%u%c%c%c", binout.HotWaterPump.enable,0xFF,0xFF,0xFF);
		printf("outassign.HotWaterPumpN.val=%u%c%c%c", ConvertPinToNumber(binout.HotWaterPump.GPIO_Pin),0xFF,0xFF,0xFF);
		
		printf("outassign.IgnitionC.val=%u%c%c%c", binout.Ignition.enable,0xFF,0xFF,0xFF);
		printf("outassign.IgnitionN.val=%u%c%c%c", ConvertPinToNumber(binout.Ignition.GPIO_Pin),0xFF,0xFF,0xFF);
		
		printf("outassign.FiSafeValveC.val=%u%c%c%c", binout.FireSafetyValve.enable,0xFF,0xFF,0xFF);
		printf("outassign.FiSafeValveN.val=%u%c%c%c", ConvertPinToNumber(binout.FireSafetyValve.GPIO_Pin),0xFF,0xFF,0xFF);
		
		printf("outassign.AlarmDryC.val=%u%c%c%c", binout.AlarmDry.enable,0xFF,0xFF,0xFF);
		printf("outassign.AlarmDryN.val=%u%c%c%c", ConvertPinToNumber(binout.AlarmDry.GPIO_Pin),0xFF,0xFF,0xFF);
		
		printf("outassign.Aux1C.val=%u%c%c%c", binout.Aux1.enable,0xFF,0xFF,0xFF);
		printf("outassign.Aux1N.val=%u%c%c%c", ConvertPinToNumber(binout.Aux1.GPIO_Pin),0xFF,0xFF,0xFF);
	}		
}
// --------------------------------------------------------------------------------------
void vGetTime(void *pvParameters)
{
	while(1){
	if (DS3231_GetTime(&ds3231_rtc))
	{
		buff_ds3231[0] = 0x0;
		buff_ds3231[1] = ds3231_rtc.Date;
		buff_ds3231[2] = ds3231_rtc.Year;
		buff_ds3231[3] = ds3231_rtc.Month;
		buff_ds3231[4] = ds3231_rtc.DaysOfWeek;
		buff_ds3231[5] = ds3231_rtc.Hour;
		buff_ds3231[6] = ds3231_rtc.Min;
		buff_ds3231[7] = ds3231_rtc.Sec;

//		NumberUartConsole = 6;
//		printf("sec.val=%u%c%c%c", buff_ds3231[7],0xFF,0xFF,0xFF);
//		printf("min.val=%u%c%c%c", buff_ds3231[6],0xFF,0xFF,0xFF);
//		printf("hour.val=%u%c%c%c", buff_ds3231[5],0xFF,0xFF,0xFF);
//		printf("day.val=%u%c%c%c", buff_ds3231[4],0xFF,0xFF,0xFF);

//		sprintf((char *)buffer_display, "sec.val=%u%c%c%c", buff_ds3231[7],0xFF,0xFF,0xFF);
//		HAL_UART_Transmit_IT(&huart6, (uint8_t *)&buffer_display, 16);
//		//--------------------------------------------------------------------
//		sprintf((char *)buffer_display,"min.val=%u%c%c%c", buff_ds3231[6],0xFF,0xFF,0xFF);
//		HAL_UART_Transmit_IT(&huart6, (uint8_t *)&buffer_display, 16);
//		//--------------------------------------------------------------------
//		sprintf((char *)buffer_display,"hour.val=%u%c%c%c", buff_ds3231[5],0xFF,0xFF,0xFF);
//		HAL_UART_Transmit_IT(&huart6, (uint8_t *)&buffer_display, 16);
//		//--------------------------------------------------------------------
//		sprintf((char *)buffer_display,"day.val=%u%c%c%c", buff_ds3231[4],0xFF,0xFF,0xFF);
//		HAL_UART_Transmit_IT(&huart6, (uint8_t *)&buffer_display, 16);
	}
	vTaskDelay(200);
}
}
void SetTime(void)
{
//		ds3231_rtc.Date = 			buff_ds3231[1];
//		ds3231_rtc.Year = 			buff_ds3231[2];
//		ds3231_rtc.Month = 			buff_ds3231[3];
//		ds3231_rtc.DaysOfWeek = buff_ds3231[4];
//		ds3231_rtc.Hour = 			buff_ds3231[5];
//		ds3231_rtc.Min = 				buff_ds3231[6];
//		ds3231_rtc.Sec = 				buff_ds3231[7];
//		ds3231_rtc.Date = 			14;
//		ds3231_rtc.Year = 			19;
//		ds3231_rtc.Month = 			11;
//		ds3231_rtc.DaysOfWeek = 5;
//		ds3231_rtc.Hour = 			20;
//		ds3231_rtc.Min = 				5;
//		ds3231_rtc.Sec = 				0;		
//if (DS3231_SetTime(&ds3231_rtc))
//					{
//						 Error_Handler();
//					}
}
// ---------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------
void ParseString(uint8_t *buff, uint8_t len)
{
	char charbuff[len];
	for (int i = 0; i<len; i++)	{	charbuff[i] = buff[i];	}
	
	switch(charbuff[0]){
		case 'p' : {
								if (charbuff[5] == '1') HAL_GPIO_WritePin(GPIOE, Relay_4_Pin, GPIO_PIN_SET);
								if (charbuff[5] == '0')	HAL_GPIO_WritePin(GPIOE, Relay_4_Pin, GPIO_PIN_RESET);
								break;
								}
		case 'f' : {
								switch (charbuff[1]){
									case 'i': {
														} 
									case 'a': {
														}
									default : break;	
								}
								break;
								}
		case 's' : {}	
		default  : break;
	}
		
}	
// ---------------- Преобразование номера реле в название пина ------------------------------------------
uint16_t ConvertNumberToPin(uint8_t number)
{
	uint16_t Pin;
			switch(number){
				case 0: Pin = Relay_0_Pin;
								break;
				case 1: Pin = Relay_1_Pin;
								break;
				case 2: Pin = Relay_2_Pin;
								break;
				case 3: Pin = Relay_3_Pin;
								break;
				case 4: Pin = Relay_4_Pin;
								break;
				case 5: Pin = Relay_5_Pin;
								break;
				case 6: Pin = Relay_6_Pin;
								break;
				case 7: Pin = Relay_7_Pin;
								break;
				case 8: Pin = Relay_8_Pin;
								break;
				case 9: Pin = Relay_9_Pin;
								break;
				default: break;
			}
return Pin;
}
// --------------------- Преобразование названия пина в номер реле ------------------------------------------
// ---------------- Преобразование номера реле в название пина ------------------------------------------
uint8_t ConvertPinToNumber(uint16_t pin)
{
	uint8_t Number;
			switch(pin){
				case Relay_0_Pin: Number = 0;
								break;
				case Relay_1_Pin: Number = 1;
								break;
				case Relay_2_Pin: Number = 2;
								break;
				case Relay_3_Pin: Number = 3;
								break;
				case Relay_4_Pin: Number = 4;
								break;
				case Relay_5_Pin: Number = 5;
								break;
				case Relay_6_Pin: Number = 6;
								break;
				case Relay_7_Pin: Number = 7;
								break;
				case Relay_8_Pin: Number = 8;
								break;
				case Relay_9_Pin: Number = 9;
								break;
				default: break;
			}
return Number;
}
// --------------------- Преобразование номера реле в название порта ----------------------------------------
GPIO_TypeDef* ConvertNumberToPort(uint8_t number)
{
	GPIO_TypeDef* Port;
			switch(number){
				case 0: Port = Relay_0_GPIO_Port;
								break;
				case 1: Port = Relay_1_GPIO_Port;
								break;
				case 2: Port = Relay_2_GPIO_Port;
								break;
				case 3: Port = Relay_3_GPIO_Port;
								break;
				case 4: Port = Relay_4_GPIO_Port;
								break;
				case 5: Port = Relay_5_GPIO_Port;
								break;
				case 6: Port = Relay_6_GPIO_Port;
								break;
				case 7: Port = Relay_7_GPIO_Port;
								break;
				case 8: Port = Relay_8_GPIO_Port;
								break;
				case 9: Port = Relay_9_GPIO_Port;
								break;
				default: break;
			}
return Port;
}
// ----------------------------------------------------------------------------------------
// Инициализация переменных, сохраненных в EEPROM
// ----------------------------------------------------------------------------------------
void ram_from_eeprom_read(uint16_t memAddr, uint16_t nByte)
{
	uint16_t devAddr = (0x57 << 1); // HAL expects address to be shifted one bit to the left addr 57
	uint8_t ram[nByte];
	HAL_StatusTypeDef status;

	for(;;) { // wait...
		status = HAL_I2C_IsDeviceReady(&hi2c1, devAddr, 1, HAL_MAX_DELAY);
		if(status == HAL_OK)
			break;
	}
	HAL_I2C_Mem_Read(&hi2c1, devAddr, memAddr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)ram, sizeof(ram), HAL_MAX_DELAY);
}
// ----------------------------------------------------------------------------------------
// Запись блока данных из памяти в EEPROM
// ----------------------------------------------------------------------------------------
void eeprom_from_ram_write(uint16_t memAddr, uint16_t nByte)
{
	uint8_t ram[nByte];
	uint16_t devAddr = (0x57 << 1); // HAL expects address to be shifted one bit to the left addr 57
	HAL_StatusTypeDef status;

	if (HAL_I2C_Mem_Write(&hi2c1, devAddr, memAddr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)ram, sizeof(ram), HAL_MAX_DELAY) != HAL_OK)
	{
		Error_Handler();
	}
for(;;) { // wait...
		status = HAL_I2C_IsDeviceReady(&hi2c1, devAddr, 1, HAL_MAX_DELAY);
		if(status == HAL_OK)
			break;
	}
}
// ----------------------------------------------------------------------------------------
// Test записи и чтения EEPROM
// ----------------------------------------------------------------------------------------
void eeprom_test(void)
{
	const char wmsg[] = "Some data";
	char rmsg[sizeof(wmsg)];
	uint16_t devAddr = (0x57 << 1); // HAL expects address to be shifted one bit to the left addr 57
	uint16_t memAddr = 0x1000;
	HAL_StatusTypeDef status;

	const char writing[] = "Starting the test - writing to the memory...\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t*)writing, sizeof(writing)-1, HAL_MAX_DELAY);

	// Hint: try to comment this line!
	if (HAL_I2C_Mem_Write(&hi2c1, devAddr, memAddr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)wmsg, sizeof(wmsg), HAL_MAX_DELAY) != HAL_OK)
	{const char waiting[] = "ERROR, writing to the memory!!!\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t*)waiting, sizeof(waiting)-1, HAL_MAX_DELAY);
	}
	else{
	const char waiting[] = "OK, now waiting until device is ready...\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t*)waiting, sizeof(waiting)-1, HAL_MAX_DELAY);

	for(;;) { // wait...
		status = HAL_I2C_IsDeviceReady(&hi2c1, devAddr, 1, HAL_MAX_DELAY);
		if(status == HAL_OK)
			break;
	}
}

	const char reading[] = "Device is ready, now reading...\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t*)reading, sizeof(reading)-1, HAL_MAX_DELAY);

	HAL_I2C_Mem_Read(&hi2c1, devAddr, memAddr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)rmsg, sizeof(rmsg), HAL_MAX_DELAY);

	const char comparing[] = "Done, now comparing...\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t*)comparing, sizeof(comparing)-1, HAL_MAX_DELAY);

	if(memcmp(rmsg, wmsg, sizeof(rmsg)) == 0) {
		const char result[] = "Test passed!\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)result, sizeof(result)-1, HAL_MAX_DELAY);
	} else {
		const char result[] = "Test failed :(\r\n";
		HAL_UART_Transmit(&huart2, (uint8_t*)result, sizeof(result)-1, HAL_MAX_DELAY);

		char tmp_buff[64];
		snprintf(tmp_buff, sizeof(tmp_buff), "rmsg = '%s'\r\n", rmsg);
		HAL_UART_Transmit(&huart2, (uint8_t*)tmp_buff, strlen(tmp_buff), HAL_MAX_DELAY);
	}
}

// ------------ Функция-обработчик события нажатия клавиши ON ----------------------------------------------------------
void on_key_event(void)
{
if (on_off == OFF_EVENT) 
		{
			on_off = ON_EVENT; 
			last_event_save_eeprom(ON_EVENT);
			HAL_GPIO_WritePin(LED_ON_GPIO_Port, LED_ON_Pin, GPIO_PIN_SET);
		}
	else 
		{	
			on_off = OFF_EVENT; 
			last_event_save_eeprom(OFF_EVENT);
			HAL_GPIO_WritePin(LED_ON_GPIO_Port, LED_ON_Pin, GPIO_PIN_RESET);
		}	
}	
// ---------------------------------------------------------------------------------------------------------------------
