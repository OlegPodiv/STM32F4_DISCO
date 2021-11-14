/*
Header file for user tasks FreeRTOS
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_TASKS_H
#define __USER_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif
//---------------------------------------------
#include "stm32_ds3231.h"	
#include "main_module.h"	
// --------------------------------------------
// Extern handle 
// --------------------------------------------
extern UART_HandleTypeDef huart6;	
extern UART_HandleTypeDef huart2;	
extern I2C_HandleTypeDef hi2c1;	
/* Exported functions prototypes ---------------------------------------------*/
void ParseCommandForUART(uint8_t *buff, uint8_t len);
void ParseCommandForDisplay(uint8_t *buff, uint8_t len);
void ParseString(uint8_t *buff, uint8_t len);	
void vGetTime(void *pvParameters);
void SetTime(void);
void eeprom_test(void);
void ram_from_eeprom_read(uint16_t memAddr, uint16_t nByte);
void eeprom_from_ram_write(uint16_t memAddr, uint16_t nByte);
void on_key_event(void);	
#ifdef __cplusplus
}
#endif

#endif /* __USER_TASKS_H */

