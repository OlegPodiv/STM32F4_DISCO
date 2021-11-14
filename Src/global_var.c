/*
Файл глобальных переменных
*/

#include "global_var.h"	
#include "user_tasks.h"
// ------  I2C константы ------------------
const uint8_t EEPROM_PAGE = 32; // размер страницы для записи в еепром
const uint8_t d_size = sizeof(double); // размер типа double в байтах (8)
const uint8_t f_size = sizeof(float); // размер типа float в байтах (4)

const uint16_t memAddrSeason = 0x100U; // Начало зоны записи MENU_SEASON в EEPROM
const uint16_t memAddrBoilerState = 0x120; // Начало зоны записи MENU_TEMPERATURE в EEPROM
const uint16_t memAddrLastEvent = 0x140; // Начало зоны записи Last Event в EEPROM

const uint16_t memAddrTemperatures = 0x400; // Начало зоны записи MENU_TEMPERATURE в EEPROM
const uint16_t memAddrTimes = 0x1000; // Начало зоны записи MENU_TIME в EEPROM
const uint16_t devAddrEEPROM = (0x57 << 1); // HAL expects address to be shifted one bit to the left addr 57

extern T_BOILER_STATE boiler;
extern T_BOILER_STATE last_boiler;
extern T_MENU_TEMPERATURE temperatures;
extern T_MENU_TIME times;
extern T_MENU_SEASON season;
extern T_EVENT on_off;
extern T_EVENT ext_event;
extern T_EVENT last_event;
// ----------------------------------------------------------------------------
// ----- Инициализация параметров MENU_TEMPERATURE ----------------------------
T_MENU_TEMPERATURE menu_temperature_init(void)
{
	T_MENU_TEMPERATURE T;
	// Все температуры в градусах цельсия
	T.D10_min = 30; T.D10_ignition = 40; T.D10_work = 60; T.D10_maintain = 200; T.D10_max = 250;
	T.D3_min = 5; T.D3_pump = 50; T.D3_target = 65; T.D3_max = 94;
	T.D6_target = 45; T.D6_max = 80;
	T.Th02 = 35; T.Th04 = 75;			
	T.D01 = 3;	T.D02 = 10;	T.D04 = 10;	T.D05 = 10;
//boiler_state_save_eeprom(TURNED_OFF_STATE);
//menu_season_save_eeprom(SUMMER);	
menu_temperature_save_eeprom(T);
	return T;
}	
// -----------------------------------------------------------------------------
// ------------ Инициализация параметров MENU_TIME -----------------------------
T_MENU_TIME menu_time_init(void)
{
T_MENU_TIME MT;
	// Все временные промежутки в секундах
	MT.T01 = 60; 
	MT.T02 = 20;	
	MT.T03 = 60; 
	
	MT.T05 = 30*60; // old 30*60 - Общее время периода основного розжига
	MT.T06 = 30; // Предзагрузка время включения шнека горелки	
	MT.T07 = 5; // Время включения шнека горелки в режиме основного розжига 
	MT.T08 = 30*60; // old 30*60 Общее время периода повторного розжига
	MT.T09 = 2; // Время включения шнека бункера в режиме повторного розжига 	
	MT.T10 = 4; // Время включения шнека горелки в режиме повторного розжига
	
	
	MT.T15 = 60; 
	MT.T16 = 240;	
	MT.T17 = 20; // Время работы автоочистки
	MT.T18 = 20; // Время работы золоудаления 	
	MT.T19 = 20; 
	MT.T20 = 20;	
	MT.T21 = 5;
	MT.T22 = 5; 
	MT.T23 = 120; 
	MT.T24 = 15*60; 
	
	MT.T26 = 180*60; 
	
	MT.T28 = 3;	
	MT.T29 = 60;
	MT.P05 = 60; // Интервал времени по которому включаются шнеки горелки и бункера в режиме розжига
	MT.P06 = 15*60;	// Цикл автоочистки old 15*60
	MT.P07 = 30*60;	// Цикл золоудаления old 30*60
	MT.P08 = 30*60;	
	MT.P09 = 30*60;
	MT.D03 = 2;	
	MT.D06 = 2;	
// Вторичные

MT.T04 = 0.3*MT.T03;	
MT.T11 = 0.2*MT.P05;
MT.T12 = 0.4*MT.P05; 
MT.T13 = 0.4*MT.P05;	
MT.T14 = 0.4*MT.P05;	
MT.T25 = 0.5*MT.T07; // Время включения шнека бункера в режиме основного розжига
MT.T27 = 0.5*MT.T14;	

menu_time_save_eeprom(MT);
return MT;
}
// -----------------------------------------------------------------------------
// --------------------- Сохранение MENU_TEMPERATURE в EEPROM ------------------
void menu_temperature_save_eeprom(T_MENU_TEMPERATURE T)
{
	uint16_t memAddr = memAddrTemperatures;
	HAL_StatusTypeDef status = HAL_ERROR;

	uint16_t e_bytes = sizeof(T); // Размер структуры в байтах
	uint8_t d = (uint8_t)(e_bytes/EEPROM_PAGE);
	uint8_t m = e_bytes%EEPROM_PAGE;

	for (uint8_t i=0; i<d; i++)
	{
	if (HAL_I2C_Mem_Write(&hi2c1, devAddrEEPROM, memAddr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&T + i*EEPROM_PAGE, EEPROM_PAGE, HAL_MAX_DELAY)  != HAL_OK)// HAL_MAX_DELAY
	{Error_Handler();}
	memAddr += EEPROM_PAGE;
status = HAL_ERROR;
while(status != HAL_OK)  status = HAL_I2C_IsDeviceReady(&hi2c1, devAddrEEPROM, 1, HAL_MAX_DELAY); // wait ....
	}
	if (m != 0) 
	{
	if (HAL_I2C_Mem_Write(&hi2c1, devAddrEEPROM, memAddr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&T + d*EEPROM_PAGE, m, HAL_MAX_DELAY) != HAL_OK)
	{Error_Handler();}
	memAddr += m;
status = HAL_ERROR;
while(status != HAL_OK) status = HAL_I2C_IsDeviceReady(&hi2c1, devAddrEEPROM, 1, HAL_MAX_DELAY); // wait ....
	}
}
// -----------------------------------------------------------------------------
// ----------------------- Загрузка MENU_TEMPERATURE из EEPROM -----------------
T_MENU_TEMPERATURE menu_temperature_load_eeprom(void)
{
	T_MENU_TEMPERATURE t;
	HAL_StatusTypeDef status = HAL_ERROR;

while(status != HAL_OK)  status = HAL_I2C_IsDeviceReady(&hi2c1, devAddrEEPROM, 1, HAL_MAX_DELAY); // wait ....
HAL_I2C_Mem_Read(&hi2c1, devAddrEEPROM, memAddrTemperatures, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&t, sizeof(t), HAL_MAX_DELAY);		

return t;	
}
// -----------------------------------------------------------------------------
// ------ Сохранение MENU_TEMPERATURE в EEPROM страницами по EEPROM_PAGE -------
void menu_temperature_save_eeprom_pages(T_MENU_TEMPERATURE T)
{
	uint16_t memAddr = memAddrTemperatures;
	HAL_StatusTypeDef status = HAL_ERROR;

	uint16_t e_bytes = sizeof(T); // Размер структуры в байтах
	uint8_t f = sizeof(double); // Размер переменной типа float
	uint8_t d = e_bytes/f; // Кол-во элементов в переменной структуры

	for (uint8_t i=0; i<d; i++)
	{
	if (HAL_I2C_Mem_Write(&hi2c1, devAddrEEPROM, memAddr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&T + i*f, f , HAL_MAX_DELAY) != HAL_OK)
	{Error_Handler();}
memAddr += EEPROM_PAGE;
status = HAL_ERROR;
while(status != HAL_OK) status = HAL_I2C_IsDeviceReady(&hi2c1, devAddrEEPROM, 1, HAL_MAX_DELAY); // wait ....
	}
}
// -----------------------------------------------------------------------------
// ------- Загрузка MENU_TEMPERATURE из EEPROM страницами по EEPROM_PAGE -------
T_MENU_TEMPERATURE menu_temperature_load_eeprom_pages(void)
{
	T_MENU_TEMPERATURE t;
	uint16_t memAddr = memAddrTemperatures;
	HAL_StatusTypeDef status = HAL_ERROR;

	uint16_t e_bytes = sizeof(t); // Размер структуры в байтах
	uint8_t f = sizeof(double); // Размер переменной типа float
	uint8_t d = e_bytes/f; // Кол-во элементов в переменной структуры
	
	for (uint8_t i=0; i<d; i++)
	{
	status = HAL_ERROR;
	while(status != HAL_OK)  status = HAL_I2C_IsDeviceReady(&hi2c1, devAddrEEPROM, 1, HAL_MAX_DELAY); // wait ....

		HAL_I2C_Mem_Read(&hi2c1, devAddrEEPROM, memAddr, I2C_MEMADD_SIZE_16BIT,(uint8_t*)&t + i*f, f, HAL_MAX_DELAY);
		memAddr += EEPROM_PAGE;
	}		
return t;	
}
// -----------------------------------------------------------------------------
// --------------------- Сохранение MENU_TIME в EEPROM -------------------------
void menu_time_save_eeprom(T_MENU_TIME MT)
{
	uint16_t memAddr = memAddrTimes; // Начало зоны записи MENU_TIME в EEPROM
	HAL_StatusTypeDef status = HAL_ERROR;

	uint16_t e_bytes = sizeof(MT); // Размер структуры в байтах
	uint8_t d = (uint8_t)(e_bytes/EEPROM_PAGE);
	uint8_t m = e_bytes%EEPROM_PAGE;

	for (uint8_t i=0; i<d; i++)
	{
		if (HAL_I2C_Mem_Write(&hi2c1, devAddrEEPROM, memAddr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&MT+i*EEPROM_PAGE, EEPROM_PAGE, HAL_MAX_DELAY) != HAL_OK)
		{
			Error_Handler();
		}
memAddr += EEPROM_PAGE;			
status = HAL_ERROR;		
while(status != HAL_OK)  status = HAL_I2C_IsDeviceReady(&hi2c1, devAddrEEPROM, 1, HAL_MAX_DELAY); // wait ....
	}
	if (m != 0) 
		{
		if (HAL_I2C_Mem_Write(&hi2c1, devAddrEEPROM, memAddr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&MT+d*EEPROM_PAGE, m, HAL_MAX_DELAY) != HAL_OK)
			{
				Error_Handler();
			}
	memAddr += m;		
	status = HAL_ERROR;
	while(status != HAL_OK)  status = HAL_I2C_IsDeviceReady(&hi2c1, devAddrEEPROM, 1, HAL_MAX_DELAY); // wait ....
		}
}
// ------------------------------------------------------------------------------
// ----------------------- Загрузка MENU_TIME из EEPROM -------------------------
T_MENU_TIME menu_time_load_eeprom(void)
{
	T_MENU_TIME mt;
	HAL_StatusTypeDef status = HAL_ERROR;

	while(status != HAL_OK)  status = HAL_I2C_IsDeviceReady(&hi2c1, devAddrEEPROM, 1, HAL_MAX_DELAY); // wait ....
	HAL_I2C_Mem_Read(&hi2c1, devAddrEEPROM, memAddrTimes, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&mt, sizeof(mt), HAL_MAX_DELAY);
return mt;	
}
// ------------------------------------------------------------------------------
// ------ Сохранение MENU_TIME в EEPROM страницами по EEPROM_PAGE ---------------
void menu_time_save_eeprom_pages(T_MENU_TIME MT)
{
	uint16_t memAddr = memAddrTimes; // Начало зоны записи MENU_TEMPERATURE в EEPROM
	HAL_StatusTypeDef status = HAL_ERROR;

	uint16_t e_bytes = sizeof(MT); // Размер структуры в байтах
	uint8_t f = sizeof(double); // Размер переменной типа double
	uint8_t d = e_bytes/f; // Кол-во элементов в переменной структуры

	for (uint8_t i=0; i<d; i++)
	{
	if (HAL_I2C_Mem_Write(&hi2c1, devAddrEEPROM, memAddr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&MT + i*f, f , HAL_MAX_DELAY) != HAL_OK)
	{Error_Handler();}
	memAddr += EEPROM_PAGE;
status = HAL_ERROR;
while(status != HAL_OK) status = HAL_I2C_IsDeviceReady(&hi2c1, devAddrEEPROM, 1, HAL_MAX_DELAY); // wait ....
	}
}
// --------------------------------------------------------------------------------------------------------------
// ------- Загрузка MENU_TIME из EEPROM страницами по EEPROM_PAGE -------
T_MENU_TIME menu_time_load_eeprom_pages(void)
{
	T_MENU_TIME mt;
	uint16_t memAddr = memAddrTimes; // Начало зоны записи MENU_TEMPERATURE в EEPROM
	HAL_StatusTypeDef status = HAL_ERROR;

	uint16_t e_bytes = sizeof(mt); // Размер структуры в байтах
	uint8_t f = sizeof(double); // Размер переменной типа float
	uint8_t d = e_bytes/f; // Кол-во элементов в переменной структуры
	
	for (uint8_t i=0; i<d; i++)
	{
	status = HAL_ERROR;
	while(status != HAL_OK)  status = HAL_I2C_IsDeviceReady(&hi2c1, devAddrEEPROM, 1, HAL_MAX_DELAY); // wait ....

		HAL_I2C_Mem_Read(&hi2c1, devAddrEEPROM, memAddr, I2C_MEMADD_SIZE_16BIT,(uint8_t*)&mt + i*f, f, HAL_MAX_DELAY);
		memAddr += EEPROM_PAGE;
	}		
return mt;	
}
// --------------------------------------------------------------------------------------------------------------
// ----------- Сохранение переменной "Сезон" в EEPROM -----------------------------------------------------------
void menu_season_save_eeprom(T_MENU_SEASON sea)
{
	uint16_t memAddr = memAddrSeason; // Начало зоны записи MENU_SEASON в EEPROM
	HAL_StatusTypeDef status = HAL_ERROR;

if (HAL_I2C_Mem_Write(&hi2c1, devAddrEEPROM, memAddr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&sea,sizeof(sea), HAL_MAX_DELAY) != HAL_OK)
{Error_Handler();}

status = HAL_ERROR;
while(status != HAL_OK) status = HAL_I2C_IsDeviceReady(&hi2c1, devAddrEEPROM, 1, HAL_MAX_DELAY); // wait ....
}
// --------------------------------------------------------------------------------------------------------------
// ----------- Загрузка переменной "Сезон" из EEPROM -----------------------------------------------------------
T_MENU_SEASON menu_season_load_eeprom(void)
{
	T_MENU_SEASON sea;
	uint16_t memAddr = memAddrSeason; // Начало зоны записи MENU_SEASON в EEPROM
	HAL_StatusTypeDef status = HAL_ERROR;
	
	while(status != HAL_OK)  status = HAL_I2C_IsDeviceReady(&hi2c1, devAddrEEPROM, 1, HAL_MAX_DELAY); // wait ....

		HAL_I2C_Mem_Read(&hi2c1, devAddrEEPROM, memAddr, I2C_MEMADD_SIZE_16BIT,(uint8_t*)&sea, sizeof(sea), HAL_MAX_DELAY);
	
	return sea;
}	
// --------------------------------------------------------------------------------------------------------------
// ----------- Сохранение переменной "Состояние котла" в EEPROM -----------------------------------------------------------
void boiler_state_save_eeprom(T_BOILER_STATE B)
{
	uint16_t memAddr = memAddrBoilerState; // Начало зоны записи Boiler State в EEPROM
	HAL_StatusTypeDef status = HAL_ERROR;

if (HAL_I2C_Mem_Write(&hi2c1, devAddrEEPROM, memAddr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&B,sizeof(B), HAL_MAX_DELAY) != HAL_OK)
{Error_Handler();}

status = HAL_ERROR;
while(status != HAL_OK) status = HAL_I2C_IsDeviceReady(&hi2c1, devAddrEEPROM, 1, HAL_MAX_DELAY); // wait ..
	
}
// -----------------------------------------------------------------------------------------------------------------------
// ----------- Загрузка переменной "Состояние котла" из EEPROM -----------------------------------------------------------
T_BOILER_STATE boiler_state_load_eeprom(void)
{
	T_BOILER_STATE B;
	uint16_t memAddr = memAddrBoilerState; // Начало зоны записи Boiler State в EEPROM
	HAL_StatusTypeDef status = HAL_ERROR;
	
	while(status != HAL_OK)  status = HAL_I2C_IsDeviceReady(&hi2c1, devAddrEEPROM, 1, HAL_MAX_DELAY); // wait ....

		HAL_I2C_Mem_Read(&hi2c1, devAddrEEPROM, memAddr, I2C_MEMADD_SIZE_16BIT,(uint8_t*)&B, sizeof(B), HAL_MAX_DELAY);
	
	return B;
}	
// ------------------------------------------------------------------------------------------------------------------------
// ----------- Сохранение переменной "Последнее событие" в EEPROM -----------------------------------------------------------
void last_event_save_eeprom(T_EVENT Ev)
{
	uint16_t memAddr = memAddrLastEvent; // Начало зоны записи Last Event в EEPROM
	HAL_StatusTypeDef status = HAL_ERROR;

if (HAL_I2C_Mem_Write(&hi2c1, devAddrEEPROM, memAddr, I2C_MEMADD_SIZE_16BIT, (uint8_t*)&Ev, sizeof(Ev), HAL_MAX_DELAY) != HAL_OK)
{Error_Handler();}

status = HAL_ERROR;
while(status != HAL_OK) status = HAL_I2C_IsDeviceReady(&hi2c1, devAddrEEPROM, 1, HAL_MAX_DELAY); // wait ..
	
}
// ------------------------------------------------------------------------------------------------------------------------
// ----------- Загрузка переменной "Последнее событие" из EEPROM -----------------------------------------------------------
T_EVENT last_event_load_eeprom(void)
{
	T_EVENT Ev;
	uint16_t memAddr = memAddrLastEvent; // Начало зоны записи Last Event в EEPROM
	HAL_StatusTypeDef status = HAL_ERROR;
	
	while(status != HAL_OK)  status = HAL_I2C_IsDeviceReady(&hi2c1, devAddrEEPROM, 1, HAL_MAX_DELAY); // wait ....

		HAL_I2C_Mem_Read(&hi2c1, devAddrEEPROM, memAddr, I2C_MEMADD_SIZE_16BIT,(uint8_t*)&Ev, sizeof(Ev), HAL_MAX_DELAY);
	
	return Ev;
}	
// ------------------------------------------------------------------------------------------------------------------------
// ------------------------------------ Функция инициализации всех глобальных переменных ----------------------------------
void global_var_init(void)
{
	ext_event = NO_EVENT;
	on_off = OFF_EVENT;
	last_event = last_event_load_eeprom();
	if (last_event != OFF_EVENT) on_key_event();
	
	times = menu_time_load_eeprom();
	temperatures = menu_temperature_load_eeprom();
	boiler = boiler_state_load_eeprom();
	season = menu_season_load_eeprom();
	
}	
// ----------------------------------------------------------------------------------------
// Test записи и чтения параметров в EEPROM
// ----------------------------------------------------------------------------------------
void eeprom_comparing(T_MENU_TEMPERATURE t_eeprom,T_MENU_TIME mt_eeprom,T_MENU_TEMPERATURE temperatures,T_MENU_TIME times)
{
	// --------------------------------------------------------------------------------------------------------------------
	printf("D3pump= %5.f %5.f \r\n", temperatures.D3_pump, t_eeprom.D3_pump);	
	printf("t2= %5.f %5.f \r\n", temperatures.Th02, t_eeprom.Th02);	
	printf("D10min= %5.f %5.f \r\n", temperatures.D10_min, t_eeprom.D10_min);printf("t4= %5.f %5.f \r\n", temperatures.Th04,  t_eeprom.Th04);	
	printf("D10max= %5.f %5.f \r\n", temperatures.D10_max, t_eeprom.D10_max);	printf("D10igni= %5.f %5.f \r\n", temperatures.D10_ignition, t_eeprom.D10_ignition);	
	printf("D3targ= %5.f %5.f \r\n", temperatures.D3_target, t_eeprom.D3_target); printf("D10main= %5.f %5.f \r\n", temperatures.D10_maintain, t_eeprom.D10_maintain);	
	printf("D6targ= %5.f %5.f \r\n", temperatures.D6_target, t_eeprom.D6_target);	printf("D3min= %5.f %5.f \r\n", temperatures.D3_min, t_eeprom.D3_min);	
	printf("D3max= %5.f %5.f \r\n", temperatures.D3_max, t_eeprom.D3_max); printf("D6max= %5.f %5.f \r\n", temperatures.D6_max, t_eeprom.D6_max);
	printf("D10work= %5.f %5.f \r\n", temperatures.D10_work, t_eeprom.D10_work);
	printf("D01= %5.f %5.f \r\n", temperatures.D01, t_eeprom.D01);printf("D02= %5.f %5.f \r\n", temperatures.D02, t_eeprom.D02);	
	printf("D04= %5.f  %5.f \r\n", temperatures.D04, t_eeprom.D04);printf("D05= %5.f %5.f \r\n", temperatures.D05, t_eeprom.D05);
	printf("size= %u \r\n", sizeof(double));
	printf("T_MENU_TEMPERATURE size= %u \r\n", sizeof(T_MENU_TEMPERATURE));
	// ----------------------------------------------------------------------------------------------------------------------
  printf("T01= %5.1f %5.1f \r\n",times.T01, mt_eeprom.T01); printf("T02= %5.1f %5.1f \r\n",times.T02, mt_eeprom.T02);
	printf("T03= %5.1f %5.1f \r\n",times.T03, mt_eeprom.T03); printf("T04= %5.1f %5.1f \r\n",times.T04, mt_eeprom.T04);
	printf("T05= %5.1f %5.1f \r\n",times.T05, mt_eeprom.T05); printf("T06= %5.1f %5.1f \r\n",times.T06, mt_eeprom.T06);
	printf("T07= %5.1f %5.1f \r\n",times.T07, mt_eeprom.T07); printf("T08= %5.1f %5.1f \r\n",times.T08, mt_eeprom.T08);
	printf("T09= %5.1f %5.1f \r\n",times.T09, mt_eeprom.T09); printf("T10= %5.1f %5.1f \r\n",times.T10, mt_eeprom.T10);
	printf("T11= %5.1f %5.1f \r\n",times.T11, mt_eeprom.T11); printf("T12= %5.1f %5.1f \r\n",times.T12, mt_eeprom.T12);
	printf("T13= %5.1f %5.1f \r\n",times.T13, mt_eeprom.T13); printf("T14= %5.1f %5.1f \r\n",times.T14, mt_eeprom.T14);
	printf("T15= %5.1f %5.1f \r\n",times.T15, mt_eeprom.T15); printf("T16= %5.1f %5.1f \r\n",times.T16, mt_eeprom.T16);
	printf("T17= %5.1f %5.1f \r\n",times.T17, mt_eeprom.T17);	printf("T18= %5.1f %5.1f \r\n",times.T18, mt_eeprom.T18); 
	printf("T19= %5.1f %5.1f \r\n",times.T19, mt_eeprom.T19);	printf("T20= %5.1f %5.1f \r\n",times.T20, mt_eeprom.T20); 
	printf("T21= %5.1f %5.1f \r\n",times.T21, mt_eeprom.T21);	printf("T22= %5.1f %5.1f \r\n",times.T22, mt_eeprom.T22); 
	printf("T23= %5.1f %5.1f \r\n",times.T23, mt_eeprom.T23);	printf("T24= %5.1f %5.1f \r\n",times.T24, mt_eeprom.T24); 
	printf("T25= %5.1f %5.1f \r\n",times.T25, mt_eeprom.T25);	printf("T26= %5.1f %5.1f \r\n",times.T26, mt_eeprom.T26); 
	printf("T27= %5.1f %5.1f \r\n",times.T27, mt_eeprom.T27);	printf("T28= %5.1f %5.1f \r\n",times.T28, mt_eeprom.T28); 
	printf("T29= %5.1f %5.1f \r\n",times.T29, mt_eeprom.T29); 
	printf("P05= %5.1f %5.1f \r\n",times.P05, mt_eeprom.P05); printf("P06= %5.1f %5.1f \r\n",times.P06, mt_eeprom.P06);
	printf("P07= %5.1f %5.1f \r\n",times.P07, mt_eeprom.P07); printf("P08= %5.1f %5.1f \r\n",times.P08, mt_eeprom.P08);
	printf("P09= %5.1f %5.1f \r\n",times.P09, mt_eeprom.P09);
	printf("D03= %5.1f %5.1f \r\n",times.D03, mt_eeprom.D03); printf("D06= %5.1f %5.1f \r\n",times.D06, mt_eeprom.D06);
	printf("size= %u \r\n", sizeof(double));	
  printf("T_MENU_TIME size= %u \r\n", sizeof(T_MENU_TIME));
}

//******************************************************************************
// ENF OF FILE
//******************************************************************************

	
