/*
 * keyboard.h
 *
 *  Created on: 23 сент. 2021 г.
 *      Author: kovalchuk
 */

#ifndef KEYBOARD_KEYBOARD_H_
#define KEYBOARD_KEYBOARD_H_

#include "stm32f1xx_hal.h"
#include "main.h"

#define initMAX_BUTTONS_NUM 10
#define buttonPOLL_DELAY_MS	50

typedef enum {
	buttonKEY_ID_NONE = 0,
	buttonKEY_ID_OK = 1,
	buttonKEY_ID_RIGHT,
	buttonKEY_ID_LEFT,
	buttonKEY_ID_UP,
	buttonKEY_ID_DOWN,
	encoderKEY_ID
} ButtonKeyId_t;

typedef struct {
	ButtonKeyId_t Button_KeyID;			/*!< Идентификатор кнопки. Должен быть уникальным для каждой из используемых кнопок */
	GPIO_TypeDef  *ButtonGPIO;			// Порт GPIO с кнопкой
	uint32_t ButtonPin;					// Номер вывода GPIO, используемого кнопкой: GPIO_PINx
	GPIO_PinState ActivePinState;
	uint32_t ActiveStateCounterMS;
} ButtonDescription_t;

typedef struct
{
  ButtonDescription_t *ButtonDescriptionArray; 	  /*!< Указатель на массив структур с описаниями кнопок */
  uint16_t ButtonDescriptionArraySize;            /*!< Размер массива структур описаний кнопок */
} ButtonsInit_t;


void keyboardInit();
ButtonKeyId_t KeyboardPoll(void);

#endif /* KEYBOARD_KEYBOARD_H_ */
