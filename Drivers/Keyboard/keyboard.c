/*
 * keyboard.c
 *
 *  Created on: 23 сент. 2021 г.
 *      Author: kovalchuk
 */

#include "keyboard.h"
#include "stm32f1xx_it.h"
#include "stdbool.h"

static ButtonsInit_t xButtonsInitStruct;
static ButtonDescription_t xButtonDescriptionArray[initMAX_BUTTONS_NUM];

void keyboardInit() {

	uint32_t ulCounter = 0;
	xButtonDescriptionArray[ulCounter].Button_KeyID = buttonKEY_ID_LEFT;
	xButtonDescriptionArray[ulCounter].ActivePinState = GPIO_PIN_RESET;
	xButtonDescriptionArray[ulCounter].ActiveStateCounterMS = 0;
	xButtonDescriptionArray[ulCounter].ButtonGPIO = Button_Left_GPIO_Port;
	xButtonDescriptionArray[ulCounter].ButtonPin = Button_Left_Pin;
	ulCounter++;

	xButtonDescriptionArray[ulCounter].Button_KeyID = buttonKEY_ID_OK;
	xButtonDescriptionArray[ulCounter].ActivePinState = GPIO_PIN_RESET;
	xButtonDescriptionArray[ulCounter].ActiveStateCounterMS = 0;
	xButtonDescriptionArray[ulCounter].ButtonGPIO = Button_Ok_GPIO_Port;
	xButtonDescriptionArray[ulCounter].ButtonPin = Button_Ok_Pin;
	ulCounter++;

	xButtonDescriptionArray[ulCounter].Button_KeyID = encoderKEY_ID;
	xButtonDescriptionArray[ulCounter].ActivePinState = GPIO_PIN_RESET;
	xButtonDescriptionArray[ulCounter].ActiveStateCounterMS = 0;
	xButtonDescriptionArray[ulCounter].ButtonGPIO = Enkoder_Key_GPIO_Port;
	xButtonDescriptionArray[ulCounter].ButtonPin = Enkoder_Key_Pin;
	ulCounter++;

	xButtonsInitStruct.ButtonDescriptionArray = xButtonDescriptionArray;
	xButtonsInitStruct.ButtonDescriptionArraySize = ulCounter;
}


ButtonKeyId_t KeyboardPoll(void) {
	static ButtonDescription_t *activeButtonDescription = NULL;
	static bool ignoreKeysFlag = false;
	ButtonDescription_t *ButtonDescription = xButtonsInitStruct.ButtonDescriptionArray;

	if ( ignoreKeysFlag ) {					// Игнорируем нажатие заданное количество мс, это же время автоповтора при удержании
		if ( getBtnElapsedTime() == 0 ) {	// Когда время игнорирования вышло, разрешаем опрос кнопок при следующем вызове
			ignoreKeysFlag = false;
		}
		return buttonKEY_ID_NONE;
	}

	if ( activeButtonDescription == NULL) {
		for ( int i = 0; i < xButtonsInitStruct.ButtonDescriptionArraySize; i++ ) {
			if( HAL_GPIO_ReadPin(ButtonDescription->ButtonGPIO, ButtonDescription->ButtonPin) == ButtonDescription->ActivePinState ) {
				activeButtonDescription = ButtonDescription;
				setBtnWaitTime(BUTTON_WAITING_TIME);
				return buttonKEY_ID_NONE;
				//return ButtonDescription->Button_KeyID;
			}
			ButtonDescription++;
		}
	}
	else if ( getBtnElapsedTime() == 0 ) {
		if ( HAL_GPIO_ReadPin(activeButtonDescription->ButtonGPIO, activeButtonDescription->ButtonPin) == activeButtonDescription->ActivePinState ) {
			setBtnWaitTime(BUTTON_IGNORE_TIME);
			ignoreKeysFlag = true;							// Обязятельно выставляем флаг игнорирования на заданное время, для исключения нежелательного повтора
			return activeButtonDescription->Button_KeyID;
		}
		else {
			activeButtonDescription = NULL;
		}
	}

	return buttonKEY_ID_NONE;
}
