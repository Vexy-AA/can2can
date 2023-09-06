#ifndef ALT_MAIN_H_
#define ALT_MAIN_H_

#include "can.h"
#include "stdint.h"
#include "slcan.hpp"
#include "tasks.hpp"
#include "main.h"


#include "rtoswrapper.hpp"
#include "event.hpp"
#include "rtos.hpp"
#include "mutex.hpp"
#include "mailbox.hpp"
#include "deviceSpecific.hpp"

#define PIN0 LL_GPIO_PIN_0
#define PIN1 LL_GPIO_PIN_1
#define PIN2 LL_GPIO_PIN_2
#define PIN3 LL_GPIO_PIN_3
#define PIN4 LL_GPIO_PIN_4
#define PIN5 LL_GPIO_PIN_5
#define PIN6 LL_GPIO_PIN_6
#define PIN7 LL_GPIO_PIN_7
#define PIN8 LL_GPIO_PIN_8
#define PIN9 LL_GPIO_PIN_9
#define PIN10 LL_GPIO_PIN_10
#define PIN11 LL_GPIO_PIN_11
#define PIN12 LL_GPIO_PIN_12
#define PIN13 LL_GPIO_PIN_13
#define PIN14 LL_GPIO_PIN_14
#define PIN15 LL_GPIO_PIN_15


#ifdef __cplusplus
extern "C"
{
#endif

int altMain();
int altMainRaw();
int8_t usbReceive(uint8_t* Buf, uint32_t *Len);
void canRxInt(CAN_HandleTypeDef *_hcan, uint8_t fifo);


#ifdef __cplusplus
}
#endif

#endif
