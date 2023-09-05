#include "altMain.h"
#include "slcan.hpp"

SLCAN::CANIface slCan;
uint32_t greenLed1 = 0;
uint32_t greenLed2 = 0;
uint32_t redLed = 0;
uint32_t whiteLed = 0;
uint32_t blueLed = 0;
uint32_t yellowLed = 0;
uint32_t currentTime = 0;
uint32_t lastTime = 0;
uint32_t msCount = 0;
int altMain(){

    HAL_CAN_Start(&hcan1);
    SLCAN::CANFrame frame;
    SLCAN::CanIOFlags flags;
    uint64_t time;
    lastTime = HAL_GetTick();
    currentTime = HAL_GetTick();
    while(1){
        currentTime = HAL_GetTick();
        if (currentTime > lastTime){
            lastTime = currentTime;
            msCount++;
            
            
            
            
            
            
            if (greenLed1){
                    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_1);
                    greenLed1--;
            }else LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_1);
            if (redLed){
                    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_3);
                    redLed--;
            }else LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_3);
            if (blueLed){
                    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1);
                    blueLed--;
            }else LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);
            if (yellowLed){
                    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3);
                    yellowLed--;
            }else LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3);
            if (whiteLed){
                    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_5);
                    whiteLed--;
            }else LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_5);
            if (greenLed2){
                    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);
                    greenLed2--;
            }else LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);
        }
        if(msCount %1000 == 0){
            yellowLed = 500;
        }
        slCan.receive(frame,time,flags);
    }
}

int8_t usbReceive(uint8_t* Buf, uint32_t *Len){
    //HAL_Delay(10);
    greenLed1 = 2;
    slCan.storeSerialMessage(Buf,Len);
    return 0;
}

void canRxInt(CAN_HandleTypeDef *_hcan, uint8_t fifo){
    greenLed2 = 2;
    slCan.storeCanMessage(fifo, SLCAN::CANIface::native_micros64());
}

