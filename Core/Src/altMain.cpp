#include "altMain.h"

extern uint32_t greenLed1;
extern uint32_t greenLed2;
SLCAN::CANIface* pSlCan = nullptr;


//uint32_t defaultTaskBuffer1[ stackTest ];
UBaseType_t stackLeft[10] = {0};

template <size_t number>
using Pins = STM32F103RBT6::Pins<number>;

Pins<6>* pLeds = nullptr;

uint32_t greenLed1 = 0;
uint32_t greenLed2 = 0;
uint32_t redLed = 0;
uint32_t whiteLed = 0;
uint32_t blueLed = 0;
uint32_t yellowLed = 0;
uint32_t currentTime = 0;
uint32_t lastTime = 0;
uint32_t msCount = 0;

void ledRedOn(void){
    if (pLeds == nullptr) return;
    pLeds->on(ledRed,OsWrapper::IThread::GetTime(),1);
}
void ledWhiteOn(void){
    if (pLeds == nullptr) return;
    pLeds->on(ledWhite,OsWrapper::IThread::GetTime(),1);
}


int altMain(){
    using namespace OsWrapper;
    UBaseType_t uxHighWaterMark;
    SLCAN::CANIface slCan(ledRedOn,ledWhiteOn);
    pSlCan = &slCan;

    Pins<6> leds;
    leds.add(ledGreen1, GPIOC,PIN1);
    leds.add(ledRed,    GPIOC,PIN3);
    leds.add(ledWhite,  GPIOC,PIN5);
    leds.add(ledGreen2, GPIOB,PIN1);
    leds.add(ledBlue,   GPIOA,PIN1);
    leds.add(ledYellow, GPIOA,PIN3);
    pLeds = &leds;
    leds.off();
    leds.on();
    leds.off();
    TaskCan taskCan(leds, slCan);
    TaskLeds taskLeds(leds);
    
    Rtos::CreateThread(taskLeds,"taskLeds",ThreadPriority::aboveNormal);
    Rtos::CreateThread(taskCan, "taskCan", ThreadPriority::lowest);
    HAL_CAN_Start(&hcan1);
    for (;;)
	{
        leds.on(ledBlue,IThread::GetTime(),500);
        uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        stackLeft[1] = uxHighWaterMark;
		vTaskDelay(1000);
	}
}

int8_t usbReceive(uint8_t* Buf, uint32_t *Len){
    if (pLeds != nullptr){
        pLeds->on(ledGreen1,OsWrapper::IThread::GetTime(),1);
    }
    greenLed1 = 2;
    if(pSlCan != nullptr)
        pSlCan->storeSerialMessage(Buf,Len);
    return 0;
}

void canRxInt(CAN_HandleTypeDef *_hcan, uint8_t fifo){
    if (pLeds != nullptr){
        pLeds->on(ledGreen2,OsWrapper::IThread::GetTime(),1);
    }
    greenLed2 = 2;
    if(pSlCan != nullptr)
        pSlCan->storeCanMessage(fifo, SLCAN::CANIface::native_micros64());
}


int altMainRaw(){
    SLCAN::CANIface slCan(ledRedOn,ledWhiteOn);
    pSlCan = &slCan;
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