#ifndef DEVICESPECIFIC_HPP
#define DEVICESPECIFIC_HPP

#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_tim.h"

#include "inline.hpp"
#include "pin.hpp"
#include "button.hpp"
#include "event.hpp"
#include <vector>

#define isUserKeyPressed !LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_13)
#define onLD2 LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5)
#define offLD2 LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5)
#define toggleLD2 LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_5)
#define motorCoil1Enable  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_11)
#define motorCoil1Disable  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_11)
#define motorCoil2Enable  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12)
#define motorCoil2Disable  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12)

#define onTestPin LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_9)
#define offTestPin LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9)

#define getTim2 LL_TIM_GetCounter(TIM2)
#define getTim3 LL_TIM_GetCounter(TIM4)

#define digitsUpdate LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_7);\
                    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_7);\
                    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_7);\
                     LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_7)

typedef enum
{
    inStart,
    inDown,
    inNext,
    inUp,
    inLeft,
    inRight,
    inPrevious,
    inStop,
    /*inUp,
    inRight,
    inDown,
    inLeft,
    inStop,
    inStart,
    inNext,
    inPrevious,*/
    inCurrrentAlert,
    outPiPwr,
    outPiPwr2,
    ledRed,
    ledGreen1,
    ledBlue,
    ledYellow,
    ledWhite,
    ledGreen2
} tPinNames;

namespace STM32F103RBT6{






class Pin : public PinDriver::Pin<GPIO_TypeDef *, uint32_t>
{
using Parent = PinDriver::Pin<GPIO_TypeDef *, uint32_t>; 
public:
    Pin(tPinNames id,
        GPIO_TypeDef * gpio,
        uint32_t pin,
        bool invert=false) :
            Parent(
                static_cast<uint8_t>(id),
                gpio,
                pin,
                LL_GPIO_SetOutputPin,
                LL_GPIO_ResetOutputPin,
                LL_GPIO_IsOutputPinSet,
                invert)
    {
    }
    Pin(tPinNames id,
        GPIO_TypeDef * gpio,
        uint32_t pin,
		uint16_t pulse,
        bool invert = false
        ) :
            Parent(
                static_cast<uint8_t>(id),
                gpio,
                pin,
                LL_GPIO_SetOutputPin,
                LL_GPIO_ResetOutputPin,
                LL_GPIO_IsOutputPinSet,
                invert)
    {
    }
    Pin(tPinNames id,
        GPIO_TypeDef * gpio,
        uint32_t pin,
        uint32_t readPin (GPIO_TypeDef *, uint32_t),
        bool invert=false) :
            Parent(
                static_cast<uint8_t>(id),
                gpio,
                pin,
                LL_GPIO_SetOutputPin,
                LL_GPIO_ResetOutputPin,
                readPin,
                invert)
    {
    }
};




template <int number>
class Pins
{
private:

    uint8_t mPinsNumber = number;

public:
    std::vector<STM32F103RBT6::Pin> mPins;
    //bool status[number];
    uint8_t status = 0;
    Pins()
    {
        mPins.reserve(number);
    }
    void add(tPinNames id, GPIO_TypeDef* gpio, uint32_t pin){
        mPins.push_back(Pin(id,gpio,pin));
    }
    void add(tPinNames id, GPIO_TypeDef* gpio, uint32_t pin,bool invert){
        mPins.push_back(Pin(id,gpio,pin,invert));
    }
    void addInput(tPinNames id, GPIO_TypeDef* gpio, uint32_t pin){
        mPins.push_back(Pin(id,gpio,pin,LL_GPIO_IsInputPinSet));
    }
    void addInput(tPinNames id, GPIO_TypeDef* gpio, uint32_t pin,bool invert){
        mPins.push_back(Pin(id,gpio,pin,LL_GPIO_IsInputPinSet,invert));
    }
    
    uint8_t getStatus(){
        for (uint8_t i= 0; i < mPins.size(); i++){
            if (mPins.at(i).read())
                status |= 1 << mPins.at(i).getId();
            else
                status &= ~(1 << mPins.at(i).getId());
        }
        return status;
    }
    bool getStatus(tPinNames id){
        for (Pin& pin : mPins){
            if (pin.getId() == static_cast<uint8_t>(id)){
                return pin.read();
            }
        }
        return false;
    }
    void on(){
        for (Pin& pin : mPins){
            pin.on();
        }
    }
    bool on(tPinNames id){
        for (Pin& pin : mPins){
            if (pin.getId() == static_cast<uint8_t>(id)){
                pin.on();
                return true;
            }
        }
        return false;
    }
    void off(){
        for (Pin& pin : mPins){
            pin.off();
        }
    }
    bool off(tPinNames id){
        for (Pin& pin : mPins){
            if (pin.getId() == static_cast<uint8_t>(id)){
                pin.off();
                return true;
            }
        }
        return false;
    }
    bool on(tPinNames id, uint32_t currentTime){
        for (Pin& pin : mPins){
            if (pin.getId() == static_cast<uint8_t>(id)){
                pin.on(currentTime);
                return true;
            }
        }
        return false;
    }
    bool on(tPinNames id, uint32_t currentTime, uint16_t pulseTime){
        for (Pin& pin : mPins){
            if (pin.getId() == static_cast<uint8_t>(id)){
                pin.on(pulseTime, currentTime);
                return true;
            }
        }
        return false;
    }
    void clock(uint32_t currentTime){
        for (Pin& pin : mPins){
            pin.clock(currentTime);
        }
    }
};










}




#endif
