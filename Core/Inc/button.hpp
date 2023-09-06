#ifndef BUTTON_HPP
#define BUTTON_HPP
#include "pin.hpp"
using namespace PinDriver;

namespace PinDriver{

template<typename gpioType,typename pinType>
class Button : public Pin<gpioType,pinType>
{
	using Button::Pin::mState;
	using Parent = Pin<gpioType,pinType>;
	//using Parent::mState;
    using typename Button::Pin::validateSetPin;
    using typename Button::Pin::validateResetPin;
    using typename Button::Pin::validateReadPin;
private:
    uint32_t mPressedTicks = 0;
    uint32_t mUnpressedTicks = 0;
    //uint8_t mButtonId;
    uint32_t mShortPress;
    uint32_t mLongPress;
    uint32_t mDoublePress;

public:
    Button(	uint8_t id,
            gpioType gpio,
    		pinType pin,
			validateSetPin setpin,
			validateResetPin resetpin,
			validateReadPin readpin,
            bool invert=false,
            uint32_t shortPress = 50,
            uint32_t longPress = 1000,
            uint32_t doublePress = 500);
    PressType wasPressed(uint32_t ticks);
};
/*****************************************************************************
    * Function Name: Constructor
    * Description:
    *
    *************************************************************************/
template<typename gpioType,typename pinType>
Button<gpioType,pinType>::Button(
    uint8_t id,
    gpioType gpio,
    pinType pin,
    validateSetPin setpin,
    validateResetPin resetpin,
    validateReadPin readpin,
    bool invert,  uint32_t shortPress,uint32_t longPress,
    uint32_t doublePress)
    :   Parent(id,gpio,pin,setpin,resetpin,readpin,invert),
        mShortPress(shortPress),
        mLongPress(longPress),
        mDoublePress(doublePress){

}
/*****************************************************************************
    * Function Name: off()
    * Description:
    *
    *************************************************************************/
template<typename gpioType,typename pinType>
PressType Button<gpioType,pinType>::wasPressed(uint32_t ticks){
    //using Pin<gpioType,pinType>::read;
    PressType pressType = IDLE;
	State oldState = mState;
    uint32_t buttonPressedTime = 0;
    if (Parent::read() && oldState == LOW){
        mPressedTicks = ticks;
    }else if (mState == LOW && oldState == HIGH){
        mUnpressedTicks = ticks;
        buttonPressedTime = mUnpressedTicks - mPressedTicks;
        if (buttonPressedTime >= mDoublePress){
            pressType = DOUBLE_PRESS;
        }else if (buttonPressedTime >= mLongPress){
            pressType = LONG_PRESS;
        }else if (buttonPressedTime >= mShortPress){
            pressType = SHORT_PRESS;
        }
    }

    return pressType;
}

}

#endif
