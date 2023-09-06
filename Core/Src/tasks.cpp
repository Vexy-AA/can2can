#include "tasks.hpp"
#define __LL_ADC_CALC_VREF(__VOLTAGE__,__ADC_DATA__,__ADC_RESOLUTION__)  \
  (__LL_ADC_DIGITAL_SCALE(__ADC_RESOLUTION__) * (__VOLTAGE__)   / (__ADC_DATA__)                                \
  )
  
extern UBaseType_t stackLeft[10];
float powerPrint = 0;

/* void TaskKeys::Execute()
{
  UBaseType_t uxHighWaterMark;
  uint8_t keysStatus = 0;
  uint32_t offKeyTimer = 0;
  while(true){
    keysStatus = mKeys.getStatus();// | 0xF0;
    keysStatus = ~keysStatus;
    mMailBoxKeys.Put(keysStatus);
    if (!mKeys.getStatus(inStop)){
      if (!offKeyTimer){
        offKeyTimer = GetTime();
      }
      if (GetTime() - offKeyTimer > 20000){
        mMailBoxPowerStateKey.Put(true);
      }
    }else{
      offKeyTimer = 0;
    }
    
    Sleep(10ms);
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    stackLeft[2] = uxHighWaterMark;
  }
}


void TaskRtc::Execute(){
  UBaseType_t uxHighWaterMark;
  sTimeCheck time;
  float gyrofreq = 0;
  while(true){
    WaitForSignal(1000ms,0x10);
//    if (mRtc.isSecondPass()){
    if (LL_RTC_IsActiveFlag_ALRA(RTC))
    	LL_RTC_ClearFlag_ALRA(RTC);
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_17))
    	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_17);
      mMailBoxTime.Get(time,0);
      time.timeCurrent = mRtc.getCurrentTime();
      mMailBoxTime.Put(time);
      //printf_("time %d\r\n",time);
      gyrofreq = test123*0.5f + gyrofreq*0.5f;
      //printf_("gyrofreq = %0.1f\r\n", gyrofreq);
      test123 = 0;
//    }


    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    stackLeft[9] = uxHighWaterMark;
  }
} */

void TaskCan::Execute(){
  UBaseType_t uxHighWaterMark;
  SLCAN::CANFrame frame;
  SLCAN::CanIOFlags flags;
  uint64_t time;
  while(1){
      mSlCan.receive(frame,time=GetTime(),flags);
      uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
      stackLeft[1] = uxHighWaterMark;
  }
}

void TaskLeds::Execute()
{
  UBaseType_t uxHighWaterMark;
  while(true){
    mLeds.clock(GetTime());
    Sleep(1ms);
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    stackLeft[2] = uxHighWaterMark;
  }
}








