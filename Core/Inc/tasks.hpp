#ifndef TASKS_HPP
#define TASKS_HPP

#include "altMain.h"
/* freertos */
#include "thread.hpp"
#include "event.hpp"
#include "mutex.hpp"
#include "mailbox.hpp"
/* c++ */
#include <vector>

/* mine */

#include "slcan.hpp"

#include "rtoswrapper.hpp"
#include "event.hpp"
#include "rtos.hpp"
#include "mutex.hpp"
#include "mailbox.hpp"
#include "deviceSpecific.hpp"

/* class TaskKeys : public OsWrapper::Thread<static_cast<std::size_t>(OsWrapper::StackDepth::medium)>
{
public:
virtual void Execute() override;
  TaskKeys( STM32F103RBT6::Pins<8>& keys,
            OsWrapper::MailBox<uint8_t,1>& mailBoxKeys,
            OsWrapper::MailBox<bool,1>& mailBoxPowerStateKey)
                      : 
                      mKeys(keys),
                      mMailBoxKeys(mailBoxKeys),
                      mMailBoxPowerStateKey(mailBoxPowerStateKey)
  { 
  }
private:
  STM32F103RBT6::Pins<8>& mKeys;
  OsWrapper::MailBox<uint8_t,1>& mMailBoxKeys;
  OsWrapper::MailBox<bool,1>& mMailBoxPowerStateKey;
};


class TaskRtc : public OsWrapper::Thread<static_cast<std::size_t>(OsWrapper::StackDepth::medium)>
{
public:
  virtual void Execute() override;
  TaskRtc(OsWrapper::MailBox<sTimeCheck,1>& mailBoxTime,
          Rtc::Rtc& rtc)
          : mMailBoxTime(mailBoxTime),
            mRtc(rtc)
  {
  }
private:
  OsWrapper::MailBox<sTimeCheck,1>& mMailBoxTime;
  Rtc::Rtc& mRtc;
}; */

class TaskCan : public OsWrapper::Thread<static_cast<std::size_t>(OsWrapper::StackDepth::bigbig)>
{
public:
  virtual void Execute() override;
  TaskCan( STM32F103RBT6::Pins<6>& leds,
           SLCAN::CANIface& slCan )
                      : 
                      mLeds(leds),
                      mSlCan(slCan)
  {
  }
private:
  STM32F103RBT6::Pins<6>& mLeds;
  SLCAN::CANIface& mSlCan;
};


class TaskLeds : public OsWrapper::Thread<static_cast<std::size_t>(OsWrapper::StackDepth::medium)>
{
public:
virtual void Execute() override;
  TaskLeds( STM32F103RBT6::Pins<6>& leds)
                      : 
                      mLeds(leds)
  { 
  }
private:
  STM32F103RBT6::Pins<6>& mLeds;
};

#endif //TASKS_HPP
