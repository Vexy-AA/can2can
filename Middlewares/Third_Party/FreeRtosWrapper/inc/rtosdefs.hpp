/*******************************************************************************
* Filename  	: rtosdefs.h
*
* Details   	: Common definitions for RTOS wrapper 
*
* Author        : Sergey Kolody
*******************************************************************************/

#ifndef RTOSDEFS_H
#define RTOSDEFS_H

//For EmbOs types and methods definition (OS_TASK)
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"
#include "event_groups.h"
#include "portmacro.h"
#define __forceinline  __attribute__((always_inline))
//#define __forceinline  _Pragma("inline=forced")
//#include "susudefs.hpp" // for __forceinline (#define __forceinline  _Pragma("inline=forced"))
#include "FreeRTOSConfig.h"
//#include "std.hpp"
//#include <iostream>
#include <chrono> // for std::chrono::duration and "ms" literal

using namespace std::chrono_literals;

namespace OsWrapper
{
  extern "C" void vPortSVCHandler(void);
  extern "C" void xPortPendSVHandler(void);
  extern "C" void xPortSysTickHandler(void);

  enum class ThreadPriority : std::uint8_t
  {
      clear = 0,
      lowest = 1,
      belowNormal = 2,
      normal = 3,
      aboveNormal = 4,
      highest = 5
      //priorityMax = 255
  } ;

  using tTaskContext = StaticTask_t;
  using tTaskHandle = TaskHandle_t;
  using tStack = StackType_t ;

  using tTaskEventMask = std::uint32_t ;
  using tTime = TickType_t ;

  using tEventHandle = EventGroupHandle_t;
  using tEvent =  StaticEventGroup_t;
  using tEventBits = EventBits_t;

  using tMailBoxContext = StaticQueue_t;
  using tMailBoxHandle = QueueHandle_t;

  using tMutex = StaticSemaphore_t;
  using tMutexHandle = SemaphoreHandle_t;

  using TicksPerSecond = std::chrono::duration<tTime , std::ratio<portTICK_PERIOD_MS,1000>> ;

}

#endif // RTOSDEFS_H
