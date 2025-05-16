#include "wdog.h"

void wdogInterrupt_init(void)
{
//  wdog_config_t wdogConfig;
//  WDOG_GetDefaultConfig(&wdogConfig);
//  wdogConfig.timeoutValue = TIMEOUT_VALUE; /* Timeout value is 8 sec. */
//  wdogConfig.enableInterrupt = true;
//  wdogConfig.interruptTimeValue = INT_BEFORETIMREOUT; /* Interrupt occurred 2 sec before WDOG timeout. */
//  WDOG_Init(MY_WDOG_BASE, &wdogConfig);
//  NVIC_SetPriority(WDOG1_IRQn,0);
  gpio_init(C18, GPO, 0, GPIO_PIN_CONFIG);
  gpio_set(C18, 0);
  gpio_interrupt_init(C18,HIGH,GPIO_INT_CONFIG);
}

