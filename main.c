/***************************************************************************//**
 * @file
 * @brief main() function.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "sl_component_catalog.h"
#include "sl_system_init.h"
#include "app.h"
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)
#include "sl_power_manager.h"
#endif // SL_CATALOG_POWER_MANAGER_PRESENT
#if defined(SL_CATALOG_KERNEL_PRESENT)
#include "sl_system_kernel.h"
#else // SL_CATALOG_KERNEL_PRESENT
#include "sl_system_process_action.h"
#endif // SL_CATALOG_KERNEL_PRESENT
#include "em_cmu.h"
#include "em_gpio.h"
#include "memlcd_app.h"
#include "em_usart.h"
#define USER_TX_LOCATION 0
#define USER_RX_LOCATION 0
#define BSP_GPIO_LEDS
#define BSP_NO_OF_LEDS 2
#define BSP_GPIO_LED0_PORT gpioPortF
#define BSP_GPIO_LED0_PIN 4
#define BSP_GPIO_LED1_PORT gpioPortF
#define BSP_GPIO_LED1_PIN 5
#define BSP_GPIO_LEDARRAY_INIT { { BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN}, { BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN } }
#define BSP_GPIO_BUTTONS
#define BSP_NO_OF_BUTTONS 2
#define BSP_GPIO_PB0_PORT gpioPortF
#define BSP_GPIO_PB0_PIN 6
#define BSP_GPIO_PB1_PORT gpioPortF
#define BSP_GPIO_PB1_PIN 7

char a = 0, b = 0;

void init_GPIO(){
// Enable GPIO clock
CMU_ClockEnable(cmuClock_GPIO, true);
// Configure PB0 and PB1 as input with glitch filter enabled
GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN,
gpioModeInputPullFilter, 1);
GPIO_PinModeSet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN,
gpioModeInputPullFilter, 1);
// Configure LED0 and LED1 as output
GPIO_PinModeSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN, gpioModePushPull,
0);
GPIO_PinModeSet(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN, gpioModePushPull,
0);
// Enable IRQ for even numbered GPIO pins
NVIC_EnableIRQ(GPIO_EVEN_IRQn);
// Enable IRQ for odd numbered GPIO pins
NVIC_EnableIRQ(GPIO_ODD_IRQn);
// Enable falling-edge interrupts for PB pins
GPIO_ExtIntConfig(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN,BSP_GPIO_PB0_PIN,
0, 1, true);
GPIO_ExtIntConfig(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, BSP_GPIO_PB1_PIN,
0, 1, true);
}

void GPIO_EVEN_IRQHandler(void)
{
// Clear all even pin interrupt flags
GPIO_IntClear(0x5555);
// Toggle LED0
GPIO_PinOutToggle(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
}
void GPIO_ODD_IRQHandler(void)
{
// Clear all odd pin interrupt flags
GPIO_IntClear(0xAAAA);
// Toggle LED01
GPIO_PinOutToggle(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);
}

void USART0_RX_IRQHandler(void) {
if (USART0->IF & USART_IF_RXDATAV) {
    //code here

    memlcd_app_display("              ",1);
           memlcd_app_display("              ",2);
           memlcd_app_display("             ",4);
           memlcd_app_display("             ",5);
    if (USART_Rx(USART0) == '1')
      {

        memlcd_app_display("LAB1",1);
        memlcd_app_display("GIAO TIEP UART",2);
        memlcd_app_display("NGUYEN VAN A",4);
        memlcd_app_display("123928312",5);
      }
    else if (USART_Rx(USART0) == '2')
      {

        memlcd_app_display("THONG TIN LED",1);
        memlcd_app_display("LED 0:",4);
        memlcd_app_display("LED 1:",5);
      }
}
}

int main(void)
{
  // Initialize Silicon Labs device, system, service(s) and protocol stack(s).
  // Note that if the kernel is present, processing task(s) will be created by
  // this call.
  sl_system_init();

  // Initialize the application. For example, create periodic timer(s) or
  // task(s) if the kernel is present.
  app_init();
  memlcd_app_init();
//  initVcomEnable();
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_USART0, true);
  USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;
  USART_InitAsync(USART0,&init);
  USART0->ROUTEPEN |= USART_ROUTEPEN_RXPEN | USART_ROUTEPEN_TXPEN;
  USART0->ROUTELOC0 = (USART0->ROUTELOC0 &~(_USART_ROUTELOC0_TXLOC_MASK| _USART_ROUTELOC0_RXLOC_MASK ))| (USER_TX_LOCATION <<_USART_ROUTELOC0_TXLOC_SHIFT)| (USER_RX_LOCATION <<_USART_ROUTELOC0_RXLOC_SHIFT);

  GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART0_TX_PORT(USER_TX_LOCATION), AF_USART0_TX_PIN(USER_TX_LOCATION), gpioModePushPull, 1);
  GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART0_RX_PORT(USER_RX_LOCATION), AF_USART0_RX_PIN(USER_RX_LOCATION), gpioModeInput, 0);
  USART_IntClear(USART0, USART_IF_RXDATAV);
  USART_IntEnable(USART0, USART_IF_RXDATAV);
  NVIC_ClearPendingIRQ(USART0_RX_IRQn);
  NVIC_EnableIRQ(USART0_RX_IRQn);
  USART_Enable(USART0,usartEnable);
#if defined(SL_CATALOG_KERNEL_PRESENT)
  // Start the kernel. Task(s) created in app_init() will start running.
  sl_system_kernel_start();
#else // SL_CATALOG_KERNEL_PRESENT
  init_GPIO();
  while (1) {
  }
#endif // SL_CATALOG_KERNEL_PRESENT
}

