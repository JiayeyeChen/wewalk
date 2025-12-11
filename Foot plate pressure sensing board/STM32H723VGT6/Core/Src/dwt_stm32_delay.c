/** @file dwt_stm32_delay.c
 *  @brief This function allows the program to delay x ms.
 * 
 *  The function allows the program to delay x ms.
 *  It is required to call the DWT_Delay_Init() first in the main.c
 *  Call DWT_Delay_us(ms) to delay the program.
 * 
 *  @bug No known bugs
 */

#include "dwt_stm32_delay.h"

uint32_t DWT_Delay_Init(void) {
  /* Disable TRC */
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; 	// ~0x01000000;
  /* Enable TRC */
  CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; 	// 0x01000000;

  /* Disable clock cycle counter */
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; 						//~0x00000001;
  /* Enable  clock cycle counter */
  DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; 						//0x00000001;

  /* Reset the clock cycle counter value */
  DWT->CYCCNT = 0;

     /* 3 NO OPERATION instructions */
     __ASM volatile ("NOP");
     __ASM volatile ("NOP");
  __ASM volatile ("NOP");

  /* Check if clock cycle counter has started */
     if(DWT->CYCCNT)
     {
       return 0; /*clock cycle counter started*/
     }
     else
  {
    return 1; /*clock cycle counter not started*/
  }
}

/* Use DWT_Delay_Init (); and DWT_Delay_us (microseconds) in the main */
