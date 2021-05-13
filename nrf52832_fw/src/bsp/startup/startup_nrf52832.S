
  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb

.global  g_pfnVectors
.global  Default_Handler

/* start address for the initialization values of the .data section. 
defined in linker script */
.word  _sidata
/* start address for the .data section. defined in linker script */  
.word  _sdata
/* end address for the .data section. defined in linker script */
.word  _edata
/* start address for the .bss section. defined in linker script */
.word  _sbss
/* end address for the .bss section. defined in linker script */
.word  _ebss
/* stack used for SystemInit_ExtMemCtl; always internal RAM used */

/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called. 
 * @param  None
 * @retval : None
*/

    .section  .text.Reset_Handler
  .weak  Reset_Handler
  .type  Reset_Handler, %function
Reset_Handler:  
  ldr   sp, =_estack    		 /* set stack pointer */

/* Copy the data segment initializers from flash to SRAM */  
  movs  r1, #0
  b  LoopCopyDataInit

CopyDataInit:
  ldr  r3, =_sidata
  ldr  r3, [r3, r1]
  str  r3, [r0, r1]
  adds  r1, r1, #4
    
LoopCopyDataInit:
  ldr  r0, =_sdata
  ldr  r3, =_edata
  adds  r2, r0, r1
  cmp  r2, r3
  bcc  CopyDataInit
  ldr  r2, =_sbss
  b  LoopFillZerobss
/* Zero fill the bss segment. */  
FillZerobss:
  movs  r3, #0
  str  r3, [r2], #4
    
LoopFillZerobss:
  ldr  r3, = _ebss
  cmp  r2, r3
  bcc  FillZerobss

/* Call the clock system intitialization function.*/
  bl  SystemInit   
/* Call static constructors */
    bl __libc_init_array
/* Call the application's entry point.*/
  bl  main
  bx  lr    
.size  Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an 
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 * @param  None     
 * @retval None       
*/
    .section  .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b  Infinite_Loop
  .size  Default_Handler, .-Default_Handler
/******************************************************************************
*
* The minimal vector table for a Cortex M3. Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
* 
*******************************************************************************/
   .section  .isr_vector,"a",%progbits
  .type  g_pfnVectors, %object
  .size  g_pfnVectors, .-g_pfnVectors
    
g_pfnVectors:
  .word  _estack
  .word  Reset_Handler
  .word  NMI_Handler
  .word  HardFault_Handler
  .word  MemManage_Handler
  .word  BusFault_Handler
  .word  UsageFault_Handler
  .word  0
  .word  0
  .word  0
  .word  0
  .word  SVC_Handler
  .word  DebugMon_Handler
  .word  0
  .word  PendSV_Handler
  .word  SysTick_Handler
  
  /* External Interrupts */
    .long   POWER_CLOCK_IRQHandler
    .long   RADIO_IRQHandler
    .long   UARTE0_UART0_IRQHandler
    .long   SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler
    .long   SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler
    .long   NFCT_IRQHandler
    .long   GPIOTE_IRQHandler
    .long   SAADC_IRQHandler
    .long   TIMER0_IRQHandler
    .long   TIMER1_IRQHandler
    .long   TIMER2_IRQHandler
    .long   RTC0_IRQHandler
    .long   TEMP_IRQHandler
    .long   RNG_IRQHandler
    .long   ECB_IRQHandler
    .long   CCM_AAR_IRQHandler
    .long   WDT_IRQHandler
    .long   RTC1_IRQHandler
    .long   QDEC_IRQHandler
    .long   COMP_LPCOMP_IRQHandler
    .long   SWI0_EGU0_IRQHandler
    .long   SWI1_EGU1_IRQHandler
    .long   SWI2_EGU2_IRQHandler
    .long   SWI3_EGU3_IRQHandler
    .long   SWI4_EGU4_IRQHandler
    .long   SWI5_EGU5_IRQHandler
    .long   TIMER3_IRQHandler
    .long   TIMER4_IRQHandler
    .long   PWM0_IRQHandler
    .long   PDM_IRQHandler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   MWU_IRQHandler
    .long   PWM1_IRQHandler
    .long   PWM2_IRQHandler
    .long   SPIM2_SPIS2_SPI2_IRQHandler
    .long   RTC2_IRQHandler
    .long   I2S_IRQHandler
    .long   FPU_IRQHandler
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
    .long   0                           /*Reserved */
                    
/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler. 
* As they are weak aliases, any function with the same name will override 
* this definition.
* 
*******************************************************************************/
   .weak      NMI_Handler
   .thumb_set NMI_Handler,Default_Handler
  
   .weak      HardFault_Handler
   .thumb_set HardFault_Handler,Default_Handler
  
   .weak      MemManage_Handler
   .thumb_set MemManage_Handler,Default_Handler
  
   .weak      BusFault_Handler
   .thumb_set BusFault_Handler,Default_Handler

   .weak      UsageFault_Handler
   .thumb_set UsageFault_Handler,Default_Handler

   .weak      SVC_Handler
   .thumb_set SVC_Handler,Default_Handler

   .weak      DebugMon_Handler
   .thumb_set DebugMon_Handler,Default_Handler

   .weak      PendSV_Handler
   .thumb_set PendSV_Handler,Default_Handler

   .weak      SysTick_Handler
   .thumb_set SysTick_Handler,Default_Handler              
  

   .weak      POWER_CLOCK_IRQHandler
   .thumb_set POWER_CLOCK_IRQHandler,Default_Handler
                  
   .weak      RADIO_IRQHandler
   .thumb_set RADIO_IRQHandler,Default_Handler
               
   .weak      UARTE0_UART0_IRQHandler
   .thumb_set UARTE0_UART0_IRQHandler,Default_Handler
            
   .weak      SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler
   .thumb_set SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler,Default_Handler
            
   .weak      SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler
   .thumb_set SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler,Default_Handler
                  
   .weak      NFCT_IRQHandler
   .thumb_set NFCT_IRQHandler,Default_Handler
                  
   .weak      GPIOTE_IRQHandler
   .thumb_set GPIOTE_IRQHandler,Default_Handler
                  
   .weak      SAADC_IRQHandler
   .thumb_set SAADC_IRQHandler,Default_Handler
                     
   .weak      TIMER0_IRQHandler
   .thumb_set TIMER0_IRQHandler,Default_Handler
                 
   .weak      TIMER1_IRQHandler
   .thumb_set TIMER1_IRQHandler,Default_Handler
                        
   .weak      TIMER2_IRQHandler
   .thumb_set TIMER2_IRQHandler,Default_Handler
                  
   .weak      RTC0_IRQHandler
   .thumb_set RTC0_IRQHandler,Default_Handler
         
   .weak      TEMP_IRQHandler
   .thumb_set TEMP_IRQHandler,Default_Handler
                  
   .weak      RNG_IRQHandler
   .thumb_set RNG_IRQHandler,Default_Handler
                  
   .weak      ECB_IRQHandler
   .thumb_set ECB_IRQHandler,Default_Handler
                 
   .weak      CCM_AAR_IRQHandler
   .thumb_set CCM_AAR_IRQHandler,Default_Handler
                  
   .weak      WDT_IRQHandler
   .thumb_set WDT_IRQHandler,Default_Handler
                  
   .weak      RTC1_IRQHandler
   .thumb_set RTC1_IRQHandler,Default_Handler
                  
   .weak      QDEC_IRQHandler
   .thumb_set QDEC_IRQHandler,Default_Handler
            
   .weak      COMP_LPCOMP_IRQHandler
   .thumb_set COMP_LPCOMP_IRQHandler,Default_Handler
            
   .weak      SWI0_EGU0_IRQHandler
   .thumb_set SWI0_EGU0_IRQHandler,Default_Handler
            
   .weak      SWI1_EGU1_IRQHandler
   .thumb_set SWI1_EGU1_IRQHandler,Default_Handler
      
   .weak      SWI2_EGU2_IRQHandler
   .thumb_set SWI2_EGU2_IRQHandler,Default_Handler
      
   .weak      SWI3_EGU3_IRQHandler
   .thumb_set SWI3_EGU3_IRQHandler,Default_Handler
                  
   .weak      SWI4_EGU4_IRQHandler
   .thumb_set SWI4_EGU4_IRQHandler,Default_Handler
                  
   .weak      TIM3_IRQHandler            
   .thumb_set TIM3_IRQHandler,Default_Handler
                  
   .weak      TIM4_IRQHandler            
   .thumb_set TIM4_IRQHandler,Default_Handler
                  
   .weak      SWI5_EGU5_IRQHandler
   .thumb_set SWI5_EGU5_IRQHandler,Default_Handler
                     
   .weak      TIMER3_IRQHandler
   .thumb_set TIMER3_IRQHandler,Default_Handler
                     
   .weak      TIMER4_IRQHandler
   .thumb_set TIMER4_IRQHandler,Default_Handler
                  
   .weak      PWM0_IRQHandler
   .thumb_set PWM0_IRQHandler,Default_Handler
                           
   .weak      PDM_IRQHandler
   .thumb_set PDM_IRQHandler,Default_Handler
                        
   .weak      MWU_IRQHandler
   .thumb_set MWU_IRQHandler,Default_Handler
                  
   .weak      PWM1_IRQHandler
   .thumb_set PWM1_IRQHandler,Default_Handler
                     
   .weak      PWM2_IRQHandler
   .thumb_set PWM2_IRQHandler,Default_Handler
                                  
   .weak      SPIM2_SPIS2_SPI2_IRQHandler
   .thumb_set SPIM2_SPIS2_SPI2_IRQHandler,Default_Handler
               
   .weak      RTC2_IRQHandler
   .thumb_set RTC2_IRQHandler,Default_Handler
            
   .weak      I2S_IRQHandler
   .thumb_set I2S_IRQHandler,Default_Handler
            
   .weak      FPU_IRQHandler
   .thumb_set FPU_IRQHandler,Default_Handler
                     

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

