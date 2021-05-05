#include "system_tm4c1294.h" // CMSIS-Core
#include "driverleds.h"      // device drivers
#include "driverbuttons.h"
#include "cmsis_os2.h"       // CMSIS-RTOS
#include "stdbool.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"
#include "system_TM4C1294.h"

#define SW1_PRESSED 1
#define SW2_PRESSED 2
#define N_LEDS 4

void UART0_Init(void);
void UARTSend(uint8_t *pui8Buffer, uint32_t ui32Count);

/*Structs*/
typedef struct{
  uint8_t current_led;
  uint8_t increment_pwm;
}param_leds;

void configButton();

/*Threads*/
osThreadId_t managerThread;

/*Queues*/
osMessageQueueId_t queue_led1;

/*********************************MANAGER THREAD*********************************/
void manager_thread(void *arg){

  uint8_t teste[] = "Teste envio serial\n";
  
  while(1) {
    UARTwrite(teste, sizeof(teste) + 1);
    osDelay(500);
  }
}

/*************************************MAIN*************************************/
void main(void)
{
  SystemInit();
  LEDInit(LED4 | LED3 | LED2 | LED1);
  configButton();
  UART0_Init();
  
  osKernelInitialize();
    
  queue_led1 = osMessageQueueNew(N_LEDS , N_LEDS, NULL);

  managerThread = osThreadNew(manager_thread, NULL, NULL);

  if (osKernelGetState() == osKernelReady)
    osKernelStart();

  while (1)
    ;
} // main

/*Inicialização dos botões*/
void configButton()
{
  ButtonInit(USW1 | USW2);
  ButtonIntEnable(USW1 | USW2);
}

/*Variaveis de Deboucing*/
const int debouncing_time = 250;
int last_Tick = 0;

/*Handler de Interrupcao*/
void button_ISR(void)
{
  int sw1 = 0, sw2 = 0;
  
  int getTick = osKernelGetTickCount();

  if(GPIOIntStatus(GPIO_PORTJ_BASE, true) & GPIO_PIN_0){
       sw1 = 1;
       GPIOIntClear(GPIO_PORTJ_BASE, GPIO_INT_PIN_0);
  }
  if(GPIOIntStatus(GPIO_PORTJ_BASE, true) & GPIO_PIN_1){
       sw2 = 1;
       GPIOIntClear(GPIO_PORTJ_BASE, GPIO_INT_PIN_1);
  }
  
  if (getTick - last_Tick >= debouncing_time)
  {
    last_Tick = getTick;
  }
}

void UART0_Init(void)
{
  // Enable UART0
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));

  // Initialize the UART for console I/O.
  UARTStdioConfig(0, 9600, SystemCoreClock);

  // Enable the GPIO Peripheral used by the UART.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

  // Configure GPIO Pins for UART mode.
  GPIOPinConfigure(0x1);
  GPIOPinConfigure(0x401);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  
//  UARTIntEnable(UART0_BASE , UART_INT_RX);
//  IntEnable(21);
}

int teste = 1;
void UART0_Handler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = UARTIntStatus(UART0_BASE, true);
    
    UARTIntClear(UART0_BASE, ui32Status);
    
    if(ui32Status & UART_INT_RX) {
      teste = 9;
    }
    //
    // Clear the asserted interrupts.
    //
}
