#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"


//*****************************************************************************
// Definition for Trigger Duration in CPU cycles
//*****************************************************************************
#define TRIGGER_DURATION 500000

//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line) {}
#endif

//*****************************************************************************
// Setting up the system clock
//*****************************************************************************
void ConfigureSystemClock() {
    // Setup the system clock to run at 50 MHz from PLL with crystal reference
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
}

//*****************************************************************************
// Restart TIMER
//*****************************************************************************
void StartTimer(void) {
    MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, TRIGGER_DURATION);
    MAP_TimerEnable(TIMER0_BASE, TIMER_A);
}

//*****************************************************************************
// The interrupt handler for PortE1
//*****************************************************************************
void PortEIntHandler(void) {
    // Clear Interrupt flag
    MAP_GPIOIntClear(GPIO_PORTE_BASE, GPIO_INT_PIN_1);

    if (MAP_GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_2) == 0x04) {
        // Write value of "1" to all the pins E3, E4, E5
        MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, 60);
        StartTimer();
    }
}

//*****************************************************************************
// Setting up the IO pins on port E
//*****************************************************************************
void ConfigureIO_PortE() {
    // Enable and wait for the port to be ready for access
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) {}

    // Configure the pins E1, E2 to be the input clocks
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2);

    // Configure the pins E3, E4, E5 to be the fan-out signals
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

    // Enable pin PE1 to work as the 10Hz signal
    // When this PE1 goes from low to high, an interrupt will be issued
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1);
    MAP_GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_RISING_EDGE);
    MAP_GPIOIntRegister(GPIO_PORTE_BASE, &PortEIntHandler);
    MAP_GPIOIntEnable(GPIO_PORTE_BASE, GPIO_INT_PIN_1);
}

//*************************************************************************************************************************************
//  Configure Periodic Timer
//*************************************************************************************************************************************
void ConfigureTimer(void) {
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // Configure the periodic timer.
    MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    // Set ADG732 SELECT switch to 250kHz (TIMER1)
    MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, TRIGGER_DURATION);

    // Enable processor interrupts.
    MAP_IntMasterEnable();

    // Enable TIMER0 interrupt
    MAP_IntEnable(INT_TIMER0A);
    MAP_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

//*****************************************************************************
// The interrupt handler for the first timer interrupt.
//*****************************************************************************
void Timer0IntHandler(void) {
    // Clear the timer interrupt.
    MAP_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Set the PE3, PE4, PE5 back to low
    MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, 0);
}

//*****************************************************************************
// Main 'C' Language entry point.
//*****************************************************************************
int main(void) {

    ConfigureSystemClock();
    ConfigureTimer();
    ConfigureIO_PortE();

    while (1) {}
}
