#include <stdint.h>
#include <stdlib.h>
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
#include "driverlib/uart.h"

#define COMMAND_LENGTH 20

//*****************************************************************************
// Global variables
//*****************************************************************************
// Delay values in ms
volatile uint32_t g_ui32DelayShutter;
volatile uint32_t g_ui32DelayGasJet;
volatile uint32_t g_ui32DelayShotCameras;
// UART command array
volatile uint32_t g_ui32UARTCommand[COMMAND_LENGTH];
volatile uint32_t g_ui32UARTCommandIndex;

//*****************************************************************************
// Reset UART Commands
//*****************************************************************************
void ResetUARTCommand() {
    uint8_t i = 0;
    for (i = 0; i < COMMAND_LENGTH; i++) g_ui32UARTCommand[i] = 0;
    g_ui32UARTCommandIndex = 0;
}

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
    // Setup the system clock to run at 80 MHz from PLL with crystal reference
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
}

//*****************************************************************************
// Restart TIMER
//*****************************************************************************
void StartTimer(void) {
    // Enable the timers.
    MAP_TimerEnable(TIMER0_BASE, TIMER_A);
    MAP_TimerEnable(TIMER1_BASE, TIMER_A);
    MAP_TimerEnable(TIMER2_BASE, TIMER_A);
}

//*****************************************************************************
// The interrupt handler for PortE1
//*****************************************************************************
void PortEIntHandler(void) {
    // Clear Interrupt flag
    MAP_GPIOIntClear(GPIO_PORTE_BASE, GPIO_INT_PIN_1);

    if (MAP_GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_2) == 0x4) {
        StartTimer();
    }

    MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, 0x0);
}

//*****************************************************************************
// Setting up the IO pins on port E
//*****************************************************************************
void ConfigureIO_PortE() {
    // Enable and wait for the port to be ready for access
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) {}

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

//*****************************************************************************
// Setting up the IO pins on port F
//*****************************************************************************
void ConfigureIO_PortF() {
    // Enable the GPIO port that is used for the on-board LED.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Enable the GPIO pins for the LED (PF2).
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
}

//*************************************************************************************************************************************
//  Configure Periodic Timer
//*************************************************************************************************************************************
void ConfigureTimer(void) {
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

    // Enable processor interrupts.
    MAP_IntMasterEnable();

    // Configure the two 32-bit periodic timers.
    MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT);
    MAP_TimerConfigure(TIMER1_BASE, TIMER_CFG_ONE_SHOT);
    MAP_TimerConfigure(TIMER2_BASE, TIMER_CFG_ONE_SHOT);
    MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, MAP_SysCtlClockGet() / 1000 * g_ui32DelayShutter - 228);
    MAP_TimerLoadSet(TIMER1_BASE, TIMER_A, MAP_SysCtlClockGet() / 1000 * g_ui32DelayGasJet - 263);
    MAP_TimerLoadSet(TIMER2_BASE, TIMER_A, MAP_SysCtlClockGet() / 1000 * g_ui32DelayShotCameras - 291);

    // Setup the interrupts for the timer timeouts.
    MAP_IntEnable(INT_TIMER0A);
    MAP_IntEnable(INT_TIMER1A);
    MAP_IntEnable(INT_TIMER2A);
    MAP_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    MAP_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    MAP_TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
}

//*****************************************************************************
// The interrupt handler for TIMER0
//*****************************************************************************
void Timer0IntHandler(void) {
    // Clear the timer interrupt.
    MAP_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Set the PE3, PE4, PE5 back to low
    MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, 0x8);

}

//*****************************************************************************
// The interrupt handler for TIMER1
//*****************************************************************************
void Timer1IntHandler(void) {
    // Clear the timer interrupt.
    MAP_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    // Set the PE3, PE4, PE5 back to low
    MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0x10);
}

//*****************************************************************************
// The interrupt handler for TIMER2
//*****************************************************************************
void Timer2IntHandler(void) {
    // Clear the timer interrupt.
    MAP_TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    // Set the PE3, PE4, PE5 back to low
    MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0x20);
}

void I10ToA(uint32_t value, char* result) {

    int base = 10;
    char* ptr = result, *ptr1 = result, tmp_char;
    int tmp_value;

    do {
        tmp_value = value;
        value = value / base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
    } while (value);

    // Apply negative sign
    if (tmp_value < 0) *ptr++ = '-';
    *ptr-- = '\0';
    while (ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
}

//*****************************************************************************
// Send a string to the UART.
//*****************************************************************************
void UARTSend(const char *pui8Buffer, uint32_t ui32Count) {
    // Loop while there are more characters to send.
    while (ui32Count--) {
        // Write the next character to the UART.
        MAP_UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
    }
}

//*****************************************************************************
// Handle UART commands
//*****************************************************************************
void HandleCommand() {
    MAP_UARTCharPutNonBlocking(UART0_BASE, 10);
    MAP_UARTCharPutNonBlocking(UART0_BASE, 13);


    char returnValue[12] = "          \012\015";
    char inputValue[5] = "     ";
    if (g_ui32UARTCommand[0] == 'R') {
        if (g_ui32UARTCommand[1] == '1') {
            I10ToA(g_ui32DelayShutter, returnValue);
        } else if (g_ui32UARTCommand[1] == '2') {
            I10ToA(g_ui32DelayGasJet, returnValue);
        } else if (g_ui32UARTCommand[1] == '3') {
            I10ToA(g_ui32DelayShotCameras, returnValue);
        } else {}
        UARTSend(returnValue, 12);
    } else if (g_ui32UARTCommand[0] == 'W') {
        uint8_t i = 0;
        for (i = 0; i < 5; i++) inputValue[i] = (char) g_ui32UARTCommand[i + 3];
        if (g_ui32UARTCommand[1] == '1') {
            g_ui32DelayShutter = atoi(inputValue);
            I10ToA(g_ui32DelayShutter, returnValue);
        } else if (g_ui32UARTCommand[1] == '2') {
            g_ui32DelayGasJet = atoi(inputValue);
            I10ToA(g_ui32DelayGasJet, returnValue);
        } else if (g_ui32UARTCommand[1] == '3') {
            g_ui32DelayShotCameras = atoi(inputValue);
            I10ToA(g_ui32DelayShotCameras, returnValue);
        } else {}
        UARTSend(returnValue, 12);
    }

    ResetUARTCommand();
}

//*****************************************************************************
// The UART interrupt handler.
//*****************************************************************************
void UARTIntHandler(void) {
    uint32_t ui32Status;

    // Get the interrupt status.
    ui32Status = MAP_UARTIntStatus(UART0_BASE, true);

    // Clear the asserted interrupts.
    MAP_UARTIntClear(UART0_BASE, ui32Status);

    // Loop while there are characters in the receive FIFO.
    while (MAP_UARTCharsAvail(UART0_BASE)) {
        // Read the next character from the UART and write it back to the UART.
        uint32_t received = MAP_UARTCharGetNonBlocking(UART0_BASE);
        if (received != 13) {
            MAP_UARTCharPutNonBlocking(UART0_BASE, received);
            if (g_ui32UARTCommandIndex < COMMAND_LENGTH) {
                g_ui32UARTCommand[g_ui32UARTCommandIndex] = received;
                g_ui32UARTCommandIndex++;
            }
        } else {
            HandleCommand();
        }

        // Blink the LED to show a character transfer is occurring.
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
        SysCtlDelay(SysCtlClockGet() / (10000 * 3));
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
    }
}

//*****************************************************************************
// Configure UART Communication
//*****************************************************************************
void ConfigureUART() {
    // Enable the peripherals used by this example.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable processor interrupts.
    MAP_IntMasterEnable();

    // Set GPIO A0 and A1 as UART pins.
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure the UART for 115,200, 8-N-1 operation.
    MAP_UARTConfigSetExpClk(UART0_BASE, MAP_SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Enable the UART interrupt.
    MAP_IntEnable(INT_UART0);
    MAP_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}

//*****************************************************************************
// Main 'C' Language entry point.
//*****************************************************************************
int main(void) {
    g_ui32DelayShutter = 80;
    g_ui32DelayGasJet = 90;
    g_ui32DelayShotCameras = 95;

    ResetUARTCommand();
    ConfigureSystemClock();
    ConfigureTimer();
    ConfigureIO_PortE();
    ConfigureIO_PortF();
    ConfigureUART();

    while (1) {}
}
