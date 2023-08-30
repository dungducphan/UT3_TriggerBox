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

#define COMMAND_LENGTH  10
#define RETURNED_LENGTH 5
#define RETURN_SYMBOL   13

//*****************************************************************************
// Global variables
//*****************************************************************************
volatile uint32_t g_ui32Delay0_in_millisec;
volatile uint32_t g_ui32Delay1_in_millisec;
// UART command array
volatile uint32_t g_ui32UARTCommand[COMMAND_LENGTH];
volatile uint32_t g_ui32UARTCommandIndex;
// State
volatile bool     g_ui32RunningState;

//*****************************************************************************
// Reset UART Commands
//*****************************************************************************
void ResetUARTCommand() {
    uint8_t i = 0;
    for (i = 0; i < COMMAND_LENGTH; i++) g_ui32UARTCommand[i] = 0;
    g_ui32UARTCommandIndex = 0;
}

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
    // If TimerLoadSet value is SysCtlClockGet, then the timer will finish in exactly 1 second
    // If we want a X millisecond timer, the load set value has to be (SysCtlClockGet() / 1000) * X
    MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, MAP_SysCtlClockGet() / 1000 * g_ui32Delay0_in_millisec);
    MAP_TimerLoadSet(TIMER1_BASE, TIMER_A, MAP_SysCtlClockGet() / 1000 * g_ui32Delay1_in_millisec);
    // Enable the timers.
    MAP_TimerEnable(TIMER0_BASE, TIMER_A);
    MAP_TimerEnable(TIMER1_BASE, TIMER_A);
}


//*****************************************************************************
// Send a string to the UART.
//*****************************************************************************
void UARTSend(const char *pui8Buffer, uint32_t ui32Count) {
    while (ui32Count--) {
        // Write the next character to the UART.
        MAP_UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
    }
}

//*****************************************************************************
// Send Trigger Event to UART
//*****************************************************************************
void SendTriggerEvent() {
    char returnValue[1] = "E";
    UARTSend(returnValue, RETURNED_LENGTH);
}

//*****************************************************************************
// The interrupt handler for PortE1
//*****************************************************************************
void PortEIntHandler(void) {
    // Clear Interrupt flag
    MAP_GPIOIntClear(GPIO_PORTE_BASE, GPIO_INT_PIN_1);

    // Every time PE1 change state from LOW to HIGH, an interrupt is issued
    // If PE2 is at HIGH state and the TriggerBox is in RUNNING_STATE, timers will be started
    if (MAP_GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_2) == 0x4 && g_ui32RunningState) {
        SendTriggerEvent();
        StartTimer();
    }

    // Reset pins B[0-7] and D[0-3]
    MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0x0);
    MAP_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x0);
}

//*****************************************************************************
// Setting up the IO pins
//*****************************************************************************
void ConfigureIOPorts() {
//////// Port E

    // Enable and wait for the port to be ready for access
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)) {}

    // Configure the pins E1, E2 to be the input clocks
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2);

    // Enable pin PE1 to work as the 10Hz signal
    // When this PE1 goes from low to high, an interrupt will be issued
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1);
    MAP_GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_RISING_EDGE);
    MAP_GPIOIntRegister(GPIO_PORTE_BASE, &PortEIntHandler);
    MAP_GPIOIntEnable(GPIO_PORTE_BASE, GPIO_INT_PIN_1);

//////// Port B

    // Enable and wait for the port to be ready for access
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {}

    // Configure the pins B[0-7] to be the output
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

//////// Port D

    // Enable and wait for the port to be ready for access
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while (!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)) {}

    // Configure the pins D0, D1, D2, D3 to be the output
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
}

//*****************************************************************************
//  Configure Periodic Timer
//*****************************************************************************
void ConfigureTimer(void) {
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    // Enable processor interrupts.
    MAP_IntMasterEnable();

    // Configure the two 32-bit periodic timers.
    MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT);
    MAP_TimerConfigure(TIMER1_BASE, TIMER_CFG_ONE_SHOT);

    // Setup the interrupts for the timer timeouts.
    MAP_IntEnable(INT_TIMER0A);
    MAP_IntEnable(INT_TIMER1A);
    MAP_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    MAP_TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
}

//*****************************************************************************
// The interrupt handler for TIMER0
//*****************************************************************************
void Timer0IntHandler(void) {
    // Clear the timer interrupt.
    MAP_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // At the end of TIMER_0 duration, set these pins to HIGH
    // These pins are kept HIGH until the next 10 Hz signal
    MAP_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0xFF);
}

//*****************************************************************************
// The interrupt handler for TIMER1
//*****************************************************************************
void Timer1IntHandler(void) {
    // Clear the timer interrupt.
    MAP_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    // At the end of TIMER_1 duration, set these pins to HIGH
    // These pins are kept HIGH until the next 10 Hz signal
    MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, 0xFF);

}

void I10ToA(uint32_t value, char* result) {
    uint32_t base = 10;
    char* ptr = result, *ptr1 = result, tmp_char;
    int tmp_value;

    do {
        tmp_value = value;
        value = value / base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
    } while (value);

    // Apply negative sign
    if (tmp_value < 0) *ptr++ = '-';
    *ptr-- = ' ';
    while (ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
}

//*****************************************************************************
// Handle UART commands
//*****************************************************************************

void ReadDelay() {
    char returnValue[RETURNED_LENGTH] = "     ";
    if        (g_ui32UARTCommand[1] == '0') {
        I10ToA(g_ui32Delay0_in_millisec, returnValue);
    } else if (g_ui32UARTCommand[1] == '1') {
        I10ToA(g_ui32Delay1_in_millisec, returnValue);
    } else {}
    UARTSend(returnValue, RETURNED_LENGTH);
}

void WriteDelay() {uint8_t i = 0;
    char inputValue[6]   = "      ";
    for (i = 0; i < 6; i++) inputValue[i] = (char) g_ui32UARTCommand[i + 3];
    if        (g_ui32UARTCommand[1] == '0') {
        g_ui32Delay0_in_millisec = atoi(inputValue);
    } else if (g_ui32UARTCommand[1] == '1') {
        g_ui32Delay1_in_millisec = atoi(inputValue);
    } else {}
}

void HandleCommand() {
    if        (g_ui32UARTCommand[0] == 'R') {
        // Read delay values from TIMER channels
        ReadDelay();
    } else if (g_ui32UARTCommand[0] == 'W') {
        // Set delay values for TIMER channels
        WriteDelay();
    } else if (g_ui32UARTCommand[0] == 'P') {
        // Stop the Trigger box: no triggers
        g_ui32RunningState = false;
    } else if (g_ui32UARTCommand[0] == 'S') {
        // Start the Trigger box: issuing triggers
        g_ui32RunningState = true;
    }

    ResetUARTCommand();
}

//*****************************************************************************
// The UART interrupt handler.
//*****************************************************************************
void UARTIntHandler(void) {
    // Clear the asserted interrupts.
    MAP_UARTIntClear(UART0_BASE, MAP_UARTIntStatus(UART0_BASE, true));

    // Loop while there are characters in the receive FIFO.
    while (MAP_UARTCharsAvail(UART0_BASE)) {
        uint32_t received = MAP_UARTCharGetNonBlocking(UART0_BASE);
        if (received != RETURN_SYMBOL) {
            if (g_ui32UARTCommandIndex < COMMAND_LENGTH) {
                g_ui32UARTCommand[g_ui32UARTCommandIndex] = received;
                g_ui32UARTCommandIndex++;
            }
        } else {
            HandleCommand();
        }
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
    g_ui32Delay0_in_millisec = 80;
    g_ui32Delay1_in_millisec = 80;
    g_ui32RunningState = true;
    ResetUARTCommand();

    ConfigureSystemClock();
    ConfigureTimer();
    ConfigureIOPorts();
    ConfigureUART();

    while (1) {}
}
