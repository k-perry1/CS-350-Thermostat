/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Timer.h>


/* Driver configuration */
#include "ti_drivers_config.h"



/*                    *
 * ------ UART ------ *
 *                    */
#define DISPLAY(x) UART_write(uart, &output, x);

// UART Global Variables
char output[64];
int bytesToSend;

// Driver Handles - Global variables
UART_Handle uart;


void initUART(void)
{
    UART_Params uartParams;
    // Init the driver
    UART_init();
    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;
    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);
    if (uart == NULL) {
    /* UART_open() failed */
    while (1);
    }
}




/*                   *
 * ------ I2C ------ *
 *                   */

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
char *id;
} sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};

uint8_t             txBuffer[1];
uint8_t             rxBuffer[2];
I2C_Transaction     i2cTransaction;

// Driver Handles - Global variables
I2C_Handle i2c;

// Make sure you call initUART() before calling this function.
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"))

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses
    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;
    for (i=0; i<3; ++i)
    {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;

    DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))
    if (I2C_transfer(i2c, &i2cTransaction))
        {
        DISPLAY(snprintf(output, 64, "Found\n\r"))
        found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }

    if(found)
    {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.slaveAddress))
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
    }
}


int16_t readTemp(void)
{
    int j;
    int16_t temperature = 0;

    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
        * Extract degrees C from the received data;
        * see TMP sensor datasheet
        */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;

        /*
        * If the MSB is set '1', then we have a 2's complement
        * negative value which needs to be sign extended
        */
        if (rxBuffer[0] & 0x80)
        {
        temperature |= 0xF000;
        }
     }
    else
    {
            DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }

    return temperature;
}



/*                     *
 * ------ Timer ------ *
 *                     */

// Driver Handles - Global variables
Timer_Handle timer0;

volatile unsigned char TimerFlag = 0;

void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;
}

void initTimer(void)
{
    Timer_Params params;

    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);
    params.period = 1000000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}




/*                              *
 * ------ Task Scheduler ------ *
 *                              */

// Global variables
volatile unsigned char heat = 0;
int16_t setpoint;
int16_t temperature;
int16_t seconds;
volatile unsigned char button0 = 0;
volatile unsigned char button1 = 0;


void gpioButtonFxn0(uint_least8_t index)
{
    // Flag for button pressing
    button0 = 1;
}

void gpioButtonFxn1(uint_least8_t index)
{
    // Flag for button pressing
    button1 = 1;
}


/* ---- State Machine for Buttons ---- */
enum B_states { B_Start, B_0, B_1 } B_state;
void TickFct_Button() {
    // Transitions
    switch(B_state) {
        // Idle in starting state until a button is pressed
        case B_Start:
            if(button0){
                B_state = B_0;
            }
            if(button1){
                B_state = B_1;
            }
            break;
        // If other button pressed, switch states. Otherwise state remains
        case B_0:
            if(button1) {
                B_state = B_1;
            }
            break;
        case B_1:
            if(button0) {
                B_state = B_0;
            }
            break;
    }
    // State actions
    switch(B_state) {
        case B_Start:
            break;
        case B_0:
            --setpoint;     // Lower set point
            button0 = 0;    // Lower flag
            break;
        case B_1:
            ++setpoint;     // Raise set point
            button1 = 0;    //Lower flag
    }

}

/* ---- State Machine for LED ---- */
enum LED_states { LED_Start, LED_ON, LED_OFF } LED_state;
void TickFct_LED() {
    // Transitions
    switch(LED_state) {
        // Starting state, if set point is lower then temperature, keeps the light off
        case LED_Start:
            if(temperature > setpoint) {
                LED_state = LED_OFF;
                break;
            }
            LED_state = LED_ON;
            break;
        case LED_OFF:
            if (temperature < setpoint) {
                LED_state = LED_ON;
            }
            break;
        case LED_ON:
            if (temperature > setpoint) {
                LED_state = LED_OFF;
            }
            break;
    }
    // State actions
    switch(LED_state){
    case LED_Start:
        break;
    case LED_OFF:
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF); // Turns off LED =
        heat = 0;                                           // Heat is marked as off
        break;
    case LED_ON:
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);  // Turns LED on
        heat = 1;                                           // Heat is marked as on
        break;
    }
}



void *mainThread(void *arg0) {

    /* Call driver init functions */
   GPIO_init();

   /* Configure the LED and button pins */
   GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
   GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

   /* Turn on user LED */
   GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

   /* Install Button callback */
   GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

   /* Enable interrupts */
   GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

   /*
    *  If more than one input pin is available for your device, interrupts
    *  will be enabled on CONFIG_GPIO_BUTTON1.
    */
   if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
       /* Configure BUTTON1 pin */
       GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

       /* Install Button callback */
       GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
       GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
   }

   // Turn both LEDs off
   GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);

    // Initialize other peripherals
    initUART();
    initI2C();
    initTimer();


    unsigned long B_elapsedTime     = 200000;       // 200ms period for checking the button flags
    unsigned long Temp_elapsedtime  = 500000;       // 500ms period for checking temp
    unsigned long LED_elapsedTime   = 1000000;      // 1s period for updating LED and displaying information
    const unsigned long timerPeriod = 100000;       // 100ms for timing conditions

    temperature = readTemp();       // Initialize temperature to current temp
    setpoint = readTemp();          // Set setpoint to current temp
    seconds = 0;

    // Declare starting states
    B_state = B_Start;
    LED_state = LED_Start;

    while(1)
    {
        TimerFlag = 0;                      // Lower flag raised by timer
        while (!TimerFlag){}                // Wait

        if (B_elapsedTime >= 200000) {
            TickFct_Button();               // Call to button state machine
            B_elapsedTime = 0;              // Reset elapsed time for buttons
        }
        if(Temp_elapsedtime >= 500000) {
            temperature = readTemp();       // Check temperature
            Temp_elapsedtime = 0;           // Reset elapsed time for temperature
        }
        if (LED_elapsedTime >= 1000000) {
            TickFct_LED();                  // Call to LED state machine
            DISPLAY( snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, setpoint, heat, seconds)) // Prints current values to terminal
            seconds += 1;                   // Increment seconds
            LED_elapsedTime = 0;            // Reset elapsed time for LED
        }

        // Add 100ms to each elapsed time for counting
        B_elapsedTime += timerPeriod;
        LED_elapsedTime += timerPeriod;
        Temp_elapsedtime += timerPeriod;
    }
}












