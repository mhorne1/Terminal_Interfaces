/*
 * Copyright (c) 2016-2020, Texas Instruments Incorporated
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
 *  ======== i2c_freertos.c ========
 *  Built with:
 *  Code Composer Studio 10.1.1.00004
 *  SimpleLink CC32xx SDK 4.30.0.06
 *  FreeRTOS V10.2.1
 */
#include <stdint.h>
#include <unistd.h>

/* RTOS header files */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h> //

/* Driver configuration */
#include "ti_drivers_config.h"

/* Accelerometer Drivers */
#include "bma2xxdrv.h"

/* Project header files */
#include "freertos/common.h"
#include "freertos/i2c_setup.h"
#include "freertos/wifi_config.h"

//****************************************************************************
// GLOBAL VARIABLES
//****************************************************************************
/* Temperature written by the I2C thread and read by console thread */
volatile float g_temperatureC;
volatile float g_temperatureF;

/* Accelerometer values written by the I2C thread and read by console thread */
volatile int8_t g_xVal;
volatile int8_t g_yVal;
volatile int8_t g_zVal;

/* Local WIFI command queue message struct for sending */
Q_Data_Block qI2CSend;

/* Auto send TCP Client messages with I2C task */
volatile char g_i2cAutoSend = false;

//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
void i2cTask(void *pvParameters);
uint8_t accelerometerReading(I2C_Handle i2cHandle);

/*
 *  ======== i2cTask ========
 *  This task reads the temperature sensor and the accelerometer sensor via
 *  I2C at a target rate of I2CTASKTICKS
 *
 */
void i2cTask(void *pvParameters)
{
    uint8_t         txBuffer[1];    // Buffer for writing to I2C bus
    uint8_t         rxBuffer[2];    // Buffer for receiving on I2C bus
    I2C_Handle      i2c;            // Handle for I2C bus
    I2C_Params      i2cParams;      // I2C bus configuration
    I2C_Transaction i2cTransaction; // I2C driver struct
    char red_led_state = 0;         // Used to control Red LED indication
    TickType_t lastWake;            // Used to control task timing behavior

    /* Configure the LED and if applicable, the TMP_EN pin */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
#ifdef CONFIG_GPIO_TMP_EN
    GPIO_setConfig(CONFIG_GPIO_TMP_EN, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);
    /* 1.5 ms reset time for the TMP sensor */
    sleep(1);
#endif

    /*
     *  Create/Open the I2C that talks to the TMP an ACC sensors
     */
    I2C_init();

    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(CONFIG_I2C_TMP, &i2cParams);
    if (i2c == NULL) {
        while (1);
    }

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf   = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf    = rxBuffer;
    i2cTransaction.readCount  = 2;

    /*
     * Determine which I2C sensor is present.
     * We will prefer sensors in this order: TMP117 (on BoosterPacks),
     * TMP116 (on-board CC32XX LaunchPads), and last TMP006
     * (on older CC32XX LaunchPads).
     */
    /* Try BP TMP values */
    txBuffer[0] = TMP_BP_REG;
    i2cTransaction.slaveAddress = TMP_BP_ADDR;
    if (!I2C_transfer(i2c, &i2cTransaction)) {
        /* Not BP TMP, try LP TMP116 */
        i2cTransaction.slaveAddress = TMP116_LP_ADDR;
        if (!I2C_transfer(i2c, &i2cTransaction)) {
            /* Not a TMP116 try TMP006*/
            txBuffer[0] = TMP006_REG;
            i2cTransaction.slaveAddress = TMP006_ADDR;
            if (!I2C_transfer(i2c, &i2cTransaction)) {
                /* Could not resolve a sensor, error */
                while(1);
            }
        }
    }

    lastWake = xTaskGetTickCount(); // Set starting point for task timing

    while (1) {
        if (I2C_transfer(i2c, &i2cTransaction)) {
            /*
             *  Extract degrees C from the received data; see sensor datasheet.
             *  Make sure we are updating the global temperature variables
             *  in a thread-safe manner.
             *  Semaphores should be created to protect these global variables
             */
            taskENTER_CRITICAL();
            g_temperatureC = (rxBuffer[0] << 6) | (rxBuffer[1] >> 2);
            g_temperatureC *= 0.03125;
            g_temperatureF = g_temperatureC * 9 / 5 + 32;
            taskEXIT_CRITICAL();

            /*
            UART_printf("rxBuffer[0] = 0x%02X\r\n", rxBuffer[0]);
            UART_printf("rxBuffer[1] = 0x%02X\r\n", rxBuffer[1]);
            UART_printf("Current temp = %dC (%dF)\n\r",
                        (int)g_temperatureC,
                        (int)g_temperatureF);
            */
        }

        /* Read from 3-axis accelerometer */
        accelerometerReading(i2c);

        /*
        UART_printf("AccX = %d\n\r", g_xVal);
        UART_printf("AccY = %d\n\r", g_yVal);
        UART_printf("AccZ = %d\n\r", g_zVal);
        */

        /* Semaphores should be created to protect these global variables */

        /* Control Red LED Status Indication */
        if (g_wifiConnectFlag == false) {
            // Keep Red LED off
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
        } else if (( g_wifiConnectFlag == true) && (g_clientConnectFlag == false)) {
            // Keep Red LED on
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
        } else if (g_clientConnectFlag == true) {
            // Blink Red LED
            red_led_state = ~red_led_state;
            if (red_led_state) {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            }
            else {
                GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            }
        }

        /* Auto send TCP Client messages with I2C task cycle */
        if (g_i2cAutoSend) {
            if (g_wifiConnectFlag == true) {
                qI2CSend.header = WIFI_dmsg;
                xQueueSendToBack( xQueue1, &qI2CSend, 0 );
                qI2CSend.header = WIFI_tcli;
                xQueueSendToBack( xQueue1, &qI2CSend, 0 );
            } else {
                g_i2cAutoSend = false;
            }
        }

        /* Block I2C task until interval of ticks */
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(I2CTASKTICKS));
    }
}

//*****************************************************************************
//
//! Function to read accelerometer
//!
//! \param  I2C_Handle
//!
//! \return SUCCESS or FAILURE
//!
//*****************************************************************************
uint8_t accelerometerReading(I2C_Handle i2cHandle)
{
    int8_t xValRead, yValRead, zValRead;
    int32_t status;

    /* Read accelerometer axis values */
    status = BMA2xxReadNew(i2cHandle, &xValRead, &yValRead, &zValRead);
    if(status != 0)
    {
        /* try to read again */
        status = BMA2xxReadNew(i2cHandle, &xValRead, &yValRead, &zValRead);
        if(status != 0)     /* leave previous values */
        {
            UART_printf("[Link local task] Failed to read data from accelerometer\n\r");
        }
    }

    if(status == 0)
    {
        /* Semaphores should be created to protect these global variables */
        taskENTER_CRITICAL();
        g_xVal = xValRead;
        g_yVal = yValRead;
        g_zVal = zValRead;
        taskEXIT_CRITICAL();
    }

    return(status);
}
