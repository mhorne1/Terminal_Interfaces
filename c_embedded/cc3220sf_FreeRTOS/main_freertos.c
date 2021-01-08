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
 *  ======== main_freertos.c ========
 *  Built with:
 *  Code Composer Studio 10.1.1.00004
 *  SimpleLink CC32xx SDK 4.30.0.06
 *  FreeRTOS V10.2.1
 */
#include <stdint.h>

/* RTOS header files */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/* Driver header files */
#include <ti/drivers/Board.h>
#include <ti/drivers/GPIO.h>

/* Project header files */
#include "freertos/common.h"

//****************************************************************************
//                          GLOBAL VARIABLES
//****************************************************************************
/* FreeRTOS task handles */
Task_Handles_Block xTasks;

/* Create WIFI command queue */
QueueHandle_t xQueue1 = NULL;

//****************************************************************************
//                          EXTERNAL FUNCTIONS
//****************************************************************************
extern void uartTask(void *arg0);
extern void i2cTask(void *arg0);
extern void wifiTask(void *arg0);

/*
 *  ======== main ========
 */
int main(void)
{
    BaseType_t   retc;

    Board_init();

    xQueue1 = xQueueCreate(PROJECT_QUEUE_LENGTH,sizeof(Q_Data_Block));
    if (xQueue1 == NULL) {
        while (1); // xQueueCreate() failed
    }

    retc = xTaskCreate(uartTask,                          // pvTaskCode
                       "uart_name",                       // pcName
                       UARTTASKSTACKSIZE,                 // usStackDepth
                       NULL,                              // pvParameters
                       1,                                 // uxPriority
                       &xTasks.uartHandle);               // pxCreatedTask
    if (retc != pdPASS) {
        while (1); // xTaskCreate() failed
    }

    retc = xTaskCreate(i2cTask,                           // pvTaskCode
                       "i2c_name",                        // pcName
                       I2CTASKSTACKSIZE,                  // usStackDepth
                       NULL,                              // pvParameters
                       3,                                 // uxPriority
                       &xTasks.i2cHandle);                // pxCreatedTask
    if (retc != pdPASS) {
        while (1); // xTaskCreate() failed
    }

    retc = xTaskCreate(wifiTask,                          // pvTaskCode
                       "wifi_name",                       // pcName
                       WIFITASKSTACKSIZE,                 // usStackDepth
                       NULL,                              // pvParameters
                       2,                                 // uxPriority
                       &xTasks.wifiHandle);               // pxCreatedTask
    if (retc != pdPASS) {
        while (1); // xTaskCreate() failed
    }

    /* Initialize the GPIO since multiple threads are using it */
    GPIO_init();

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    return (0);
}
