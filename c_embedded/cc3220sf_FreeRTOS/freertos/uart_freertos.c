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
 *  ======== uart_freertos.c ========
 *  Built with:
 *  Code Composer Studio 10.1.1.00004
 *  SimpleLink CC32xx SDK 4.30.0.06
 *  FreeRTOS V10.2.1
 */
#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

/* RTOS header files */
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include <queue.h>

/* Driver header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#ifdef CC32XX
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC32XX.h>
#endif

/* Driver configuration */
#include "ti_drivers_config.h"

/* Project header files */
#include "freertos/common.h"
#include "freertos/i2c_setup.h"
#include "freertos/wifi_config.h"
#include "freertos/system_status.h"

//****************************************************************************
//                          GLOBAL VARIABLES
//****************************************************************************
/* UART handle created by UART task */
UART_Handle uart;

/* Allows writing to UART after UART is setup */
volatile bool uartInitFlag = false;

/* WIFI command queue message struct for sending */
Q_Data_Block qMsgSend;

/* Console display strings */
const char projectDisplay[] = "\r\nRemote Monitoring System Project - FreeRTOS - Michael Horne\r\n";
const char helpPrompt[]     = "Valid Commands\r\n"                  \
                              "--------------\r\n"                  \
                              "h: help\r\n"                         \
                              "t: Display temperature data\r\n"     \
                              "a: Display accelerometer data\r\n"   \
                              "1: Display WIFI settings\r\n"        \
                              "2: Change WIFI settings"             \
                              "3: Connect to WIFI\r\n"              \
                              "4: Update device time\r\n"           \
                              "5: Display device time (UTC)\r\n"    \
                              "6: Display TCP Client message\r\n"   \
                              "7: Test TCP Client\r\n"              \
                              "8: Close TCP Client socket\r\n"      \
                              "9: Send TCP Client message\r\n"      \
                              "z: Send new TCP message\r\n"         \
                              "x: Receive new TCP message\r\n"      \
                              "o: Test sending omnibus message\r\n" \
                              "b: Test status bits\r\n"             \
                              "--------------\r\n"                  \
                              "m: Display current memory heap\r\n"  \
                              "s: Display task stack usage\r\n"     \
                              "p: Test UART_printf()\r\n";
const char userPrompt[]       = "> ";
const char readErrDisplay[]   = "Problem reading UART.\r\n";
const char crlfDisplay[]      = "\r\n";

//****************************************************************************
//                          LOCAL FUNCTION PROTOTYPES
//****************************************************************************
void uartTask(void *pvParameters);
void simpleConsole(UART_Handle uart);
void UART_send(const char *str);
int UART_printf(const char *pcFormat, ...);
int getChar(char *pcBuffer);
int getString(char *pcBuffer, unsigned int bufLen);
void configureWIFI(char *pBuffer, unsigned int bufSize);
void checkTemp(void);
void checkAccel(void);
void testStatusBits(char *pcmd);

//****************************************************************************
//                          EXTERNAL FUNCTIONS
//****************************************************************************
extern int vsnprintf(char * s, size_t n, const char * format, va_list arg);

/*
 *  ======== uartTask ========
 *  Initializes UART and runs command console
 *  pvParameters - Optional void pointer
 *  return - void
 */
void uartTask(void *pvParameters)
{
    UART_Params uartParams;

    UART_init();

    /*
     *  Initialize the UART parameters outside the loop. Let's keep
     *  most of the defaults (e.g. baudrate = 115200) and only change the
     *  following.
     */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode  = UART_DATA_BINARY;
    uartParams.readDataMode   = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartInitFlag = true;

    /* Loop forever to start the console */
    while (1) {
        /* Create a UART for the console */
        uart = UART_open(CONFIG_UART_0, &uartParams);
        if (uart == NULL) {
            while (1);
        }

        setStatus(appStatus, STATUS_UART_INIT, true);

        simpleConsole(uart);    // Run command console

        /*
         * Since we returned from the console, we need to close the UART.
         * The Power Manager will go into a lower power mode when the UART
         * is closed.
         */
        UART_close(uart);
    }
}

/*
 *  ======== simpleConsole ========
 *  Command console handles user input. Currently this console does not accept
 *  user back-spaces or other "hard" characters.
 */
void simpleConsole(UART_Handle uart)
{
    char cmd;                       // Input character
    char tempStr[MAX_SSID_LENGTH];  // Input string buffer

    UART_writePolling(uart, projectDisplay, sizeof(projectDisplay));
    UART_printf("UART Console (h for help)\r\n");

    /* Loop forever */
    while (1) {
        getChar(&cmd);              // Get command char from input

        switch (cmd) {
            case 't': // Display temperature data
                checkTemp();
                break;
            case 'a': // Display accelerometer data
                checkAccel();
                break;
            case '1': // Display WIFI settings and status
                qMsgSend.header = WIFI_dset;
                xQueueSendToBack( xQueue1, &qMsgSend, 0 );
                break;
            case '2': // Change WIFI settings
                configureWIFI(tempStr,sizeof(tempStr));
                break;
            case '3': // Connect to WIFI
                qMsgSend.header = WIFI_conn;
                xQueueSendToBack( xQueue1, &qMsgSend, 0 );
                break;
            case '4': // Update device time to UTC time
                qMsgSend.header = WIFI_utim;
                xQueueSendToBack( xQueue1, &qMsgSend, 0 );
                break;
            case '5': // Display device time (UTC time)
                qMsgSend.header = WIFI_dtim;
                xQueueSendToBack( xQueue1, &qMsgSend, 0 );
                break;
            case '6': // Display TCP Client message
                qMsgSend.header = WIFI_dmsg;
                xQueueSendToBack( xQueue1, &qMsgSend, 0 );
                break;
            case '7': // Test sending TCP Client message
                qMsgSend.header = WIFI_tcli;
                xQueueSendToBack( xQueue1, &qMsgSend, 0 );
                break;
            case '8': // Close TCP Client socket
                qMsgSend.header = WIFI_csid;
                xQueueSendToBack( xQueue1, &qMsgSend, 0 );
                break;
            case '9': // Display and then send TCP Client message
                qMsgSend.header = WIFI_dmsg;
                xQueueSendToBack( xQueue1, &qMsgSend, 0 );
                qMsgSend.header = WIFI_tcli;
                xQueueSendToBack( xQueue1, &qMsgSend, 0 );
                break;
            case 'z': // Send new TCP message
                qMsgSend.header = WIFI_cmsg;
                UART_printf("Enter '1' or '2'\r\n");
                getChar(&cmd);
                if (cmd == '1') {
                    qMsgSend.param = WIFI_msg_type1;    // Select Type 1 message
                } else if (cmd == '2') {
                    qMsgSend.param = WIFI_msg_type2;    // Select Type 2 message
                } else {
                    break;                              // Invalid selection
                }
                xQueueSendToBack( xQueue1, &qMsgSend, 0 );
                break;
            case 'x': // Receive new TCP message from server
                qMsgSend.header = WIFI_rmsg;
                xQueueSendToBack( xQueue1, &qMsgSend, 0 );
                break;
            case 'o': // Test sending omnibus message to TCP Server
                qMsgSend.header = WIFI_omsg;
                xQueueSendToBack( xQueue1, &qMsgSend, 0 );
                break;
            case 'b': // Set status bit
                testStatusBits(&cmd);
                break;
            case 'm': // Check memory heap allocation
                UART_printf("configTOTAL_HEAP_SIZE: %d\r\n",
                            (unsigned int)configTOTAL_HEAP_SIZE);
                UART_printf("xPortGetFreeHeapSize: %d\r\n",
                            (unsigned int)xPortGetFreeHeapSize());
                UART_printf("xPortGetMinimumEverFreeHeapSize: %d\r\n",
                            (unsigned int)xPortGetMinimumEverFreeHeapSize());
                break;
            case 's': // Check task stack usage
                UART_printf("UART Task lowest stack = %d/%d\r\n",
                            (unsigned int)uxTaskGetStackHighWaterMark(xTasks.uartHandle),
                            UARTTASKSTACKSIZE);
                UART_printf("I2C Task lowest stack = %d/%d\r\n",
                            (unsigned int)uxTaskGetStackHighWaterMark(xTasks.i2cHandle),
                            I2CTASKSTACKSIZE);
                UART_printf("WIFI Task lowest stack = %d/%d\r\n",
                            (unsigned int)uxTaskGetStackHighWaterMark(xTasks.wifiHandle),
                            WIFITASKSTACKSIZE);
                break;
            case 'p': // Test formatted printing
                UART_printf("0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ\n\r");
                break;
            case 'i': // Auto send TCP Client messages with I2C task
                g_i2cAutoSend = ~g_i2cAutoSend;
                if (g_i2cAutoSend) {
                    UART_printf("Started ");
                } else {
                    UART_printf("Stopped ");
                }
                UART_printf("Automatically sending TCP Client messages\n\r");
                break;
            case 'h':
            default:
                UART_writePolling(uart, helpPrompt, sizeof(helpPrompt));
                break;
        }
    }
}

//*****************************************************************************
//
//! Outputs a character string to the console
//!
//! This function
//!        1. prints the input string character by character on to the console.
//!
//! \param[in]  str - is the pointer to the string to be printed
//!
//! \return none
//!
//! \note If UART_NONPOLLING defined in than Message or UART write should be
//!       called in task/thread context only.
//
//*****************************************************************************
void UART_send(const char *str)
{
    #ifdef UART_NONPOLLING
        UART_write(uart, str, strlen(str));
    #else
        UART_writePolling(uart, str, strlen(str));
    #endif
}

//*****************************************************************************
//
//! prints the formatted string on to the console
//!
//! \param[in]  format  - is a pointer to the character string specifying the
//!                       format in the following arguments need to be
//!                       interpreted.
//! \param[in]  [variable number of] arguments according to the format in the
//!             first parameters
//!
//! \return count of characters printed
//
//*****************************************************************************
int UART_printf(const char *pcFormat, ...)
{
    int iRet = 0;
    char *pcBuff;
    //char *pcTemp;
    int iSize = 256;
    va_list list;

    if(uartInitFlag == false) {
        iRet = -1;
        return(iRet);
    }

    //pcBuff = (char*)malloc(iSize);
    pcBuff = (char*)pvPortMalloc(iSize);
    if(pcBuff == NULL)
    {
        return(-1);
    }
    while(1)
    {
        va_start(list,pcFormat);
        iRet = vsnprintf(pcBuff, iSize, pcFormat, list);
        va_end(list);
        if((iRet > -1) && (iRet < iSize))
        {
            break;
        }
        else
        {
            vPortFree(pcBuff);
            iSize *= 2;
            //if((pcTemp = realloc(pcBuff, iSize)) == NULL)
            //if((pcTemp = (char*)pvPortMalloc(iSize)) == NULL)
            if((pcBuff = (char*)pvPortMalloc(iSize)) == NULL)
            {
                UART_send("Could not reallocate memory\n\r");
                iRet = -1;
                break;
            }
            /*
            else
            {
                pcBuff = pcTemp;
            }
            */
        }
    }
    if (g_clientSendFlag == true) { // Allow copying of string to send buffer
        taskENTER_CRITICAL();
        strncpy((char *)g_pClientSendBuffer, pcBuff, SERVER_FRAME_LENGTH);
        taskEXIT_CRITICAL();
    }
    UART_send(pcBuff);
    //free(pcBuff);
    vPortFree(pcBuff);

    return(iRet);
}

//*****************************************************************************
//
//! Get input char from UART
//!
//! \param[in]  pcBuffer    - is where input is stored
//!
//! \return Length of the bytes received. -1 if buffer length exceeded.
//!
//*****************************************************************************
int getChar(char *pcBuffer)
{
    int status = 0;

    status = UART_read(uart, pcBuffer, sizeof(*pcBuffer));
    if (status == 0) {
        UART_writePolling(uart, readErrDisplay, sizeof(readErrDisplay));
        *(pcBuffer) = 'h';
    } else {
        UART_writePolling(uart, pcBuffer, sizeof(*pcBuffer));
        UART_writePolling(uart, crlfDisplay, sizeof(crlfDisplay));
    }
    return status;
}

//*****************************************************************************
//
//! Get input string from UART
//!
//! \param[in]  pcBuffer    - is where input is stored
//! \param[in]  bufLen      - is the length of buffer store available
//!
//! \return Length of the bytes received. -1 if buffer length exceeded.
//!
//*****************************************************************************
int getString(char *pcBuffer, unsigned int bufLen)
{
    char cmd;
    int i = -1;

    for(i=0; i<(bufLen-1); i++)
    {
        UART_read(uart, &cmd, sizeof(cmd));
        if((cmd == '\r') || (cmd == '\n') || (cmd == '\b')) {
            *(pcBuffer + i) = 0;
            break;
        } else {
            *(pcBuffer + i) = cmd;
            UART_writePolling(uart, &cmd, sizeof(cmd));
        }
    }
    UART_writePolling(uart, crlfDisplay, sizeof(crlfDisplay));
    //UART_printf("Input = %s\r\n", pcBuffer);
    return i;
}

/*
 *  ======== configureWIFI ========
 *  This function allows user input to change the WIFI settings like
 *  WIFI name, WIFI password, and security type.
 *  Changes are received by the WIFI task and then implemented.
 *  pBuffer - char array
 *  bufSize - length of char array
 *  return - void
 */
void configureWIFI(char *pBuffer, unsigned int bufSize)
{
    UART_printf("Enter WIFI name (or SSID):\r\n");
    getString(pBuffer,bufSize);             // Get user input
    qMsgSend.header = WIFI_cset;            // wifiManager command
    qMsgSend.param = WIFI_cfg_ssid;         // WIFI configuration parameter
    qMsgSend.pcbuf = pBuffer;               // char buffer pointer
    xQueueSendToBack( xQueue1, &qMsgSend, 0 );  // Send message to WIFI task

    UART_printf("Password for WIFI?\r\n");
    UART_printf("Y: Yes, will specify WIFI password\r\n");
    UART_printf("N: No, public/open WIFI\r\n");
    getString(pBuffer,bufSize);
    if ((pBuffer[0] == 'N') || (pBuffer[0] == 'n')) {
        qMsgSend.param = WIFI_cfg_open;
        xQueueSendToBack( xQueue1, &qMsgSend, 0 );
        return;
    } else {
        qMsgSend.param = WIFI_cfg_wpa2;
        xQueueSendToBack( xQueue1, &qMsgSend, 0 );
    }

    UART_printf("Enter WIFI password:\r\n");
    getString(pBuffer,bufSize);
    qMsgSend.param = WIFI_cfg_key;
    xQueueSendToBack( xQueue1, &qMsgSend, 0 );
}

/*
 *  ======== checkTemp ========
 *  This function allows user input to view the most recent temperature sensor
 *  values that were read
 *  parameters - void
 *  return - void
 */
void checkTemp(void)
{
    float localTempC;       // Temperature value in Celsius
    float localTempF;       // Temperature value in Fahrenheit

    /*
    *  Access the global float temperature variables in a
    *  thread-safe manner.
    *  Semaphores should be created to protect these global variables
    */
    taskENTER_CRITICAL();
    localTempC = g_temperatureC;
    localTempF = g_temperatureF;
    taskEXIT_CRITICAL();
    UART_printf("Current temp = %3.2fC (%3.2fF)\n\r", localTempC, localTempF);
}

/*
 *  ======== checkAccel ========
 *  This function allows user input to view the most recent accelerometer
 *  sensor values that were read
 *  parameters - void
 *  return - void
 */
void checkAccel(void)
{
    int8_t localxVal;       // Accelerometer X-axis value
    int8_t localyVal;       // Accelerometer Y-axis value
    int8_t localzVal;       // Accelerometer Z-axis value

    /*
     *  Access the global int8_t accelerometer variables in a
     *  thread-safe manner.
     *  Semaphores should be created to protect these global variables
     */
    taskENTER_CRITICAL();
    localxVal = (int8_t)g_xVal;
    localyVal = (int8_t)g_yVal;
    localzVal = (int8_t)g_zVal;
    taskEXIT_CRITICAL();
    UART_printf("AccX = %d\n\r", localxVal);
    UART_printf("AccY = %d\n\r", localyVal);
    UART_printf("AccZ = %d\n\r", localzVal);
}

/*
 *  ======== testStatusBits ========
 *  This function allows user input to either assert or clear
 *  one of the status bits
 *  pcmd - char pointer
 *  return - void
 */
void testStatusBits(char *pcmd)
{
    uint16_t tempVar;       // Numeric input buffer

    UART_printf("Enter a number [0-F]\r\n");
    //getString(tempStr,sizeof(tempStr));
    //tempVar = atoi(tempStr);
    getChar(pcmd);
    if ((*pcmd>='0') && (*pcmd<='9')) {
        tempVar = *pcmd - '0';                // '0' == 48
    } else if ((*pcmd>='A') && (*pcmd<='F')) {
        tempVar = (*pcmd - 'A') + 10;         // 'A' == 65
    } else if ((*pcmd>='a') && (*pcmd<='f')) {
        tempVar = (*pcmd - 'a') + 10;         // 'a' == 97
    }

    UART_printf("Status word 0 or 1?\r\n");
    getChar(pcmd);
    if (*pcmd=='1') {
        tempVar += 32;
    }

    //UART_printf("tempVar = %u\r\n", tempVar);

    UART_printf("Assert or Clear?\r\n");
    getChar(pcmd);
    if ((*pcmd=='A') || (*pcmd=='a')) {
        setStatus(appStatus, tempVar, true);
    } else {
        setStatus(appStatus, tempVar, false);
    }
    UART_printf("appStatus[0] = 0x%08X\r\nappStatus[1] = 0x%08X\r\n",
                appStatus[0], appStatus[1]);
}
