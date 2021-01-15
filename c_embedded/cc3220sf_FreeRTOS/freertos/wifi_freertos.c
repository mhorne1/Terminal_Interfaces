/*
 * Copyright (c) 2016, Texas Instruments Incorporated
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
 *  ======== wifi_freertos.c ========
 *  Built with:
 *  Code Composer Studio 10.1.1.00004
 *  SimpleLink CC32xx SDK 4.30.0.06
 *  FreeRTOS V10.2.1
 */
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>

/* RTOS header files */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/* TI-DRIVERS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/net/wifi/simplelink.h>
#include <ti/net/utils/clock_sync.h>
#include <ti/drivers/net/wifi/slnetifwifi.h>

/* POSIX header files */
#include "pthread.h"
#include "semaphore.h"

/* Project header files */
#include "freertos/common.h"
#include "freertos/wifi_config.h"
#include "freertos/wifi_time.h"
#include "freertos/wifi_client.h"
#include "freertos/i2c_setup.h"

//****************************************************************************
//                          GLOBAL VARIABLES
//****************************************************************************
/* WIFI command queue message struct for receiving */
Q_Data_Block qMsgRecv;

/* Struct containing WIFI configuration strings and security mode */
WifiSetup_ControlBlock WifiSetup_CB = {
    .ssid = LOCALTIME_SSID_NAME,            // WIFI name
    .key = LOCALTIME_SECURITY_KEY,          // WIFI Password
    .security = LOCALTIME_SECURITY_TYPE     // WIFI security type
};

/* Struct containing WIFI operation mode */
LocalTime_ControlBlock LocalTime_CB;

/* Struct containing socket configuration for TCP Client */
PowerMeasure_AppData    PowerMeasure_appData = {
    .useCase = PM_USECASE,                  /* The exercised use case  */
    .alwaysConnectedUseCase = AC_USECASE,   /* The Always connected use case  */
    .pktsToDo = NUM_OF_PKT,                 /* how many Intervals do on this use case */
    .sockID = OPEN_SOCK_ONCE,               /* socket ID*/
    .socketType = SOCKET_TYPE,              /* Socket type */
    .ipAddr = DEST_IP_ADDR,                 /* IP address */
    .port = PORT,                           /* socket port number */
};

/* Struct containing server address info and send buffer */
PowerMeasure_ControlBlock   PowerMeasure_CB;

/* Struct containing socket configuration for TCP Client */
ClientMsg_ControlBlock    ClientMsg_appData = {
    .msg_len = FRAME_LENGTH,            /* 80 bytes for text */
    .msg_type = 1,                      /* 1 for text  */
    .msg_number = 0,                    /* Beginning number */
    .message = DEFAULT_MESSAGE,         /* Client message */
};

/* Semaphores should be created to protect these global variables */

/* New client message number */
volatile uint32_t g_message_number = 0;

/* Pointer to send buffer used by TCP Client */
signed char *g_pClientSendBuffer = PowerMeasure_CB.frameData;

/* Flag controls copying string to send buffer used by TCP Client */
bool g_clientSendFlag = false;

/* Flag for limiting activities until WIFI is connected */
bool g_wifiConnectFlag = false;

/* Flag for limiting activities until time is acquired */
bool g_timeAcquiredFlag = false;

/* Flag for indicating when the TCP Client is connected to the server */
bool g_clientConnectFlag = false;

/* Used to store copy of WIFI mode */
int32_t g_wifiMode;

//****************************************************************************
//                          LOCAL FUNCTION PROTOTYPES
//****************************************************************************
void wifiTask(void *pvParameters);
int wifiManager( unsigned int cmd );

//****************************************************************************
//                          EXTERNAL FUNCTIONS
//****************************************************************************
extern int32_t ti_net_SlNet_initConfig();

//*****************************************************************************
//
//! \brief WIFI Task created by main function.
//!
//! \param pvParameters is a general void pointer (not used here).
//!
//! \return none
//
//*****************************************************************************
void wifiTask(void *pvParameters)
{
    int32_t status = 0;                         // Function return value
    pthread_t spawn_thread = (pthread_t)NULL;   // Used for SimpleLink task
    pthread_attr_t pAttrs_spawn;                // Used for SimpleLink task
    struct sched_param priParam;                // Used for SimpleLink task
    int32_t mode;                               // WIFI mode

    /* Initialize SlNetSock layer with CC31xx/CC32xx interface */
    status = ti_net_SlNet_initConfig();
    if(0 != status)
    {
        UART_printf("Failed to initialize SlNetSock\n\r");
    }

    SPI_init();

    /* Clear SimpleLink Status */
    LocalTime_CB.status = 0;

    /* Start the SimpleLink Host */
    pthread_attr_init(&pAttrs_spawn);
    priParam.sched_priority = LOCALTIME_SPAWN_TASK_PRIORITY;
    status = pthread_attr_setschedparam(&pAttrs_spawn, &priParam);
    status |= pthread_attr_setstacksize(&pAttrs_spawn,
                                        LOCALTIME_SPAWN_STACK_SIZE);

    status = pthread_create(&spawn_thread, &pAttrs_spawn, sl_Task, NULL);

    if (status != 0) {
        UART_printf("Could not create simplelink task\n\r");
        return;
    }

    /* Displays the Application Banner */
    //LocalTime_displayBanner();

    /* initialize the device */
    mode = sl_WifiConfig();
    if (mode < 0) {
        UART_printf("Failed to configure device in its default state\n\r");
        return;
    }

    /* Turn WIFI Network Processor (NWP) on */
    mode = sl_Start(NULL, NULL, NULL);

    g_wifiMode = mode; // Store copy of WIFI mode

    while(1)
    {
        if ((mode == ROLE_STA) &&
            (!LOCALTIME_IS_CONNECTED(LocalTime_CB.status)) &&
            (AUTO_RECONNECT)) {
            LocalTime_connect();
        }
        else {
            /* Check for message from WIFI command queue */
            xQueueReceive( xQueue1, &qMsgRecv, pdMS_TO_TICKS(WIFITASKTICKS) );
            if (qMsgRecv.header != 0) {         // Check valid header command
                wifiManager(qMsgRecv.header);   // Process header command
                qMsgRecv.header = 0;            // Reset header command
            }
        }
    }
}

/*
 *  ======== wifiManager ========
 *  This function accepts commands to control the activities of the WIFI task.
 *  Commands vary from displaying WIFI settings to sending messages to the
 *  TCP Client server. Commands are compared with WIFI task flags which
 *  require activities to be run in a logical order.
 *  cmd - Activity command
 *  return - int status
 */
int wifiManager(unsigned int cmd)
{
    int32_t status = -1;        // Function return value
    int32_t send_status = 0;    // Function return value for TCP Client
    float outTempC;             // Temperature value in Celsius
    float outTempF;             // Temperature value in Fahrenheit
    int8_t outxVal;             // Accelerometer X-axis value
    int8_t outyVal;             // Accelerometer Y-axis value
    int8_t outzVal;             // Accelerometer Z-axis value
    struct tm netTime;          // Acquired time value
    char *timeString;           // Pointer to string converted from time value
    uint8_t ssid[33];           // Used for WIFI connection
    uint16_t len = 33;          // Used for WIFI connection
    uint16_t config_opt = SL_WLAN_AP_OPT_SSID;  // Used for WIFI connection

    if (g_wifiConnectFlag == false) {
        if (cmd > WIFI_conn) {
            UART_printf("WIFI not connected yet\n\r");
            return status;
        }
    } else if (g_timeAcquiredFlag == false) {
        if (cmd >= WIFI_tcli) {
            UART_printf("Time not acquired yet\n\r");
            return status;
        }
    }

    switch (cmd) {
        case WIFI_dset: // Display WIFI settings and status
            UART_printf("WIFI Name = %s\n\r", WifiSetup_CB.ssid);
            if (WifiSetup_CB.security == SL_WLAN_SEC_TYPE_WPA_WPA2) {
                UART_printf("Security Type = WPA2\n\r");
                UART_printf("Password = %s\n\r", WifiSetup_CB.key);
            } else if (WifiSetup_CB.security == SL_WLAN_SEC_TYPE_OPEN) {
                UART_printf("Security type = OPEN\n\r");
                UART_printf("No password for public/open WIFI\n\r");
            }

            UART_printf("External TCP Server Settings:\n\r");
            UART_printf("IP Address = %d.%d.%d.%d\n\r",
                        (uint8_t)SL_IPV4_BYTE(PowerMeasure_appData.ipAddr,3),
                        (uint8_t)SL_IPV4_BYTE(PowerMeasure_appData.ipAddr,2),
                        (uint8_t)SL_IPV4_BYTE(PowerMeasure_appData.ipAddr,1),
                        (uint8_t)SL_IPV4_BYTE(PowerMeasure_appData.ipAddr,0));
            UART_printf("Port = %d\n\r", PowerMeasure_appData.port);

            UART_printf("WIFI Activity Status:\n\r");
            UART_printf("WIFI ");
            if (g_wifiConnectFlag == true) {
                UART_printf("[Connected]\n\r");
            } else {
                UART_printf("[Disconnected]\n\r");
            }
            UART_printf("Time ");
            if (g_timeAcquiredFlag == true) {
                UART_printf("[Acquired]\n\r");
            } else {
                UART_printf("[Not Acquired]\n\r");
            }
            UART_printf("TCP Client ");
            if (g_clientConnectFlag == true) {
                UART_printf("[Connected]\n\r");
            } else {
                UART_printf("[Disconnected]\n\r");
            }
            break;
        case WIFI_cset: // Change WIFI settings
            if (qMsgRecv.param == WIFI_cfg_ssid) {
                //UART_printf("ssid = pcbuf = %s\n\r", qMsgRecv.pcbuf);
                strcpy((char *)WifiSetup_CB.ssid, qMsgRecv.pcbuf);
            } else if (qMsgRecv.param == WIFI_cfg_key) {
                //UART_printf("key = pcbuf = %s\n\r", qMsgRecv.pcbuf);
                strcpy((char *)WifiSetup_CB.key, qMsgRecv.pcbuf);
            } else if (qMsgRecv.param == WIFI_cfg_wpa2) {
                WifiSetup_CB.security = SL_WLAN_SEC_TYPE_WPA_WPA2;
            } else if (qMsgRecv.param == WIFI_cfg_open) {
                WifiSetup_CB.security = SL_WLAN_SEC_TYPE_OPEN;
                memset(&WifiSetup_CB.key[0], 0, sizeof(WifiSetup_CB.key));
            }
            break;
        case WIFI_conn: // Connect to WIFI
            if (g_wifiConnectFlag == true) {
                UART_printf("WIFI already connected\n\r");
                break;
            }

            if (g_wifiMode == ROLE_STA) {
                LocalTime_connect();
                g_wifiConnectFlag = (LOCALTIME_IS_IP_ACQUIRED(LocalTime_CB.status));
            }
            else if(g_wifiMode == ROLE_AP)
            {
                sl_Memset(ssid,0,33);
                sl_WlanGet(SL_WLAN_CFG_AP_ID, &config_opt, &len, ssid);
                UART_printf("AP mode SSID %s\n\r",ssid);
            }
            break;
        case WIFI_utim: // Update device time to UTC time
            status = ClockSync_update(); // Redundant, ClockSync_get() updates
            if (status == 0) {
                g_timeAcquiredFlag = true;
                UART_printf("Time updated\n\r");
            } else if (status == CLOCKSYNC_ERROR_INTERVAL) {
                UART_printf("Minimum time between updates did not elapse\n\r");
            } else {
                UART_printf("Time update failed = %d\n\r",status);
            }
            break;
        case WIFI_dtim: // Display device time (UTC time)
            status = ClockSync_get(&netTime);
            if ((status == 0) || (status == CLOCKSYNC_ERROR_INTERVAL)) {
                g_timeAcquiredFlag = true;
                UART_printf("UTC time = %s\r",asctime(&netTime));
            } else {
                UART_printf("Error = %d\n\r",status);
            }
            break;
        case WIFI_dmsg: // Display newly prepared message for TCP Client
            status = ClockSync_get(&netTime);
            if ((status == 0) || (status == CLOCKSYNC_ERROR_INTERVAL)) {
                /* Semaphores should be created to protect these global variables */
                g_timeAcquiredFlag = true;

                /* Get measurement data from I2C task */
                taskENTER_CRITICAL();
                outTempC = g_temperatureC;
                outTempF = g_temperatureF;
                outxVal = (int8_t)g_xVal;
                outyVal = (int8_t)g_yVal;
                outzVal = (int8_t)g_zVal;
                taskEXIT_CRITICAL();

                timeString = asctime(&netTime);         // Get string with date and time
                timeString[strlen(timeString)-1] = 0;   // Remove newline char

                g_clientSendFlag = true; // Allow copying of string to send buffer
                /* TCP Client ASCII Message Format */
                UART_printf("%s,%3.2f,%3.2f,%d,%d,%d\n\r",
                               timeString,
                               outTempC,
                               outTempF,
                               outxVal,
                               outyVal,
                               outzVal);
                g_clientSendFlag = false; // Deny copying of string to send buffer
                //UART_printf("frameData = %s",PowerMeasure_CB.frameData);
            }
            break;
        case WIFI_tcli: // Test sending TCP Client message
            send_status = -1;
            UART_printf("Running TCP Client\n\r");

            prepareDataFrame(PowerMeasure_appData.port,
                             PowerMeasure_appData.ipAddr);

            send_status = bsdTcpClient(PowerMeasure_appData.port,
                                       PowerMeasure_appData.sockID);

            if (send_status == -1) {
                g_clientConnectFlag = false;
                UART_printf("ERROR TCP Client could not connect to server [sockID=%d]\n\r",
                            PowerMeasure_appData.sockID);
            } else if (send_status == 0) {
                g_clientConnectFlag = false;
                UART_printf("ERROR Server stopped responding [sockID=%d]\n\r",
                            PowerMeasure_appData.sockID);
                UART_printf("TCP Client timed out and closed socket [sockID=%d]\n\r",
                            PowerMeasure_appData.sockID);
                PowerMeasure_appData.sockID = OPEN_SOCK_ONCE;
            } else if (send_status == SERVER_FRAME_LENGTH) {
                g_clientConnectFlag = true;
                UART_printf("TCP Client sent message to server [sockID=%d]\n\r",
                            PowerMeasure_appData.sockID);
            }
            break;
        case WIFI_cmsg: // Test sending new client message
            UART_printf("Sending new client message\n\r");
            send_status = -1;
            g_message_number += 1; // Increment client message number

            prepareDataFrame(PowerMeasure_appData.port,
                             PowerMeasure_appData.ipAddr);

            ClientMsg_appData.msg_len = sl_Htonl(FRAME_LENGTH);         // Set message size
            ClientMsg_appData.msg_type = sl_Htons(1);                   // Set message type
            //ClientMsg_appData.msg_number = sl_Htonl(g_message_number);  // Set message number
            ClientMsg_appData.msg_number = g_message_number;  // Set message number

            send_status = bsdNewTcpClient(PowerMeasure_appData.port,
                                          PowerMeasure_appData.sockID);
            break;
        case WIFI_csid: // Close TCP Client socket ID
            if (PowerMeasure_appData.sockID != OPEN_SOCK_ONCE) {
                UART_printf("Closing TCP Client socket [sockID=%d]\n\r",
                            PowerMeasure_appData.sockID);
                status = sl_Close(PowerMeasure_appData.sockID);
                PowerMeasure_appData.sockID = OPEN_SOCK_ONCE;
                g_clientConnectFlag = false;
            } else {
                UART_printf("No TCP Client socket to close\r\n");
            }
            break;
        default:
            UART_printf("Unknown WIFI command\r\n");
            break;
    }
    status = 0;
    return status;
}

//*****************************************************************************
// SimpleLink Callback Functions (Retained As-Is)
//*****************************************************************************

void SimpleLinkNetAppRequestMemFreeEventHandler(uint8_t *buffer)
{
}

void SimpleLinkNetAppRequestEventHandler(SlNetAppRequest_t *pNetAppRequest,
                                         SlNetAppResponse_t *pNetAppResponse)
{
}

//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(pWlanEvent == NULL)
    {
        return;
    }

    switch(pWlanEvent->Id)
    {
    case SL_WLAN_EVENT_CONNECT:
        LOCALTIME_SET_STATUS_BIT(LocalTime_CB.status, STATUS_BIT_CONNECTION);

        UART_printf("STA Connected to AP: %s , BSSID: %x:%x:%x:%x:%x:%x\n\r",
                   pWlanEvent->Data.Connect.SsidName,
                   pWlanEvent->Data.Connect.Bssid[0],
                   pWlanEvent->Data.Connect.Bssid[1],
                   pWlanEvent->Data.Connect.Bssid[2],
                   pWlanEvent->Data.Connect.Bssid[3],
                   pWlanEvent->Data.Connect.Bssid[4],
                   pWlanEvent->Data.Connect.Bssid[5]);
        break;

    case SL_WLAN_EVENT_DISCONNECT:
        LOCALTIME_CLR_STATUS_BIT(LocalTime_CB.status, STATUS_BIT_CONNECTION);
        LOCALTIME_CLR_STATUS_BIT(LocalTime_CB.status, STATUS_BIT_IP_ACQUIRED);

        break;

    case SL_WLAN_EVENT_STA_ADDED:
        /* when device is in AP mode and any client connects to it.       */
        LOCALTIME_SET_STATUS_BIT(LocalTime_CB.status, STATUS_BIT_CONNECTION);
        break;

    case SL_WLAN_EVENT_STA_REMOVED:
        /* when device is in AP mode and any client disconnects from it.  */
        LOCALTIME_CLR_STATUS_BIT(LocalTime_CB.status, STATUS_BIT_CONNECTION);
        LOCALTIME_CLR_STATUS_BIT(LocalTime_CB.status, STATUS_BIT_IP_LEASED);
        break;

    default:
        break;
    }
}

//*****************************************************************************
//
//! \brief The Function Handles the Fatal errors
//!
//! \param[in]  slFatalErrorEvent - Pointer to Fatal Error Event info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkFatalErrorEventHandler(SlDeviceFatal_t *slFatalErrorEvent)
{
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(pNetAppEvent == NULL)
    {
        return;
    }

    switch(pNetAppEvent->Id)
    {
    case SL_NETAPP_EVENT_IPV4_ACQUIRED:
        LOCALTIME_SET_STATUS_BIT(LocalTime_CB.status, STATUS_BIT_IP_ACQUIRED);
        UART_printf("IPv4 acquired: IP = %d.%d.%d.%d\n\r",
                  (uint8_t)SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,3),
                  (uint8_t)SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,2),
                  (uint8_t)SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,1),
                  (uint8_t)SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,0));
        break;

    case SL_NETAPP_EVENT_DHCPV4_LEASED:
        LOCALTIME_SET_STATUS_BIT(LocalTime_CB.status, STATUS_BIT_IP_LEASED);
        break;

    case SL_NETAPP_EVENT_DHCPV4_RELEASED:
        LOCALTIME_CLR_STATUS_BIT(LocalTime_CB.status, STATUS_BIT_IP_LEASED);
        break;

    default:
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerEventHandler(
    SlNetAppHttpServerEvent_t *pHttpEvent,
    SlNetAppHttpServerResponse_t *
    pHttpResponse)
{
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
}

//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
}

//*****************************************************************************
//
//! \brief Display Application Banner
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
int32_t LocalTime_displayBanner(void)
{
    int32_t status = -1;
    uint8_t macAddress[SL_MAC_ADDR_LEN];
    uint16_t macAddressLen = SL_MAC_ADDR_LEN;
    uint16_t configSize = 0;
    uint8_t configOpt = SL_DEVICE_GENERAL_VERSION;
    SlDeviceVersion_t ver = {0};

    configSize = sizeof(SlDeviceVersion_t);
    status = sl_Start(0, 0, 0);
    if(status < 0)
    {
        return(-1);
    }

    /* Print device version info. */
    status =
        sl_DeviceGet(SL_DEVICE_GENERAL, &configOpt, &configSize,
                     (uint8_t*)(&ver));
    if(status < 0)
    {
        return(-1);
    }

    /* Print device Mac address */
    status = sl_NetCfgGet(SL_NETCFG_MAC_ADDRESS_GET, 0, &macAddressLen,
                          &macAddress[0]);
    if(status < 0)
    {
        return(-1);
    }

    UART_printf("\n\r");
    UART_printf("\t ==============================================\n\r");
    UART_printf("\t    %s Example Ver: %s\n\r",LOCALTIME_APPLICATION_NAME,
               LOCALTIME_APPLICATION_VERSION);
    UART_printf("\t ==============================================\n\r");
    UART_printf("\n\r");
    UART_printf("\t CHIP: 0x%x",ver.ChipId);
    UART_printf("\n\r");
    UART_printf("\t MAC:  %d.%d.%d.%d",ver.FwVersion[0],ver.FwVersion[1],
               ver.FwVersion[2],
               ver.FwVersion[3]);
    UART_printf("\n\r");
    UART_printf("\t PHY:  %d.%d.%d.%d",ver.PhyVersion[0],ver.PhyVersion[1],
               ver.PhyVersion[2],
               ver.PhyVersion[3]);
    UART_printf("\n\r");
    UART_printf("\t NWP:  %d.%d.%d.%d",ver.NwpVersion[0],ver.NwpVersion[1],
               ver.NwpVersion[2],
               ver.NwpVersion[3]);
    UART_printf("\n\r");
    UART_printf("\t ROM:  %d",ver.RomVersion);
    UART_printf("\n\r");
    UART_printf("\t HOST: %s", SL_DRIVER_VERSION);
    UART_printf("\n\r");
    UART_printf("\t MAC address: %02x:%02x:%02x:%02x:%02x:%02x", macAddress[0],
               macAddress[1], macAddress[2], macAddress[3], macAddress[4],
               macAddress[5]);
    UART_printf("\n\r");
    UART_printf("\n\r");
    UART_printf("\t ==============================================\n\r");
    UART_printf("\n\r");
    UART_printf("\n\r");
    status = sl_Stop(LOCALTIME_STOP_TIMEOUT);
    if(status < 0)
    {
        return(-1);
    }

    return(status);
}

//*****************************************************************************
//
//! \brief    Connect to WLAN AP
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
void LocalTime_connect(void)
{
    SlWlanSecParams_t secParams = {0};
    //char sel[2];
    uint8_t i = 0;

    UART_printf("Please wait...trying to connect to AP: %s\n\r",
                WifiSetup_CB.ssid);
    UART_printf("\n\r");
    /*
    secParams.Key = (signed char*)LOCALTIME_SECURITY_KEY;
    secParams.KeyLen = strlen(LOCALTIME_SECURITY_KEY);
    secParams.Type = LOCALTIME_SECURITY_TYPE;
    */
    secParams.Key = WifiSetup_CB.key;
    secParams.KeyLen = strlen((const char *)WifiSetup_CB.key);
    secParams.Type = WifiSetup_CB.security;
    /*
    sl_WlanConnect((signed char*)LOCALTIME_SSID_NAME,
                   strlen(LOCALTIME_SSID_NAME), 0, &secParams, 0);
    */
    sl_WlanConnect(WifiSetup_CB.ssid,
                   strlen((const char *)WifiSetup_CB.ssid),
                   0,
                   &secParams, 0);
    i = 0;
    while((!LOCALTIME_IS_IP_ACQUIRED(LocalTime_CB.status)) &&
          (i < LOCALTIME_IP_ACQUIRED_WAIT_SEC))
    {
        sleep(1);
        i++;
    }

    if(!LOCALTIME_IS_IP_ACQUIRED(LocalTime_CB.status))
    {
        UART_printf("Could not connect to AP: %s\n\r", WifiSetup_CB.ssid);
        UART_printf(
            "Please press enter to continue or reset the device after "
            "configuring your network parameters in wifi_config.h\n\r");
        //GetCmd(sel,sizeof(sel));
    }
}

//*****************************************************************************
//
//! \brief  This function prepares the Data Frame & initializes the
//!         IP address data struct            .
//!
//! \param  Port & IP address
//!
//! \return None
//
//*****************************************************************************
void prepareDataFrame(uint16_t port,uint32_t ipAddr)
{
    /*
    uint16_t idx;

    for(idx = 0;idx < FRAME_LENGTH;idx++)
    {
        PowerMeasure_CB.frameData[idx] = (signed char)(idx % 255);
    }
    */
    //strcpy((char *)PowerMeasure_CB.frameData, clientBuffer);
    PowerMeasure_CB.ipV4Addr.sin_family = SL_AF_INET;
    PowerMeasure_CB.ipV4Addr.sin_port = sl_Htons(port);
    PowerMeasure_CB.ipV4Addr.sin_addr.s_addr = sl_Htonl(ipAddr);
}

//*****************************************************************************
//
//! \brief This function implements the TCP client .
//!
//! \param Port - socket port number; Sid - socket id,
//!        -ve if socket is already opened.
//!
//! \return 0 on success, -ve otherwise.
//
//*****************************************************************************
int32_t bsdTcpClient(uint16_t port, int16_t sid)
{
    int16_t         sockId;
    int16_t         idx = 0;
    int16_t         status = -1;

    if (sid < 0) {
        /* Need to open socket  */
        sockId = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
        /* Make connection establishment */
        status = sl_Connect(sockId,
                            ( SlSockAddr_t *)&PowerMeasure_CB.ipV4Addr,
                            sizeof(SlSockAddrIn_t));
        /* Success: status = 0 */
        /* Fail: status = -1 */
        //UART_printf("status of sl_Connect = %d\n\r", status);
        if (status < 0) {
            sl_Close(sockId);
        }
    }
    else {
        /* Socket is already opened */
        sockId = sid;
    }

    while (idx < NUM_OF_PKT)
    {
        status = sl_Send(sockId,
                         PowerMeasure_CB.frameData,
                         FRAME_LENGTH,
                         0 );
        /* Success: status = number of bytes sent */
        /* Fail: status = -1 */
        //UART_printf("status of sl_Send = %d\n\r", status);
        if (status <= 0 ) {
            status = sl_Close(sockId);
            /* Success: status = 0 */
            /* Fail: status = -1 */
            //UART_printf("status of sl_Close = %d\n\r", status);
        }
        idx++;
    }
    if (sid == ALWAYS_OPEN_SOCK) {
        /* Next time, use a new socket */
        status = sl_Close(sockId);
    }
    else {
        /* store the current open socket id*/
        PowerMeasure_appData.sockID = sockId;
    }
    //return(0);
    return(status);
}

//*****************************************************************************
//
//! \brief This function implements the modified TCP client .
//!
//! \param Port - socket port number; Sid - socket id,
//!        -ve if socket is already opened.
//!
//! \return 0 on success, -ve otherwise.
//
//*****************************************************************************
int32_t bsdNewTcpClient(uint16_t port, int16_t sid)
{
    int16_t         sockId;
    int16_t         idx = 0;
    int16_t         status = -1;

    if (sid < 0) {
        /* Need to open socket  */
        sockId = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
        /* Make connection establishment */
        status = sl_Connect(sockId,
                            ( SlSockAddr_t *)&PowerMeasure_CB.ipV4Addr,
                            sizeof(SlSockAddrIn_t));
        /* Success: status = 0 */
        /* Fail: status = -1 */
        //UART_printf("status of sl_Connect = %d\n\r", status);
        if (status < 0) {
            sl_Close(sockId);
        }
    }
    else {
        /* Socket is already opened */
        sockId = sid;
    }

    while (idx < NUM_OF_PKT)
    {
        status = sl_Send(sockId,
                         &ClientMsg_appData,
                         sizeof(ClientMsg_ControlBlock),
                         0);
        /* Success: status = number of bytes sent */
        /* Fail: status = -1 */
        //UART_printf("status of sl_Send = %d\n\r", status);
        if (status <= 0 ) {
            status = sl_Close(sockId);
            /* Success: status = 0 */
            /* Fail: status = -1 */
            //UART_printf("status of sl_Close = %d\n\r", status);
        }
        idx++;
    }
    if (sid == ALWAYS_OPEN_SOCK) {
        /* Next time, use a new socket */
        status = sl_Close(sockId);
    }
    else {
        /* store the current open socket id*/
        PowerMeasure_appData.sockID = sockId;
    }
    //return(0);
    return(status);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
