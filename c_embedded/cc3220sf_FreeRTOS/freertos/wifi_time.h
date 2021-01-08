#ifndef WIFI_SETUP_H_
#define WIFI_SETUP_H_

//*****************************************************************************
//                          DEFINES
//*****************************************************************************
#define AUTO_RECONNECT                                  (0)

/* WIFI name or AP SSID */
#define LOCALTIME_SSID_NAME                             DEFAULT_WIFI_SSID_NAME //
/* Security type */
#define LOCALTIME_SECURITY_TYPE                         DEFAULT_WIFI_SECURITY_TYPE //
/* Password for WIFI or the secured AP */
#define LOCALTIME_SECURITY_KEY                          DEFAULT_WIFI_SECURITY_KEY //

#define LOCALTIME_APPLICATION_NAME                      "RMS WIFI"
#define LOCALTIME_APPLICATION_VERSION                   "1.0.0"

#define LOCALTIME_SLNET_IF_WIFI_PRIO                    (5)
#define LOCALTIME_SPAWN_STACK_SIZE                      (4096)
#define LOCALTIME_STOP_TIMEOUT                          (200)
#define LOCALTIME_IP_ACQUIRED_WAIT_SEC                  (6)
#define LOCALTIME_SPAWN_TASK_PRIORITY                   (9)
#define LOCALTIME_CLR_STATUS_BIT_ALL(status_variable)   (status_variable = 0)
#define LOCALTIME_SET_STATUS_BIT(status_variable, bit)  (status_variable |= \
                                                             (1 << (bit)))
#define LOCALTIME_CLR_STATUS_BIT(status_variable, bit)  (status_variable &= \
                                                             ~(1 << (bit)))
#define LOCALTIME_GET_STATUS_BIT(status_variable, bit)  (0 != \
                                                         (status_variable & \
                                                          (1 << (bit))))

#define LOCALTIME_IS_CONNECTED(status_variable) \
    LOCALTIME_GET_STATUS_BIT(status_variable,STATUS_BIT_CONNECTION)
#define LOCALTIME_IS_IP_ACQUIRED(status_variable) \
    LOCALTIME_GET_STATUS_BIT(status_variable,STATUS_BIT_IP_ACQUIRED)

#define LOCALTIME_ASSERT_ON_ERROR(error_code) \
    { \
        if(error_code < 0) \
        { \
            UART_printf("error %d\r\n",error_code); \
            return error_code; \
        } \
    }

//*****************************************************************************
//                          TYPEDEFS
//*****************************************************************************
// Status bits - These are used to set/reset the corresponding bits in
// given variable

typedef enum _LocalTime_Status_e_
{
    STATUS_BIT_NWP_INIT = 0,      // If this bit is set: Network Processor is
                                  // powered up
    STATUS_BIT_CONNECTION,        // If this bit is set: the device is
                                  //connected to
                                  // the AP or client is connected to device
                                  //(AP)
    STATUS_BIT_IP_LEASED,         // If this bit is set: the device has leased
                                  //    IP to
                                  // any connected client
    STATUS_BIT_IP_ACQUIRED,       // If this bit is set:
                                  //the device has acquired an IP
    STATUS_BIT_SMARTCONFIG_START, // If this bit is set: the SmartConfiguration
                                  // process is started from SmartConfig app
    STATUS_BIT_P2P_DEV_FOUND,     // If this bit is set: the device (P2P mode)
                                  // found any p2p-device in scan
    STATUS_BIT_P2P_REQ_RECEIVED,  // If this bit is set: the device (P2P mode)
                                  // found any p2p-negotiation request
    STATUS_BIT_CONNECTION_FAILED, // If this bit is set: the device(P2P mode)
                                  // connection to client(or reverse way)
                                  // is failed
    STATUS_BIT_PING_DONE,         // If this bit is set: the device has
                                  // completed the ping operation
    STATUS_BIT_IPV6L_ACQUIRED,    // If this bit is set:
                                  //the device has acquired an IPv6 address
    STATUS_BIT_IPV6G_ACQUIRED,    // If this bit is set:
                                  //the device has acquired an IPv6 address
    STATUS_BIT_AUTHENTICATION_FAILED,
    STATUS_BIT_RESET_REQUIRED,
}LocalTime_Status_e;

typedef enum
{
    LocalTime_GetTime,
    LocalTime_UpdateTime,
    LocalTime_SetTimezone,
    LocalTime_GetTimezone,
    LocalTime_SwitchAP,
    LocalTime_SwitchStation
}LocalTime_UseCases;

/* Control block definition */
typedef struct _LocalTime_ControlBlock_t_
{
    uint32_t status;                // SimpleLink Status
    LocalTime_UseCases useCase;     // LocalTime Use Case
}LocalTime_ControlBlock;

/* WIFI configuration settings control block definition */
typedef struct _WifiSetup_ControlBlock_t_
{
    signed char     ssid[MAX_SSID_LENGTH];  // WIFI name
    signed char     key[MAX_SSID_LENGTH];   // WIFI password
    unsigned char   security;               // WIFI security type
}WifiSetup_ControlBlock;

//****************************************************************************
//                          FUNCTION PROTOTYPES
//****************************************************************************
void SimpleLinkNetAppRequestMemFreeEventHandler(uint8_t *buffer);
void SimpleLinkNetAppRequestEventHandler(SlNetAppRequest_t *pNetAppRequest,
                                         SlNetAppResponse_t *pNetAppResponse);
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent);
void SimpleLinkFatalErrorEventHandler(SlDeviceFatal_t *slFatalErrorEvent);
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent);
void SimpleLinkHttpServerEventHandler(
    SlNetAppHttpServerEvent_t *pHttpEvent,
    SlNetAppHttpServerResponse_t *
    pHttpResponse);
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent);
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock);
int32_t LocalTime_displayBanner(void);
void LocalTime_connect(void);

#endif /* WIFI_SETUP_H_ */
