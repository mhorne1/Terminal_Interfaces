#ifndef PROJECT_COMMON_H
#define PROJECT_COMMON_H

/* Stack size in 32-bit words */
#define UARTTASKSTACKSIZE       2048 / sizeof(portSTACK_TYPE)
#define I2CTASKSTACKSIZE        1024 / sizeof(portSTACK_TYPE)
#define WIFITASKSTACKSIZE       2048 / sizeof(portSTACK_TYPE)

#define PROJECT_QUEUE_LENGTH    (3)
#define PROJECT_BF_FLAGS        (0)

#define PROJECT_UPTIME_WORDS    (2)
#define PROJECT_STATUS_WORDS    (2)
#define PROJECT_TEMPER_VALUES   (2)
#define PROJECT_ACCEL_VALUES    (4)

typedef enum
{
    TPRI_uart = 1,              // Uart task priority
    TPRI_wifi,                  // Wifi task priority
    TPRI_i2c,                   // I2C task priority
    TPRI_final,                 // Final task priority
}TaskPriorityUseCases;

typedef struct _Task_Handles_t_
{
    TaskHandle_t uartHandle;    // Uart task handle
    TaskHandle_t i2cHandle;     // I2C task handle
    TaskHandle_t wifiHandle;    // Wifi task handle
}Task_Handles_Block;

typedef struct _Q_Data_t_
{
    unsigned int header;    // header command
    int param;              // parameter value
    char *pcbuf;            // char buffer pointer
}Q_Data_Block;


#ifdef PROJECT_BF_FLAGS
    typedef struct _Task_Flags_t_
    {                               // bits description
        uint16_t UART_CFG : 1;      // 0 UART configured
        uint16_t I2C_CFG : 1;       // 1 I2C configured
        uint16_t I2C_NEW : 1;       // 2 New data available
        uint16_t WIFI_CONN : 1;     // 3 WIFI connected
        uint16_t WIFI_TIME : 1;     // 4 Time acquired
        uint16_t WIFI_TCP : 1;      // 5 TCP Client connected to server
        uint16_t FLAG06 : 1;        // 6
        uint16_t FLAG07 : 1;        // 7
        uint16_t FLAG08 : 1;        // 8
        uint16_t FLAG09 : 1;        // 9
        uint16_t FLAG10 : 1;        // 10
        uint16_t FLAG11 : 1;        // 11
        uint16_t FLAG12 : 1;        // 12
        uint16_t FLAG13 : 1;        // 13
        uint16_t FLAG14 : 1;        // 14
        uint16_t FLAG15 : 1;        // 15
    }Task_Flags_Block;
#endif

extern Task_Handles_Block xTasks;
extern Q_Data_Block qMsgSend;
extern Q_Data_Block qMsgRecv;
extern QueueHandle_t xQueue1;
extern volatile char g_i2cAutoSend;
#ifdef PROJECT_BF_FLAGS
    extern Task_Flags_Block projectFlags;
#endif

extern int UART_printf(const char *pcFormat, ...);

#endif  /* PROJECT_COMMON_H */
