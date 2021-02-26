#ifndef SYSTEM_STATUS_H_
#define SYSTEM_STATUS_H_

#define MAX_STATUS_WORDS    PROJECT_STATUS_WORDS

typedef enum {
    STATUS_UART_INIT = 0,   //
    STATUS_I2C_INIT,        //
    STATUS_WIFI_INIT,       //
    STATUS_WIFI_CONNECT,    //
    STATUS_TIME_SET,        //
    STATUS_TCP_CONNECT,     //
    STATUS_BIT_06,      //
    STATUS_BIT_07,      //
    STATUS_BIT_08,      //
    STATUS_BIT_09,      //
    STATUS_BIT_10,      //
    STATUS_BIT_11,      //
    STATUS_BIT_12,      //
    STATUS_BIT_13,      //
    STATUS_BIT_14,      //
    STATUS_BIT_15,      //
    STATUS_BIT_16,      //
    STATUS_BIT_17,      //
    STATUS_BIT_18,      //
    STATUS_BIT_19,      //
    STATUS_BIT_20,      //
    STATUS_BIT_21,      //
    STATUS_BIT_22,      //
    STATUS_BIT_23,      //
    STATUS_BIT_24,      //
    STATUS_BIT_25,      //
    STATUS_BIT_26,      //
    STATUS_BIT_27,      //
    STATUS_BIT_28,      //
    STATUS_BIT_29,      //
    STATUS_BIT_30,      //
    STATUS_BIT_31,      //
    STATUS_BIT_32,      //
    STATUS_BIT_33,      //
    STATUS_BIT_34,      //
    STATUS_BIT_35,      //
    STATUS_BIT_36,      //
    STATUS_BIT_37,      //
    STATUS_BIT_38,      //
    STATUS_BIT_39,      //
    STATUS_BIT_40,      //
    STATUS_BIT_41,      //
    STATUS_BIT_42,      //
    STATUS_BIT_43,      //
    STATUS_BIT_44,      //
    STATUS_BIT_45,      //
    STATUS_BIT_46,      //
    STATUS_BIT_47,      //
    STATUS_BIT_48,      //
    STATUS_BIT_49,      //
    STATUS_BIT_50,      //
    STATUS_BIT_51,      //
    STATUS_BIT_52,      //
    STATUS_BIT_53,      //
    STATUS_BIT_54,      //
    STATUS_BIT_55,      //
    STATUS_BIT_56,      //
    STATUS_BIT_57,      //
    STATUS_BIT_58,      //
    STATUS_BIT_59,      //
    STATUS_BIT_60,      //
    STATUS_BIT_61,      //
    STATUS_BIT_62,      //
    STATUS_BIT_63,      //
    STATUS_BIT_LAST,    //
}STATUSBITS;

// Application Status Word Array
extern uint32_t g_appStatus[MAX_STATUS_WORDS];

//****************************************************************************
//                          FUNCTION PROTOTYPES
//****************************************************************************
int8_t setStatus(uint32_t *arr, uint16_t index, uint8_t set);

#endif /* SYSTEM_STATUS_ */
