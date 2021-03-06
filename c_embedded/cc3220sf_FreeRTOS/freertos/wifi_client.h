#ifndef WIFI_CLIENT_H_
#define WIFI_CLIENT_H_

#define DEST_IP_ADDR                SERVER_DEST_IP_ADDR
#define PORT                        SERVER_PORT
#define FRAME_LENGTH                SERVER_FRAME_LENGTH

#define DEFAULT_MESSAGE             "Message from CC3220SF"

#define NUM_OF_PKT                  (1)
#define ALWAYS_OPEN_SOCK            (-1)
#define OPEN_SOCK_ONCE              (-2)

#define UPTIME_WORDS                PROJECT_UPTIME_WORDS
#define STATUS_WORDS                PROJECT_STATUS_WORDS
#define TEMPER_VALUES               PROJECT_TEMPER_VALUES
#define ACCEL_VALUES                PROJECT_ACCEL_VALUES

typedef enum
{
    SocketType_UDP,
    SocketType_TCP,
    SocketType_SEC_TCP
}SocketTypes;

#define SOCKET_TYPE                 SocketType_TCP

typedef enum
{
    UseCase_Normal,
    UseCase_LSI,
    UseCase_IotLowPower
}AlwaysConnectedUseCases;

/* options-> UseCase_Normal, UseCase_LSI, UseCase_IotLowPower */
#define AC_USECASE                  UseCase_Normal

typedef enum
{
    UseCase_HIB,
    UseCase_LPDS,
    UseCase_Transceiver,
    UseCase_IntermittentlyConnected,
    UseCase_AlwaysConnected
}Power_UseCases;

/* options-> UseCase_HIB, UseCase_LPDS, UseCase_Transceiver,
UseCase_IntermittentlyConnected, UseCase_AlwaysConnected */
#define PM_USECASE                  UseCase_AlwaysConnected


typedef struct _PowerMeasure_AppData_t_
{   /* The exercised use case  */
    Power_UseCases                 useCase;
    /* The always connected use case  */
    AlwaysConnectedUseCases        alwaysConnectedUseCase;
    /* how many packet to transmit on each interval of this use case */
    uint32_t                       pktsToDo;
    /* socket ID*/
    int32_t                        sockID;
    /* Socket type */
    SocketTypes                    socketType;
    /* IP address */
    uint32_t                       ipAddr;
    /* socket port number */
    uint32_t                       port;
}PowerMeasure_AppData;

/* Control block definition */
typedef struct _PowerMeasure_ControlBlock_t_
{
    uint32_t        slStatus;    //SimpleLink Status
    mqd_t           queue;
    sem_t           sem;
    signed char     frameData[FRAME_LENGTH];
    SlSockAddrIn_t  ipV4Addr;
}PowerMeasure_ControlBlock;

/* Control block definition */
typedef struct _ClientHeader_ControlBlock_t_
{
    /* Number of bytes in the message body */
    uint32_t        msg_len;
    /* Type of message that the body contains, no padding */
    uint32_t        msg_type;
    /* Incremental message number */
    uint32_t        msg_number;
}ClientHeader_ControlBlock;

/* Control block definition */
typedef struct _ClientBody_ControlBlock_t_
{
    /* Text message (Type 1) */
    signed char     message[FRAME_LENGTH];
}ClientBody_ControlBlock;

/* Control block definition */
typedef struct _ClientMsg_ControlBlock_t_
{
    /* Standardized message header */
    ClientHeader_ControlBlock       header;
    /* Message body */
    ClientBody_ControlBlock         body;
}ClientMsg_ControlBlock;


/* Type 2 message body control block definition */
typedef struct _T2Msg_ControlBlock_t_
{
    /* Numeric message (Type 2) */
    uint32_t        var_a;
    uint32_t        var_b;
    uint32_t        var_c;
    uint32_t        var_d;
    float           var_e;
    float           var_f;
    float           var_g;
    float           var_h;
}T2Msg_ControlBlock;

/* Full Type 2 message control block definition */
typedef struct _ClientT2Msg_ControlBlock_t_
{
    /* Standardized message header */
    ClientHeader_ControlBlock       header;
    /* Message body */
    T2Msg_ControlBlock              body;
}ClientT2Msg_ControlBlock;


/* Omnibus message body control block definition */
typedef struct _OmnibusMsg_Body_t_
{
    /* Omnibus message */
    uint32_t        uptime[UPTIME_WORDS];       // RTC (seconds)
    uint32_t        datetime;                   // Date and time since epoch
    uint32_t        var_a;                      // (Milliseconds)
    uint32_t        status[STATUS_WORDS];       //
    uint32_t        var_b;                      //
    int16_t         temp_c[TEMPER_VALUES];      // (Celsius)
    int8_t          accel[ACCEL_VALUES];        //
    uint32_t        var_c;                      //
    uint32_t        var_d;                      //
    uint32_t        var_e;                      //
    uint32_t        var_f;                      //
    uint32_t        var_g;                      //
    uint32_t        var_h;                      //
    uint32_t        var_i;                      //
}OmnibusMsg_Body;

/* Full Omnibus message control block definition */
typedef struct _OmnibusMsg_Block_t_
{
    /* Standardized message header */
    ClientHeader_ControlBlock       header;
    /* Message body */
    OmnibusMsg_Body                 body;
}OmnibusMsg_Block;

//****************************************************************************
//                          FUNCTION PROTOTYPES
//****************************************************************************
void prepareDataFrame(uint16_t port,uint32_t ipAddr);
int32_t bsdTcpClient(uint16_t port, int16_t sid);
int32_t bsdCustomTcpClient(uint16_t port, int16_t sid, int16_t msg_type);

#endif /* WIFI_CLIENT_H_ */
