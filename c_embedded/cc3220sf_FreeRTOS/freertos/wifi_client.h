#ifndef WIFI_CLIENT_H_
#define WIFI_CLIENT_H_

#define DEST_IP_ADDR                SERVER_DEST_IP_ADDR
#define PORT                        SERVER_PORT
#define FRAME_LENGTH                SERVER_FRAME_LENGTH

#define DEFAULT_MESSAGE             "Message from CC3220SF"

#define NUM_OF_PKT                  (1)
#define ALWAYS_OPEN_SOCK            (-1)
#define OPEN_SOCK_ONCE              (-2)

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


//****************************************************************************
//                          FUNCTION PROTOTYPES
//****************************************************************************
void prepareDataFrame(uint16_t port,uint32_t ipAddr);
int32_t bsdTcpClient(uint16_t port, int16_t sid);
int32_t bsdNewTcpClient(uint16_t port, int16_t sid);

#endif /* WIFI_CLIENT_H_ */
