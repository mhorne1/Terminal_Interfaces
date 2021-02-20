#ifndef WIFI_CONFIG_H_
#define WIFI_CONFIG_H_

/* Default WIFI name or AP SSID */
#define DEFAULT_WIFI_SSID_NAME                             "Enter WIFI name here"
/* Default security type, typically either:
 * SL_WLAN_SEC_TYPE_OPEN or SL_WLAN_SEC_TYPE_WPA_WPA2 */
#define DEFAULT_WIFI_SECURITY_TYPE                         SL_WLAN_SEC_TYPE_WPA_WPA2
/* Default password for WIFI or the secured AP */
#define DEFAULT_WIFI_SECURITY_KEY                          "Enter WIFI password here"

/* Maximum length of WIFI name or password */
#ifndef MAX_SSID_LENGTH
#define MAX_SSID_LENGTH                                     (32)
#endif /* MAX_SSID_LENGTH */

/* Default IP Address of desktop computer running server (Python script or Iperf) */
#define SERVER_DEST_IP_ADDR                                 SL_IPV4_VAL(192,168,2,250)
/* Default port for server (5001 for Iperf server) */
#define SERVER_PORT                                         (5001)
/* Maximum transmission buffer size used by both TCP Client and server */
#define SERVER_FRAME_LENGTH                                 (80)

/* Number of ticks that WIFI Task waits for message from WIFI command queue */
#define WIFITASKTICKS                                       (100)

typedef enum
{                   // WIFI commands for queue (and WIFI activity requirements)
    WIFI_dset = 1,  // Display WIFI settings
    WIFI_cset,      // Change WIFI settings
    WIFI_conn,      // Connect to WIFI
    WIFI_cmsg,      // Send client message to server, requires WIFI connection
    WIFI_rmsg,      // Receive message from server, requires WIFI connection
    WIFI_omsg,      // Send omnibus message to TCP Server, requires WIFI connection
    WIFI_utim,      // Update device time, requires WIFI connection, redundant with ClockSync_get()
    WIFI_dtim,      // Display device time (UTC), requires WIFI connection
    WIFI_dmsg,      // Display TCP Client message, requires WIFI connection
    WIFI_tcli,      // Test TCP Client, requires WIFI connection and time
    WIFI_csid,      // Close TCP Client socket ID, requires WIFI connection and time
    WIFI_smsg,      // Send TCP Client message, requires WIFI connection and time
}WIFICommandUseCases;

typedef enum
{
    WIFI_cfg_ssid = 1,  // Access WIFI name
    WIFI_cfg_key,       // Access WIFI password
    WIFI_cfg_open,      // Select SL_WLAN_SEC_TYPE_OPEN
    WIFI_cfg_wpa1,      // Select SL_WLAN_SEC_TYPE_WPA
    WIFI_cfg_wpa2,      // Select SL_WLAN_SEC_TYPE_WPA_WPA2
    WIFI_cfg_wpa3,      // Select SL_WLAN_SEC_TYPE_WPA3
}WIFIConfigUseCases;

typedef enum
{
    WIFI_msg_type1 = 1,     // Text
    WIFI_msg_type2,         // Numeric
    WIFI_msg_type3,         // Acknowledge
    WIFI_msg_omnibus = 50,  // System status and measurements
    WIFI_msg_last,          // Unused
}WIFIMessageTypes;

extern signed char *g_pClientSendBuffer;
extern bool g_clientSendFlag;
extern bool g_wifiConnectFlag;
extern bool g_timeAcquiredFlag;
extern bool g_clientConnectFlag;

#endif /* WIFI_CONFIG_H_ */
