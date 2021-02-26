#ifndef FREERTOS_I2C_SETUP_H_
#define FREERTOS_I2C_SETUP_H_

#define MAX_TEMPER_VALUES   PROJECT_TEMPER_VALUES
#define MAX_ACCEL_VALUES    PROJECT_ACCEL_VALUES

/* Number of target ticks that I2C Task blocks until */
#define I2CTASKTICKS        (1000)
/*
 *  ======== TMP Registers ========
 */
#define TMP006_REG          0x0001  /* Die Temp Result Register for TMP006 */
#define TMP_BP_REG          0x0000  /* Die Temp Result Register for BP TMP sensor */

/*
 *  The CC32XX LaunchPads come with an on-board TMP006 or TMP116 temperature
 *  sensor depending on the revision. Newer revisions come with the TMP116.
 *  The Build Automation Sensors (BP-BASSESENSORSMKII) BoosterPack
 *  contains a TMP117.
 *
 *  We are using the DIE temperature because it's cool!
 *
 *  Additionally: no calibration is being done on the TMPxxx device to simplify
 *  the example code.
 */
#define TMP006_ADDR         0x41;
#define TMP_BP_ADDR         0x48;
#define TMP116_LP_ADDR      0x49;

/*
 *  ======== HIGH_TEMP ========
 *  Send alert when this temperature (in Celsius) is exceeded
 */
#define HIGH_TEMP 30

/* Temperature written by the I2C task, and read by UART task */
extern volatile int16_t g_appTempC[MAX_TEMPER_VALUES];
extern volatile float g_temperatureC;
extern volatile float g_temperatureF;

/* Accelerometer values written by the I2C task, and read by UART task */
extern volatile int8_t g_appAccel[MAX_ACCEL_VALUES];
extern volatile int8_t g_xVal;
extern volatile int8_t g_yVal;
extern volatile int8_t g_zVal;

#endif /* FREERTOS_I2C_SETUP_H_ */
