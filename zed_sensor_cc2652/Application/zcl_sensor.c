/**************************************************************************************************
  Filename:       zcl_sampletemperaturesensor.c
  Revised:        $Date: 2014-10-24 16:04:46 -0700 (Fri, 24 Oct 2014) $
  Revision:       $Revision: 40796 $

  Description:    Zigbee Cluster Library - sample device application.


  Copyright 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
  This application implements a ZigBee Temperature Sensor, based on Z-Stack 3.0.

  This application is based on the common sample-application user interface. Please see the main
  comment in zcl_sampleapp_ui.c. The rest of this comment describes only the content specific for
  this sample application.

  Application-specific UI peripherals being used:

  - LEDs:
    LED1 is not used in this application

  Application-specific menu system:

    <SET LOCAL TEMP> Set the temperature of the local temperature sensor
      Up/Down changes the temperature
      This screen shows the following information:
        Line2:
          Shows the temperature of the local temperature sensor

*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <Application/zcl_sensor.h>
#include "rom_jt_154.h"
#include "zcomdef.h"
#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ms.h"
#include <string.h>

#include "bdb_interface.h"
#include "bdb_reporting.h"
#include "ti_drivers_config.h"

#include <ti/drivers/apps/Button.h>
#include <ti/drivers/apps/LED.h>

#ifndef CUI_DISABLE
#include "zcl_sampleapps_ui.h"
#include "zcl_sample_app_def.h"
#endif

#include "nvintf.h"
#include "zstackmsg.h"
#include "zcl_port.h"

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include "zstackapi.h"
#include "util_timer.h"
#include "mac_util.h"

#if !defined (DISABLE_GREENPOWER_BASIC_PROXY) && (ZG_BUILD_RTR_TYPE)
#include "gp_common.h"
#endif

#if defined ( BDB_TL_INITIATOR )
#include "touchlink_initiator_app.h"
#elif defined ( BDB_TL_TARGET )
#include "touchlink_target_app.h"
#endif

#if defined(SENSOR_AHT10)
#include <Application/sensor_controller/AHT10/scif.h>
#elif defined(SENSOR_SI7021)
#include <Application/sensor_controller/SI7021/scif.h>
#elif defined(SENSOR_LTR390)
#include <Application/sensor_controller/LTR390/scif.h>
#elif defined(SENSOR_APDS9930)
#include <Application/sensor_controller/ADPS9930/scif.h>
#endif

/* Driver Header files */
#include <ti/drivers/ADC.h>
/* Driver configuration */
#include "ti_drivers_config.h"

#include "math.h"

/*********************************************************************
 * MACROS
 */
#define GUI_LOCAL_TEMP    1

#define APP_TITLE "   Temp Sensor  "

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

#ifdef SENSOR_AHT10
int32_t prev_temp = 0;
int32_t prev_hum = 0;
SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_OUTPUT_T output;
#elif defined(SENSOR_SI7021)
int32_t prev_temp = 0;
int32_t prev_hum = 0;
SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_OUTPUT_T output;
#elif defined(SENSOR_LTR390) || defined(SENSOR_APDS9930)
int32_t prev_lux = 0;
SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_OUTPUT_T output;
#endif

// Semaphore used to post events to the sensor application thread
static Semaphore_Handle appSensorSemHandle;
static Semaphore_Struct appSensorSem;

/* ADC conversion result variables */
uint16_t adcValue0;
uint32_t adcValue0MicroVolt;
uint16_t batt = 0;
uint16_t batt_prev = 0;

void scTaskAlertCallback(void);

LED_Handle gGreenLedHandle;

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static bool onNwk = false;

#ifdef BDB_REPORTING
#if BDBREPORTING_MAX_ANALOG_ATTR_SIZE == 8
  uint8_t reportableChange[] = {0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x2C01 is 300 in int16_t
#endif
#if BDBREPORTING_MAX_ANALOG_ATTR_SIZE == 4
//  uint8_t reportableChange[] = {0x2C, 0x01, 0x00, 0x00}; // 0x2C01 is 300 in int16_t
  uint8_t reportableChange[] = {0x32, 0x00, 0x00, 0x00}; // 50 in int16_t
#endif
#if BDBREPORTING_MAX_ANALOG_ATTR_SIZE == 2
  uint8_t reportableChange[] = {0x2C, 0x01}; // 0x2C01 is 300 in int16_t
#endif
#endif


// Semaphore used to post events to the application thread
static Semaphore_Handle appSemHandle;
static Semaphore_Struct appSem;

/* App service ID used for messaging with stack service task */
static uint8_t  appServiceTaskId;
/* App service task events, set by the stack service task when sending a message */
static uint32_t appServiceTaskEvents;
static endPointDesc_t  zclSensorEpDesc = {0};

#if ZG_BUILD_ENDDEVICE_TYPE
static Clock_Handle EndDeviceRejoinClkHandle;
static Clock_Struct EndDeviceRejoinClkStruct;
#endif

//#ifndef CUI_DISABLE
static uint16_t zclSensor_BdbCommissioningModes;
//#endif

// Passed in function pointers to the NV driver
static NVINTF_nvFuncts_t *pfnZdlNV = NULL;

#ifndef CUI_DISABLE
CONST char zclSampleTemperatureSensor_appStr[] = APP_TITLE_STR;
CUI_clientHandle_t gCuiHandle;
static uint32_t gSampleTemperatureSensorInfoLine;
#endif
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void zclSensor_Init( void );
static void zclSensor_initialization(void);
static void zclSensor_process_loop(void);
static void zclSensor_initParameters(void);
static void zclSensor_ProcessCommissioningStatus(bdbCommissioningModeMsg_t *bdbCommissioningModeMsg);

static void zclSensor_initializeClocks(void);
static void zclSensor_processZStackMsgs(zstackmsg_genericReq_t *pMsg);

static void zclSensor_processAfIncomingMsgInd(zstack_afIncomingMsgInd_t *pInMsg);
#if ZG_BUILD_ENDDEVICE_TYPE
static void zclSensor_processEndDeviceRejoinTimeoutCallback(UArg a0);
#endif

static void zclSensor_processKey(uint8_t key, Button_EventMask buttonEvents, uint32_t duration);
#ifndef CUI_DISABLE
static void zclSensor_RemoveAppNvmData(void);
static void zclSampleTemperatureSensor_InitializeStatusLine(CUI_clientHandle_t gCuiHandle);
void zclSampleTemperatureSensor_UpdateStatusLine(void);
#endif

static void zclSensor_BasicResetCB( void );
//static void zclSampleTemperatureSensor_ProcessCommissioningStatus(bdbCommissioningModeMsg_t* bdbCommissioningModeMsg);

// Functions to process ZCL Foundation incoming Command/Response messages
static uint8_t zclSensor_ProcessIncomingMsg( zclIncoming_t *msg );
#ifdef ZCL_READ
static uint8_t zclSensor_ProcessInReadRspCmd( zclIncoming_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8_t zclSensor_ProcessInWriteRspCmd( zclIncoming_t *pInMsg );
#endif
static uint8_t zclSensor_ProcessInDefaultRspCmd( zclIncoming_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8_t zclSensor_ProcessInDiscCmdsRspCmd( zclIncoming_t *pInMsg );
static uint8_t zclSensor_ProcessInDiscAttrsRspCmd( zclIncoming_t *pInMsg );
static uint8_t zclSensor_ProcessInDiscAttrsExtRspCmd( zclIncoming_t *pInMsg );
#endif // ZCL_DISCOVER


/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclSensor_CmdCallbacks =
{
  zclSensor_BasicResetCB,        // Basic Cluster Reset command
  NULL,                                           // Identfiy cmd
  NULL,                                           // Identify Query command
  NULL,                                           // Identify Query Response command
  NULL,                                           // Identify Trigger Effect command
#ifdef ZCL_ON_OFF
  NULL,             				                      // On/Off cluster command
  NULL,                                           // On/Off cluster enhanced command Off with Effect
  NULL,                                           // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                           // On/Off cluster enhanced command On with Timed Off
#endif
#ifdef ZCL_LEVEL_CTRL
  NULL,                                           // Level Control Move to Level command
  NULL,                                           // Level Control Move command
  NULL,                                           // Level Control Step command
  NULL,                                           // Level Control Stop command
  NULL,                                           // Level Control Move to Closest Frequency command
#endif
#ifdef ZCL_GROUPS
  NULL,                                           // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                           // Scene Store Request command
  NULL,                                           // Scene Recall Request command
  NULL,                                           // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                           // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                           // Get Event Log command
  NULL,                                           // Publish Event Log command
#endif
  NULL,                                           // RSSI Location command
  NULL                                            // RSSI Location Response command
};


/*******************************************************************************
 * @fn          sampleApp_task
 *
 * @brief       Application task entry point for the Z-Stack
 *              Sample Application
 *
 * @param       pfnNV - pointer to the NV functions
 *
 * @return      none
 */
void sampleApp_task(NVINTF_nvFuncts_t *pfnNV)
{
  // Save and register the function pointers to the NV drivers
  pfnZdlNV = pfnNV;
  zclport_registerNV(pfnZdlNV, ZCL_PORT_SCENE_TABLE_NV_ID);

  // Initialize application
  zclSensor_initialization();

  // No return from task process
  zclSensor_process_loop();
}


//////////// add sensor task

/*******************************************************************************
 * @fn          sensorApp_task
 *
 * @brief       Application task entry point for the processing
 *              the sensor controller data
 *
 * @param       none
 *
 * @return      none
 */
uint32_t SensorControllerTickSecs = 15;  // the sensor controller will generate an interrupt every x seconds
int16_t BatteryCheckTicks = 4; // only check the battery level every x interrupts

void sensorApp_task()
{
    uint32_t rtcTicks = SensorControllerTickSecs << 16;
    int16_t battery_check = SensorControllerTickSecs * BatteryCheckTicks;
    int16_t battery_loop = battery_check;


#if defined(SENSOR_AHT10) || defined(SENSOR_SI7021) || defined(SENSOR_LTR390) || defined(SENSOR_APDS9930)
    // Initialize the SCIF operating system abstraction layer
    scifOsalInit();
//    scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
    scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);

    // Initialize the SCIF driver
    scifInit(&scifDriverSetup);
    // Enable RTC ticks, with N Hz tick interval
    scifStartRtcTicksNow(rtcTicks);

#endif

    /* create semaphores for messages / events
     */
    Semaphore_Params semSensorParam;
    Semaphore_Params_init(&semSensorParam);
    semSensorParam.mode = ti_sysbios_knl_Semaphore_Mode_BINARY;
    Semaphore_construct(&appSensorSem, 0, &semSensorParam);
    appSensorSemHandle = Semaphore_handle(&appSensorSem);

    ADC_Handle   adc;
    ADC_Params   params;
    int_fast16_t res;

    ADC_init();

#ifdef SENSOR_AHT10
    // Start the "I2C Temp and Humidity Sensor" Sensor Controller task
    scifStartTasksNbl(1 << SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_TASK_ID);
#elif defined(SENSOR_SI7021)
    scifStartTasksNbl(1 << SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_TASK_ID);
#elif defined(SENSOR_LTR390)
    scifStartTasksNbl(1 << SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_TASK_ID);
#elif defined(SENSOR_APDS9930)
    scifStartTasksNbl(1 << SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_TASK_ID);
#endif


    /* Forever loop */
    for(;;)
    {
        /* Wait for response message */
        if(Semaphore_pend(appSensorSemHandle, BIOS_WAIT_FOREVER ))
        {
            battery_loop++;
#ifdef SENSOR_AHT10
            if (output.status == 25) {
                int32_t calc;
                calc = (output.byte1 << 8) | (output.byte2); // | (output.byte3 >> 4);
                calc = (calc * 10000) >> 16;
                if (prev_hum != (int16_t)calc) {
                    zclHumiditySensor_MeasuredValue = (int16_t)(calc);
                    prev_hum = (int16_t)calc;
                }

//                calc = ((output.byte3 & 0x0F) << 8) | (output.byte4); // | (output.byte5);
//                calc = ((calc * 20000) >> 12) - 5000;
                calc = (((output.byte3 & 0x0F) << 16) | ((output.byte4) << 8) | output.byte5);
                calc = ((((calc) >> 10) * 20000) >> 10) - 5000;
                if (prev_temp != (int16_t)calc) {
                    zclTemperatureSensor_MeasuredValue = (int16_t)(calc);
                    prev_temp = (int16_t)calc;
                }
            } else {
               // no valid data
            }
#elif defined(SENSOR_SI7021)
            uint32_t calc;
            calc = (((uint16_t)output.hum * 12500) >> 16) - 600;
            if (prev_hum != calc) {
                zclHumiditySensor_MeasuredValue = calc;
                prev_hum = calc;
            }
            calc = (((uint16_t)output.temp * 17672) >> 16) - 4785;
            if (prev_temp != calc) {
                zclTemperatureSensor_MeasuredValue = calc;
                prev_temp = calc;
            }
#elif defined(SENSOR_LTR390)
            uint32_t calc;
            calc = ((output.byte3 & 0x0F) << 16) | (output.byte2 << 8) | output.byte1; // | (output.byte3 >> 4);
            calc /=  5;
            calc = log10(calc) * 10000;
            zclIlluminanceSensor_MeasuredValue = calc;

#elif defined(SENSOR_APDS9930)
            uint32_t calc1;
            uint32_t calc2;
            uint32_t iac;
            uint32_t lpc;
            calc1 = (output.ch0data << 10) - 1907 * (output.ch1data);
            calc2 = 764 * (output.ch0data) - 1322 * (output.ch1data);
            iac = (calc1 > calc2 ? calc1 : calc2) >> 10;
            lpc = 9594;
            zclIlluminanceSensor_MeasuredValue = log10( (iac * lpc) >> 16) * 10000;
#endif

            // just report the raw battery values for testing
            if (battery_loop > battery_check) {
                battery_loop = 0;
                ADC_Params_init(&params);
                adc = ADC_open(CONFIG_ADC_0, &params);

                if (adc == NULL) {
                    continue;
                }

                /* Blocking mode conversion */
                res = ADC_convert(adc, &adcValue0);

                if (res == ADC_STATUS_SUCCESS) {

                    adcValue0MicroVolt = ADC_convertRawToMicroVolts(adc, adcValue0);
                    batt = (uint16_t) (adcValue0MicroVolt/1000);
                    if (batt != batt_prev) {
                        batt_prev = batt;
                        if (batt >= 3000) {
                            zclBatterySensor_MeasuredValue = (uint8_t) (30);
                            zclBatteryPercentSensor_MeasuredValue = (uint8_t) 200;
                        } else {
                            zclBatterySensor_MeasuredValue = (uint8_t) (batt / 100);
                            zclBatteryPercentSensor_MeasuredValue = (uint8_t)( 2 * batt  / 30);
                        }


                        // Build and send a ZCL temperature reading to the matched device
                        zclReportCmd_t *pReportCmd;
                        pReportCmd = malloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
                        if(pReportCmd != NULL)
                        {
                            // Fill in the single attribute information for the temperature reading
                            pReportCmd->numAttr = 1;
                            pReportCmd->attrList[0].attrID = ATTRID_POWER_CONFIGURATION_BATTERY_PERCENTAGE_REMAINING;
                            pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT8;
                            pReportCmd->attrList[0].attrData = (void *)&zclBatteryPercentSensor_MeasuredValue;// (uint8 *)&(zclSampleTemperatureSensor_MeasuredValue);

                            afAddrType_t dstaddr;
                            dstaddr.addr.shortAddr = 0x0000;
                            dstaddr.addrMode = afAddr16Bit;
                            dstaddr.endPoint = 1;
                            dstaddr.panId = 0;

                            // Call ZCL function to send the report
                            zcl_SendReportCmd(8, &dstaddr, ZCL_CLUSTER_ID_GENERAL_POWER_CFG, pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, 1, 0 );

                            free(pReportCmd);
                        }
                    }
                }

                ADC_close(adc);
            }
        }
    }
}


// SCIF driver callback: Sensor Controller task code has generated an alert interrupt
void scTaskAlertCallback(void) {

#ifdef SENSOR_AHT10
    scifClearAlertIntSource();
    output = scifTaskData.i2cTempAndHumiditySensor.output;
    Semaphore_post(appSensorSemHandle);

    scifAckAlertEvents();
#elif defined(SENSOR_SI7021)
    scifClearAlertIntSource();
    output = scifTaskData.i2cTempAndHumiditySensor.output;
    Semaphore_post(appSensorSemHandle);

    scifAckAlertEvents();

#elif defined(SENSOR_LTR390)
    scifClearAlertIntSource();
    output = scifTaskData.i2cTempAndHumiditySensor.output;
    Semaphore_post(appSensorSemHandle);

    scifAckAlertEvents();

#elif defined(SENSOR_APDS9930)
    scifClearAlertIntSource();
    output = scifTaskData.i2cTempAndHumiditySensor.output;
    Semaphore_post(appSensorSemHandle);

    scifAckAlertEvents();

#endif

}

/////////////////////// sensor

/*******************************************************************************
 * @fn          zclSensor_initialization
 *
 * @brief       Initialize the application
 *
 * @param       none
 *
 * @return      none
 */
static void zclSensor_initialization(void)
{
    /* Initialize user clocks */
    zclSensor_initializeClocks();

    /* create semaphores for messages / events
     */
    Semaphore_Params semParam;
    Semaphore_Params_init(&semParam);
    semParam.mode = ti_sysbios_knl_Semaphore_Mode_COUNTING;
    Semaphore_construct(&appSem, 0, &semParam);
    appSemHandle = Semaphore_handle(&appSem);

    appServiceTaskId = OsalPort_registerTask(Task_self(), appSemHandle, &appServiceTaskEvents);

    //Initialize stack
    zclSensor_Init();
}



/*******************************************************************************
 * @fn      SetupZStackCallbacks
 *
 * @brief   Setup the Zstack Callbacks wanted
 *
 * @param   none
 *
 * @return  none
 */
static void SetupZStackCallbacks(void)
{
    zstack_devZDOCBReq_t zdoCBReq = {0};

    // Register for Callbacks, turn on:
    //  Device State Change,
    //  ZDO Match Descriptor Response,
    zdoCBReq.has_devStateChange = true;
    zdoCBReq.devStateChange = true;
    zdoCBReq.has_ieeeAddrRsp = true;
    zdoCBReq.ieeeAddrRsp = true;

    (void)Zstackapi_DevZDOCBReq(appServiceTaskId, &zdoCBReq);
}

#define SAMPLEAPP_KEY_EVT_UI                  0x0200

typedef void (* uiAppProcessKeyCB_t)(uint8_t key, Button_EventMask _buttonEvents, uint32_t duration);
static uint16_t events = 0;
static Button_Handle  keys;
static uiAppProcessKeyCB_t gpAppKeyCB;
static Button_Handle gLeftButtonHandle;


/*********************************************************************
 * @fn      zclSampleAppsUI_changeKeyCallback
 *
 * @brief   Key event handler function
 *
 * @param   keysPressed - keys to be process in application context
 *
 * @return  none
 */
static void zclSampleAppsUI_changeKeyCallback(Button_Handle _buttonHandle, Button_EventMask _buttonEvents)
{
    if (_buttonEvents & (Button_EV_CLICKED | Button_EV_LONGCLICKED))
    {
        keys = _buttonHandle;

        events |= SAMPLEAPP_KEY_EVT_UI;

        // Wake up the application thread when it waits for clock event
        Semaphore_post(appSemHandle);
    }
}



static void UI_processKey(Button_Handle _buttonHandle,
                                 Button_EventMask _buttonEvents)
{
  if((_buttonHandle == gLeftButtonHandle) && gpAppKeyCB)
  {
      uint32_t duration = Button_getLastPressedDuration(_buttonHandle);
      gpAppKeyCB(CONFIG_BTN_LEFT, _buttonEvents, duration);
  }
//    if((_buttonHandle == gRightButtonHandle) && gpAppKeyCB)
//    {
//        gpAppKeyCB(CONFIG_BTN_RIGHT, _buttonEvents);
//    }
}


/*********************************************************************
 * @fn          zclSampleTemperatureSensor_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
static void zclSensor_Init( void )
{
#ifdef BDB_REPORTING
      zstack_bdbRepAddAttrCfgRecordDefaultToListReq_t Req = {0};
#endif

  //Register Endpoint
  zclSensorEpDesc.endPoint = SENSOR_ENDPOINT;
  zclSensorEpDesc.simpleDesc = &zclSensor_SimpleDesc;
  zclport_registerEndpoint(appServiceTaskId, &zclSensorEpDesc);

  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SENSOR_ENDPOINT, &zclSensor_CmdCallbacks );

  // Register the application's attribute list and reset to default values
  zclSensor_ResetAttributesToDefaultValues();
  zcl_registerAttrList( SENSOR_ENDPOINT, zclSensor_NumAttributes, zclSensor_Attrs );

  // Register the Application to receive the unprocessed Foundation command/response messages
  zclport_registerZclHandleExternal(SENSOR_ENDPOINT, zclSensor_ProcessIncomingMsg);

  //Write the bdb initialization parameters
  zclSensor_initParameters();

  //Setup ZDO callbacks
  SetupZStackCallbacks();

#if defined ( BDB_TL_INITIATOR )
  zclSensor_BdbCommissioningModes |= BDB_COMMISSIONING_MODE_INITIATOR_TL;
#endif

#if !defined (DISABLE_GREENPOWER_BASIC_PROXY) && (ZG_BUILD_RTR_TYPE)
  gp_endpointInit(appServiceTaskId);
#endif

#ifdef BDB_REPORTING

  /// setup Humidity reporting
  Req.attrID = ATTRID_POWER_CONFIGURATION_BATTERY_PERCENTAGE_REMAINING;
  Req.cluster = ZCL_CLUSTER_ID_GENERAL_POWER_CFG;
  Req.endpoint = SENSOR_ENDPOINT;
  Req.maxReportInt = 60;
  Req.minReportInt = 30;
  OsalPort_memcpy(Req.reportableChange,reportableChange,BDBREPORTING_MAX_ANALOG_ATTR_SIZE);
  Zstackapi_bdbRepAddAttrCfgRecordDefaultToListReq(appServiceTaskId,&Req);

#if defined(SENSOR_AHT10) || defined(SENSOR_SI7021)

  Req.attrID = ATTRID_TEMPERATURE_MEASUREMENT_MEASURED_VALUE;
  Req.cluster = ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT;
  Req.endpoint = SENSOR_ENDPOINT;
  Req.maxReportInt = 60;
  Req.minReportInt = 30;
  OsalPort_memcpy(Req.reportableChange,reportableChange,BDBREPORTING_MAX_ANALOG_ATTR_SIZE);
  Zstackapi_bdbRepAddAttrCfgRecordDefaultToListReq(appServiceTaskId,&Req);

  /// setup Humidity reporting
  Req.attrID = ATTRID_RELATIVITY_HUMIDITY_MEASURED_VALUE;
  Req.cluster = ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY;
  Req.endpoint = SENSOR_ENDPOINT;
  Req.maxReportInt = 60;
  Req.minReportInt = 30;
  OsalPort_memcpy(Req.reportableChange,reportableChange,BDBREPORTING_MAX_ANALOG_ATTR_SIZE);
  Zstackapi_bdbRepAddAttrCfgRecordDefaultToListReq(appServiceTaskId,&Req);

#elif defined(SENSOR_LTR390) || defined(SENSOR_APDS9930)

  Req.attrID = ATTRID_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE;
  Req.cluster = ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT;
  Req.endpoint = SENSOR_ENDPOINT;
  Req.maxReportInt = 60;
  Req.minReportInt = 30;
  OsalPort_memcpy(Req.reportableChange,reportableChange,BDBREPORTING_MAX_ANALOG_ATTR_SIZE);
  Zstackapi_bdbRepAddAttrCfgRecordDefaultToListReq(appServiceTaskId,&Req);

#endif
#endif

  /* Initialize btns */
  Button_Params bparams;
  Button_Params_init(&bparams);
  gLeftButtonHandle = Button_open(CONFIG_BTN_LEFT, NULL, &bparams);

  gpAppKeyCB = zclSensor_processKey;

  // Set button callback
  Button_setCallback(gLeftButtonHandle, zclSampleAppsUI_changeKeyCallback);

  zclSensor_BdbCommissioningModes = BDB_COMMISSIONING_MODE_NWK_STEERING | BDB_COMMISSIONING_MODE_FINDING_BINDING;

  LED_Params ledParams;
  LED_Params_init(&ledParams);
  gGreenLedHandle = LED_open(CONFIG_LED_GREEN, &ledParams);


#ifndef CUI_DISABLE
  // set up default application BDB commissioning modes based on build type
  if(ZG_BUILD_COORDINATOR_TYPE && ZG_DEVICE_COORDINATOR_TYPE)
  {
    zclSensor_BdbCommissioningModes = BDB_COMMISSIONING_MODE_NWK_FORMATION | BDB_COMMISSIONING_MODE_NWK_STEERING | BDB_COMMISSIONING_MODE_FINDING_BINDING;
  }
  else if (ZG_BUILD_JOINING_TYPE && ZG_DEVICE_JOINING_TYPE)
  {
    zclSensor_BdbCommissioningModes = BDB_COMMISSIONING_MODE_NWK_STEERING | BDB_COMMISSIONING_MODE_FINDING_BINDING;
  }


  gCuiHandle = UI_Init( appServiceTaskId,                     // Application Task ID
            &appServiceTaskEvents,                // The events processed by the sample application
            appSemHandle,                         // Semaphore to post the events in the application thread
            &zclSampleTemperatureSensor_IdentifyTime,
            &zclSensor_BdbCommissioningModes,   // A pointer to the application's bdbCommissioningModes
            zclSampleTemperatureSensor_appStr,                   // A pointer to the app-specific name string
            zclSensor_processKey,               // A pointer to the app-specific key process function
            zclSensor_RemoveAppNvmData          // A pointer to the app-specific NV Item reset function
            );

  //Request the Red LED for App
//  LED_Params ledParams;
//  LED_Params_init(&ledParams);
//  LED_Handle gRedLedHandle = LED_open(CONFIG_LED_RED, &ledParams);
//  (void) gRedLedHandle;


  //Initialize the SampleDoorLock UI status line
  zclSampleTemperatureSensor_InitializeStatusLine(gCuiHandle);
#endif


#if !defined (DISABLE_GREENPOWER_BASIC_PROXY) && (ZG_BUILD_RTR_TYPE)
  app_Green_Power_Init(appServiceTaskId, &appServiceTaskEvents, appSemHandle, SAMPLEAPP_PROCESS_GP_DATA_SEND_EVT,
                       SAMPLEAPP_PROCESS_GP_EXPIRE_DUPLICATE_EVT, SAMPLEAPP_PROCESS_GP_TEMP_MASTER_EVT);
#endif

#if defined ( BDB_TL_INITIATOR )
    touchLinkInitiatorApp_Init(appServiceTaskId);
#elif defined ( BDB_TL_TARGET )
    touchLinkTargetApp_Init(appServiceTaskId);
#endif

  // Call BDB initialization. Should be called once from application at startup to restore
  // previous network configuration, if applicable.
  zstack_bdbStartCommissioningReq_t zstack_bdbStartCommissioningReq;
  zstack_bdbStartCommissioningReq.commissioning_mode = 0;
  Zstackapi_bdbStartCommissioningReq(appServiceTaskId,&zstack_bdbStartCommissioningReq);
}

//#ifndef CUI_DISABLE
/*********************************************************************
 * @fn          zclSampleTemperatureSensor_RemoveAppNvmData
 *
 * @brief       Callback when Application performs reset to Factory New Reset.
 *              Application must restore the application to default values
 *
 * @param       none
 *
 * @return      none
 */
static void zclSensor_RemoveAppNvmData(void)
{

}
//#endif

static void zclSensor_initParameters(void)
{
    zstack_bdbSetAttributesReq_t zstack_bdbSetAttrReq;

    zstack_bdbSetAttrReq.bdbCommissioningGroupID              = BDB_DEFAULT_COMMISSIONING_GROUP_ID;
    zstack_bdbSetAttrReq.bdbPrimaryChannelSet                 = BDB_DEFAULT_PRIMARY_CHANNEL_SET;
    zstack_bdbSetAttrReq.bdbScanDuration                      = BDB_DEFAULT_SCAN_DURATION;
    zstack_bdbSetAttrReq.bdbSecondaryChannelSet               = BDB_DEFAULT_SECONDARY_CHANNEL_SET;
    zstack_bdbSetAttrReq.has_bdbCommissioningGroupID          = TRUE;
    zstack_bdbSetAttrReq.has_bdbPrimaryChannelSet             = TRUE;
    zstack_bdbSetAttrReq.has_bdbScanDuration                  = TRUE;
    zstack_bdbSetAttrReq.has_bdbSecondaryChannelSet           = TRUE;
#if (ZG_BUILD_COORDINATOR_TYPE)
    zstack_bdbSetAttrReq.has_bdbJoinUsesInstallCodeKey        = TRUE;
    zstack_bdbSetAttrReq.has_bdbTrustCenterNodeJoinTimeout    = TRUE;
    zstack_bdbSetAttrReq.has_bdbTrustCenterRequireKeyExchange = TRUE;
    zstack_bdbSetAttrReq.bdbJoinUsesInstallCodeKey            = BDB_DEFAULT_JOIN_USES_INSTALL_CODE_KEY;
    zstack_bdbSetAttrReq.bdbTrustCenterNodeJoinTimeout        = BDB_DEFAULT_TC_NODE_JOIN_TIMEOUT;
    zstack_bdbSetAttrReq.bdbTrustCenterRequireKeyExchange     = BDB_DEFAULT_TC_REQUIRE_KEY_EXCHANGE;
#endif
#if (ZG_BUILD_JOINING_TYPE)
    zstack_bdbSetAttrReq.has_bdbTCLinkKeyExchangeAttemptsMax  = TRUE;
    zstack_bdbSetAttrReq.has_bdbTCLinkKeyExchangeMethod       = TRUE;
    zstack_bdbSetAttrReq.bdbTCLinkKeyExchangeAttemptsMax      = BDB_DEFAULT_TC_LINK_KEY_EXCHANGE_ATTEMPS_MAX;
    zstack_bdbSetAttrReq.bdbTCLinkKeyExchangeMethod           = BDB_DEFAULT_TC_LINK_KEY_EXCHANGE_METHOD;
#endif

    Zstackapi_bdbSetAttributesReq(appServiceTaskId, &zstack_bdbSetAttrReq);
}

/*******************************************************************************
 * @fn      zclSampleTemperatureSensor_initializeClocks
 *
 * @brief   Initialize Clocks
 *
 * @param   none
 *
 * @return  none
 */
static void zclSensor_initializeClocks(void)
{
#if ZG_BUILD_ENDDEVICE_TYPE
    // Initialize the timers needed for this application
    EndDeviceRejoinClkHandle = UtilTimer_construct(
    &EndDeviceRejoinClkStruct,
    zclSensor_processEndDeviceRejoinTimeoutCallback,
    SAMPLEAPP_END_DEVICE_REJOIN_DELAY,
    0, false, 0);
#endif
}

#if ZG_BUILD_ENDDEVICE_TYPE
/*******************************************************************************
 * @fn      zclSampleTemperatureSensor_processEndDeviceRejoinTimeoutCallback
 *
 * @brief   Timeout handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void zclSensor_processEndDeviceRejoinTimeoutCallback(UArg a0)
{
    (void)a0; // Parameter is not used

    appServiceTaskEvents |= SAMPLEAPP_END_DEVICE_REJOIN_EVT;

    // Wake up the application thread when it waits for clock event
    Semaphore_post(appSemHandle);
}
#endif


/*******************************************************************************
 * @fn      zclSampleTemperatureSensor_process_loop
 *
 * @brief   Application task processing start.
 *
 * @param   none
 *
 * @return  void
 */
static void zclSensor_process_loop(void)
{
    /* Forever loop */
    for(;;)
    {
        zstackmsg_genericReq_t *pMsg = NULL;
        bool msgProcessed = FALSE;

        /* Wait for response message */
        if(Semaphore_pend(appSemHandle, BIOS_WAIT_FOREVER ))
        {
            /* Retrieve the response message */
            if( (pMsg = (zstackmsg_genericReq_t*) OsalPort_msgReceive( appServiceTaskId )) != NULL)
            {
                /* Process the message from the stack */
                zclSensor_processZStackMsgs(pMsg);
                // Free any separately allocated memory
                msgProcessed = Zstackapi_freeIndMsg(pMsg);
            }

            if((msgProcessed == FALSE) && (pMsg != NULL))
            {
                OsalPort_msgDeallocate((uint8_t*)pMsg);
            }

#if !defined (DISABLE_GREENPOWER_BASIC_PROXY) && (ZG_BUILD_RTR_TYPE)
            if(appServiceTaskEvents & SAMPLEAPP_PROCESS_GP_DATA_SEND_EVT)
            {
                if(zgGP_ProxyCommissioningMode == TRUE)
                {
                  zcl_gpSendCommissioningNotification();
                }
                else
                {
                  zcl_gpSendNotification();
                }
                appServiceTaskEvents &= ~SAMPLEAPP_PROCESS_GP_DATA_SEND_EVT;
            }

            if(appServiceTaskEvents & SAMPLEAPP_PROCESS_GP_EXPIRE_DUPLICATE_EVT)
            {
                gp_expireDuplicateFiltering();
                appServiceTaskEvents &= ~SAMPLEAPP_PROCESS_GP_EXPIRE_DUPLICATE_EVT;
            }

            if(appServiceTaskEvents & SAMPLEAPP_PROCESS_GP_TEMP_MASTER_EVT)
            {
                gp_returnOperationalChannel();
                appServiceTaskEvents &= ~SAMPLEAPP_PROCESS_GP_TEMP_MASTER_EVT;
            }
#endif

            if (events & SAMPLEAPP_KEY_EVT_UI)
            {
              UI_processKey(keys, Button_EV_CLICKED);
              events &= ~SAMPLEAPP_KEY_EVT_UI;
            }


#ifndef CUI_DISABLE
            //Process the events that the UI may have
            zclsampleApp_ui_event_loop();
#endif

#if ZG_BUILD_ENDDEVICE_TYPE
            if ( appServiceTaskEvents & SAMPLEAPP_END_DEVICE_REJOIN_EVT )
            {
              zstack_bdbZedAttemptRecoverNwkRsp_t zstack_bdbZedAttemptRecoverNwkRsp;

              Zstackapi_bdbZedAttemptRecoverNwkReq(appServiceTaskId,&zstack_bdbZedAttemptRecoverNwkRsp);

              appServiceTaskEvents &= ~SAMPLEAPP_END_DEVICE_REJOIN_EVT;
            }
#endif

#ifndef CUI_DISABLE
  //Update status line
  zclSampleTemperatureSensor_UpdateStatusLine();
#endif

        }
    }
}




/*******************************************************************************
 * @fn      zclSampleTemperatureSensor_processZStackMsgs
 *
 * @brief   Process event from Stack
 *
 * @param   pMsg - pointer to incoming ZStack message to process
 *
 * @return  void
 */
static void zclSensor_processZStackMsgs(zstackmsg_genericReq_t *pMsg)
{
      switch(pMsg->hdr.event)
      {
          case zstackmsg_CmdIDs_BDB_NOTIFICATION:
              {
                  zstackmsg_bdbNotificationInd_t *pInd;
                  pInd = (zstackmsg_bdbNotificationInd_t*)pMsg;
                  zclSensor_ProcessCommissioningStatus(&(pInd->Req));
              }
              break;

          case zstackmsg_CmdIDs_BDB_IDENTIFY_TIME_CB:
              {
#ifndef CUI_DISABLE
                zstackmsg_bdbIdentifyTimeoutInd_t *pInd;
                pInd = (zstackmsg_bdbIdentifyTimeoutInd_t*) pMsg;
                uiProcessIdentifyTimeChange(&(pInd->EndPoint));
#endif
              }
              break;

          case zstackmsg_CmdIDs_BDB_BIND_NOTIFICATION_CB:
              {
#ifndef CUI_DISABLE
                zstackmsg_bdbBindNotificationInd_t *pInd;
                pInd = (zstackmsg_bdbBindNotificationInd_t*) pMsg;
                uiProcessBindNotification(&(pInd->Req));
#endif
              }
              break;

          case zstackmsg_CmdIDs_AF_INCOMING_MSG_IND:
              {
                  // Process incoming data messages
                  zstackmsg_afIncomingMsgInd_t *pInd;
                  pInd = (zstackmsg_afIncomingMsgInd_t *)pMsg;
                  zclSensor_processAfIncomingMsgInd( &(pInd->req) );
              }
              break;





#if (ZG_BUILD_JOINING_TYPE)
          case zstackmsg_CmdIDs_BDB_CBKE_TC_LINK_KEY_EXCHANGE_IND:
          {
            zstack_bdbCBKETCLinkKeyExchangeAttemptReq_t zstack_bdbCBKETCLinkKeyExchangeAttemptReq;
            /* Z3.0 has not defined CBKE yet, so lets attempt default TC Link Key exchange procedure
             * by reporting CBKE failure.
             */

            zstack_bdbCBKETCLinkKeyExchangeAttemptReq.didSuccess = FALSE;

            Zstackapi_bdbCBKETCLinkKeyExchangeAttemptReq(appServiceTaskId,
                                                         &zstack_bdbCBKETCLinkKeyExchangeAttemptReq);
          }
          break;

          case zstackmsg_CmdIDs_BDB_FILTER_NWK_DESCRIPTOR_IND:

           /*   User logic to remove networks that do not want to join
            *   Networks to be removed can be released with Zstackapi_bdbNwkDescFreeReq
            */

            Zstackapi_bdbFilterNwkDescComplete(appServiceTaskId);
          break;

#endif
          case zstackmsg_CmdIDs_DEV_STATE_CHANGE_IND:
          {
#ifndef CUI_DISABLE
            // The ZStack Thread is indicating a State change
            zstackmsg_devStateChangeInd_t *pInd =
                (zstackmsg_devStateChangeInd_t *)pMsg;
                  UI_DeviceStateUpdated(&(pInd->req));
#endif
          }
              zstackmsg_devStateChangeInd_t *pInd =
                          (zstackmsg_devStateChangeInd_t *)pMsg;
              zstack_devStateChangeInd_t* pReq = &(pInd->req);
              if (pReq != NULL)
              {
                switch (pReq->state)
                {
                case zstack_DevState_DEV_ZB_COORD:
                case zstack_DevState_DEV_ROUTER:
                case zstack_DevState_DEV_END_DEVICE:
                    onNwk = true;
                    LED_stopBlinking(gGreenLedHandle);
                    LED_startBlinking(gGreenLedHandle, 500, 3);
                    break;
                default:
                    onNwk = false;
                    LED_stopBlinking(gGreenLedHandle);
                    LED_setOff(gGreenLedHandle);
                  break;
                }
              }
              else
              {
                  onNwk = false;
                  LED_stopBlinking(gGreenLedHandle);
                  LED_setOff(gGreenLedHandle);
               }
          break;

          ///////////////////
          case zstackmsg_CmdIDs_ZDO_IEEE_ADDR_RSP:
          {
              zstackmsg_zdoIeeeAddrRspInd_t *pInd =
                      (zstackmsg_zdoIeeeAddrRspInd_t *)pMsg;

              if(pInd->rsp.status == zstack_ZdpStatus_SUCCESS)
              {

                zstack_sysNwkInfoReadRsp_t  Rsp;
                //Get our IEEE address
                Zstackapi_sysNwkInfoReadReq(appServiceTaskId, &Rsp);

#if defined(SENSOR_AHT10) || defined(SENSOR_SI7021)
                uint16 clusterIds[] = {ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT, ZCL_CLUSTER_ID_GENERAL_POWER_CFG, ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY};
#elif defined(SENSOR_LTR390) || defined(SENSOR_APDS9930)
                uint16 clusterIds[] = {ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT, ZCL_CLUSTER_ID_GENERAL_POWER_CFG};

#endif
                zAddrType_t dstaddr;
                OsalPort_memcpy(dstaddr.addr.extAddr, pInd->rsp.ieeeAddr, Z_EXTADDR_LEN);
                dstaddr.addrMode = Addr64Bit;
                uint8_t srcEp = SENSOR_ENDPOINT;
                uint8_t dstEp = 1;
                // The response to MT interface has to be pack into buf
                BindingEntry_t *result = bindAddEntry( srcEp, &dstaddr, dstEp, 3, clusterIds );

              }
          }

          /*
           * These are messages/indications from ZStack that this
           * application doesn't process.  These message can be
           * processed by your application, remove from this list and
           * process them here in this switch statement.
           */

#if !defined (DISABLE_GREENPOWER_BASIC_PROXY) && (ZG_BUILD_RTR_TYPE)
          case zstackmsg_CmdIDs_GP_DATA_IND:
          {
              zstackmsg_gpDataInd_t *pInd;
              pInd = (zstackmsg_gpDataInd_t*)pMsg;
              gp_processDataIndMsg( &(pInd->Req) );
          }
          break;

          case zstackmsg_CmdIDs_GP_SECURITY_REQ:
          {
              zstackmsg_gpSecReq_t *pInd;
              pInd = (zstackmsg_gpSecReq_t*)pMsg;
              gp_processSecRecMsg( &(pInd->Req) );
          }
          break;

          case zstackmsg_CmdIDs_GP_CHECK_ANNCE:
          {
              zstackmsg_gpCheckAnnounce_t *pInd;
              pInd = (zstackmsg_gpCheckAnnounce_t*)pMsg;
              gp_processCheckAnnceMsg( &(pInd->Req) );
          }
          break;


          case zstackmsg_CmdIDs_GP_COMMISSIONING_MODE_IND:
          {
#ifndef CUI_DISABLE
            zstackmsg_gpCommissioningModeInd_t *pInd;
            pInd = (zstackmsg_gpCommissioningModeInd_t*)pMsg;
            UI_SetGPPCommissioningMode( &(pInd->Req) );
#endif
          }
          break;

#endif

#ifdef BDB_TL_TARGET
          case zstackmsg_CmdIDs_BDB_TOUCHLINK_TARGET_ENABLE_IND:
          {
            zstackmsg_bdbTouchLinkTargetEnableInd_t *pInd =
              (zstackmsg_bdbTouchLinkTargetEnableInd_t*)pMsg;

            uiProcessTouchlinkTargetEnable(pInd->Enable);
          }
          break;
#endif
          case zstackmsg_CmdIDs_DEV_PERMIT_JOIN_IND:
          case zstackmsg_CmdIDs_BDB_TC_LINK_KEY_EXCHANGE_NOTIFICATION_IND:
          case zstackmsg_CmdIDs_AF_DATA_CONFIRM_IND:
          case zstackmsg_CmdIDs_ZDO_DEVICE_ANNOUNCE:
          case zstackmsg_CmdIDs_ZDO_NWK_ADDR_RSP:
          case zstackmsg_CmdIDs_ZDO_NODE_DESC_RSP:
          case zstackmsg_CmdIDs_ZDO_POWER_DESC_RSP:
          case zstackmsg_CmdIDs_ZDO_SIMPLE_DESC_RSP:
          case zstackmsg_CmdIDs_ZDO_ACTIVE_EP_RSP:
          case zstackmsg_CmdIDs_ZDO_COMPLEX_DESC_RSP:
          case zstackmsg_CmdIDs_ZDO_USER_DESC_RSP:
          case zstackmsg_CmdIDs_ZDO_USER_DESC_SET_RSP:
          case zstackmsg_CmdIDs_ZDO_SERVER_DISC_RSP:
          case zstackmsg_CmdIDs_ZDO_END_DEVICE_BIND_RSP:
          case zstackmsg_CmdIDs_ZDO_BIND_RSP:
          case zstackmsg_CmdIDs_ZDO_UNBIND_RSP:
          case zstackmsg_CmdIDs_ZDO_MGMT_NWK_DISC_RSP:
          case zstackmsg_CmdIDs_ZDO_MGMT_LQI_RSP:
          case zstackmsg_CmdIDs_ZDO_MGMT_RTG_RSP:
          case zstackmsg_CmdIDs_ZDO_MGMT_BIND_RSP:
          case zstackmsg_CmdIDs_ZDO_MGMT_LEAVE_RSP:
          case zstackmsg_CmdIDs_ZDO_MGMT_DIRECT_JOIN_RSP:
          case zstackmsg_CmdIDs_ZDO_MGMT_PERMIT_JOIN_RSP:
          case zstackmsg_CmdIDs_ZDO_MGMT_NWK_UPDATE_NOTIFY:
          case zstackmsg_CmdIDs_ZDO_SRC_RTG_IND:
          case zstackmsg_CmdIDs_ZDO_CONCENTRATOR_IND:
          case zstackmsg_CmdIDs_ZDO_LEAVE_CNF:
          case zstackmsg_CmdIDs_ZDO_LEAVE_IND:
          case zstackmsg_CmdIDs_SYS_RESET_IND:
          case zstackmsg_CmdIDs_AF_REFLECT_ERROR_IND:
          case zstackmsg_CmdIDs_ZDO_TC_DEVICE_IND:
              break;

          default:
              break;
      }

}



/*******************************************************************************
 *
 * @fn          zclSensor_processAfIncomingMsgInd
 *
 * @brief       Process AF Incoming Message Indication message
 *
 * @param       pInMsg - pointer to incoming message
 *
 * @return      none
 *
 */
static void zclSensor_processAfIncomingMsgInd(zstack_afIncomingMsgInd_t *pInMsg)
{
    afIncomingMSGPacket_t afMsg;

    /*
     * All incoming messages are passed to the ZCL message processor,
     * first convert to a structure that ZCL can process.
     */
    afMsg.groupId = pInMsg->groupID;
    afMsg.clusterId = pInMsg->clusterId;
    afMsg.srcAddr.endPoint = pInMsg->srcAddr.endpoint;
    afMsg.srcAddr.panId = pInMsg->srcAddr.panID;
    afMsg.srcAddr.addrMode = (afAddrMode_t)pInMsg->srcAddr.addrMode;
    if( (afMsg.srcAddr.addrMode == afAddr16Bit)
        || (afMsg.srcAddr.addrMode == afAddrGroup)
        || (afMsg.srcAddr.addrMode == afAddrBroadcast) )
    {
        afMsg.srcAddr.addr.shortAddr = pInMsg->srcAddr.addr.shortAddr;
    }
    else if(afMsg.srcAddr.addrMode == afAddr64Bit)
    {
        OsalPort_memcpy(afMsg.srcAddr.addr.extAddr, &(pInMsg->srcAddr.addr.extAddr), 8);
    }
    afMsg.macDestAddr = pInMsg->macDestAddr;
    afMsg.endPoint = pInMsg->endpoint;
    afMsg.wasBroadcast = pInMsg->wasBroadcast;
    afMsg.LinkQuality = pInMsg->linkQuality;
    afMsg.correlation = pInMsg->correlation;
    afMsg.rssi = pInMsg->rssi;
    afMsg.SecurityUse = pInMsg->securityUse;
    afMsg.timestamp = pInMsg->timestamp;
    afMsg.nwkSeqNum = pInMsg->nwkSeqNum;
    afMsg.macSrcAddr = pInMsg->macSrcAddr;
    afMsg.radius = pInMsg->radius;
    afMsg.cmd.DataLength = pInMsg->n_payload;
    afMsg.cmd.Data = pInMsg->pPayload;

    zcl_ProcessMessageMSG(&afMsg);
}




/*********************************************************************
 * @fn      zclSensor_ProcessCommissioningStatus
 *
 * @brief   Callback in which the status of the commissioning process are reported
 *
 * @param   bdbCommissioningModeMsg - Context message of the status of a commissioning process
 *
 * @return  none
 */
static void zclSensor_ProcessCommissioningStatus(bdbCommissioningModeMsg_t* bdbCommissioningModeMsg)
{
    switch(bdbCommissioningModeMsg->bdbCommissioningMode)
    {
      case BDB_COMMISSIONING_FORMATION:
        if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
        {
          //YOUR JOB:

        }
        else
        {
          //Want to try other channels?
          //try with bdb_setChannelAttribute
        }
      break;
      case BDB_COMMISSIONING_NWK_STEERING:
        if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
        {
          //YOUR JOB:
          //We are on the nwk, what now?
            zstack_zdoIeeeAddrReq_t Req_addr;
            Req_addr.startIndex = 0;
            Req_addr.type = zstack_NwkAddrReqType_SINGLE_DEVICE;
            Req_addr.nwkAddr = 0x0000; //pInd->rsp.nwkAddrOfInterest;
            Zstackapi_ZdoIeeeAddrReq(appServiceTaskId, &Req_addr);

        }
        else
        {
          //See the possible errors for nwk steering procedure
          //No suitable networks found
          //Want to try other channels?
          //try with bdb_setChannelAttribute
            onNwk = false;
        }
      break;
      case BDB_COMMISSIONING_FINDING_BINDING:
        if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
        {
          //YOUR JOB:
        }
        else
        {
          //YOUR JOB:
          //retry?, wait for user interaction?
        }
      break;
      case BDB_COMMISSIONING_INITIALIZATION:
        //Initialization notification can only be successful. Failure on initialization
        //only happens for ZED and is notified as BDB_COMMISSIONING_PARENT_LOST notification

        //YOUR JOB:
        //We are on a network, what now?

      break;
#if ZG_BUILD_ENDDEVICE_TYPE
    case BDB_COMMISSIONING_PARENT_LOST:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_NETWORK_RESTORED)
      {
        //We did recover from losing parent
          onNwk = true;

      }
      else
      {
        //Parent not found, attempt to rejoin again after a fixed delay
        UtilTimer_setTimeout( EndDeviceRejoinClkHandle, SAMPLEAPP_END_DEVICE_REJOIN_DELAY );
        UtilTimer_start(&EndDeviceRejoinClkStruct);
      }
    break;
#endif
    }

#ifndef CUI_DISABLE
  UI_UpdateBdbStatusLine(bdbCommissioningModeMsg);
#endif
}

/*********************************************************************
 * @fn      zclSensor_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSensor_BasicResetCB( void )
{
  zclSensor_ResetAttributesToDefaultValues();
#ifndef CUI_DISABLE
  zclSampleTemperatureSensor_UpdateStatusLine();
#endif
}


/******************************************************************************
 *
 *  Functions for processing ZCL Foundation incoming Command/Response messages
 *
 *****************************************************************************/


/*********************************************************************
 * @fn      zclSensor_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  uint8_t - TRUE if got handled
 */
static uint8_t zclSensor_ProcessIncomingMsg( zclIncoming_t *pInMsg)
{
  uint8_t handled = FALSE;
  switch ( pInMsg->hdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      zclSensor_ProcessInReadRspCmd( pInMsg );
      handled = TRUE;
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      zclSensor_ProcessInWriteRspCmd( pInMsg );
      handled = TRUE;
      break;
#endif
#ifdef ZCL_REPORT
    // See ZCL Test Applicaiton (zcl_testapp.c) for sample code on Attribute Reporting
    case ZCL_CMD_CONFIG_REPORT:
      //zclSampleTemperatureSensor_ProcessInConfigReportCmd( pInMsg );
      break;
      case ZCL_CMD_READ_REPORT_CFG:
      //zclSampleTemperatureSensor_ProcessInReadReportCfgCmd( pInMsg );
      break;
    case ZCL_CMD_CONFIG_REPORT_RSP:
      //zclSampleTemperatureSensor_ProcessInConfigReportRspCmd( pInMsg );
      break;
    case ZCL_CMD_READ_REPORT_CFG_RSP:
      //zclSampleTemperatureSensor_ProcessInReadReportCfgRspCmd( pInMsg );
      break;

    case ZCL_CMD_REPORT:
      //zclSampleTemperatureSensor_ProcessInReportCmd( pInMsg );
      break;
#endif
    case ZCL_CMD_DEFAULT_RSP:
      zclSensor_ProcessInDefaultRspCmd( pInMsg );
      handled = TRUE;
      break;
#ifdef ZCL_DISCOVER
    case ZCL_CMD_DISCOVER_CMDS_RECEIVED_RSP:
      zclSensor_ProcessInDiscCmdsRspCmd( pInMsg );
      handled = TRUE;
      break;

    case ZCL_CMD_DISCOVER_CMDS_GEN_RSP:
      zclSensor_ProcessInDiscCmdsRspCmd( pInMsg );
      handled = TRUE;
      break;

    case ZCL_CMD_DISCOVER_ATTRS_RSP:
      zclSensor_ProcessInDiscAttrsRspCmd( pInMsg );
      handled = TRUE;
      break;

    case ZCL_CMD_DISCOVER_ATTRS_EXT_RSP:
      zclSensor_ProcessInDiscAttrsExtRspCmd( pInMsg );
      handled = TRUE;
      break;
#endif
    default:
      break;
  }


  return handled;
}

#ifdef ZCL_READ
/*********************************************************************
 * @fn      zclSensor_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8_t zclSensor_ProcessInReadRspCmd( zclIncoming_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8_t i;

  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < readRspCmd->numAttr; i++ )
  {
    // Notify the originator of the results of the original read attributes
    // attempt and, for each successfull request, the value of the requested
    // attribute
  }

  return ( TRUE );
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      zclSensor_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8_t zclSensor_ProcessInWriteRspCmd( zclIncoming_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8_t i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < writeRspCmd->numAttr; i++ )
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }

  return ( TRUE );
}
#endif // ZCL_WRITE

/*********************************************************************
 * @fn      zclSensor_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8_t zclSensor_ProcessInDefaultRspCmd( zclIncoming_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;

  // Device is notified of the Default Response command.
  (void)pInMsg;

  return ( TRUE );
}

#ifdef ZCL_DISCOVER
/*********************************************************************
 * @fn      zclSensor_ProcessInDiscCmdsRspCmd
 *
 * @brief   Process the Discover Commands Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8_t zclSensor_ProcessInDiscCmdsRspCmd( zclIncoming_t *pInMsg )
{
  zclDiscoverCmdsCmdRsp_t *discoverRspCmd;
  uint8_t i;

  discoverRspCmd = (zclDiscoverCmdsCmdRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numCmd; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclSensor_ProcessInDiscAttrsRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8_t zclSensor_ProcessInDiscAttrsRspCmd( zclIncoming_t *pInMsg )
{
  zclDiscoverAttrsRspCmd_t *discoverRspCmd;
  uint8_t i;

  discoverRspCmd = (zclDiscoverAttrsRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclSensor_ProcessInDiscAttrsExtRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Extended Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8_t zclSensor_ProcessInDiscAttrsExtRspCmd( zclIncoming_t *pInMsg )
{
  zclDiscoverAttrsExtRsp_t *discoverRspCmd;
  uint8_t i;

  discoverRspCmd = (zclDiscoverAttrsExtRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}
#endif // ZCL_DISCOVER


/*********************************************************************
 * @fn      zclSensor_processKey
 *
 * @brief   Key event handler function
 *
 * @param   key - key to handle action for
 *          buttonEvents - event to handle action for
 *
 * @return  none
 */
static void zclSensor_processKey(uint8_t key, Button_EventMask buttonEvents, uint32_t duration)
{
    if (buttonEvents & Button_EV_CLICKED)
    {
        // publish a zcl_SendReportCmd for temp and humidity
        if ((key == CONFIG_BTN_LEFT) && (duration < 1000))
        {
            if (onNwk) {
#if defined(SENSOR_AHT10) || defined(SENSOR_SI7021)
                zclReportCmd_t *pReportCmd;
                pReportCmd = malloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
                if(pReportCmd != NULL)
                {
                    // Fill in the single attribute information for the temperature reading
                    pReportCmd->numAttr = 1;
                    pReportCmd->attrList[0].attrID = ATTRID_TEMPERATURE_MEASUREMENT_MEASURED_VALUE;
                    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
                    pReportCmd->attrList[0].attrData = (void *)&zclTemperatureSensor_MeasuredValue;// (uint8 *)&(zclSampleTemperatureSensor_MeasuredValue);

                    afAddrType_t dstaddr;
                    dstaddr.addr.shortAddr = 0x0000;
                    dstaddr.addrMode = afAddr16Bit;
                    dstaddr.endPoint = 1;
                    dstaddr.panId = 0;

                    // Call ZCL function to send the report
                    zcl_SendReportCmd(8, &dstaddr, ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT, pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, 1, 0 );

                    free(pReportCmd);
                }

                pReportCmd = malloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
                if(pReportCmd != NULL)
                {
                    // Fill in the single attribute information for the temperature reading
                    pReportCmd->numAttr = 1;
                    pReportCmd->attrList[0].attrID = ATTRID_RELATIVITY_HUMIDITY_MEASURED_VALUE;
                    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT16;
                    pReportCmd->attrList[0].attrData = (void *)&zclHumiditySensor_MeasuredValue;// (uint8 *)&(zclSampleTemperatureSensor_MeasuredValue);

                    afAddrType_t dstaddr;
                    dstaddr.addr.shortAddr = 0x0000;
                    dstaddr.addrMode = afAddr16Bit;
                    dstaddr.endPoint = 1;
                    dstaddr.panId = 0;

                    // Call ZCL function to send the report
                    zcl_SendReportCmd(8, &dstaddr, ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY, pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, 1, 0 );

                    free(pReportCmd);
                }
                LED_startBlinking(gGreenLedHandle, 250, 2);
#elif defined(SENSOR_LTR390) || defined(SENSOR_APDS9930)
                zclReportCmd_t *pReportCmd;
                pReportCmd = malloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
                if(pReportCmd != NULL)
                {
                    // Fill in the single attribute information for the temperature reading
                    pReportCmd->numAttr = 1;
                    pReportCmd->attrList[0].attrID = ATTRID_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE;
                    pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT16;
                    pReportCmd->attrList[0].attrData = (void *)&zclIlluminanceSensor_MeasuredValue;// (uint8 *)&(zclSampleTemperatureSensor_MeasuredValue);

                    afAddrType_t dstaddr;
                    dstaddr.addr.shortAddr = 0x0000;
                    dstaddr.addrMode = afAddr16Bit;
                    dstaddr.endPoint = 1;
                    dstaddr.panId = 0;

                    // Call ZCL function to send the report
                    zcl_SendReportCmd(8, &dstaddr, ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT, pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, 1, 0 );

                    free(pReportCmd);
                }

                LED_startBlinking(gGreenLedHandle, 250, 2);


#endif
            } else {
                LED_startBlinking(gGreenLedHandle, 100, 5);

            }

        }
        //////////////////////////
        // commission if holding the button for 1 to 5 seconds
        if ((key == CONFIG_BTN_LEFT) && (duration >= 1000 & duration < 5000))
        {
            zstack_bdbStartCommissioningReq_t zstack_bdbStartCommissioningReq;
            zstack_bdbStartCommissioningReq.commissioning_mode = zclSensor_BdbCommissioningModes;
            Zstackapi_bdbStartCommissioningReq(appServiceTaskId,&zstack_bdbStartCommissioningReq);
        }
        //////////////////////////
        // hardware reset if greater than 5 seconds
        if ((key == CONFIG_BTN_LEFT) && (duration > 5000))
        {
            onNwk = false;
            LED_startBlinking(gGreenLedHandle, 1000, 2);
            zclSensor_RemoveAppNvmData();
            Zstackapi_bdbResetLocalActionReq(appServiceTaskId);

        }

//        if(key == CONFIG_BTN_RIGHT)
//        {
//            //Unused
//        }
    }
}

#ifndef CUI_DISABLE

void zclSampleTemperatureSensor_UiActionChangeTemp(const char _input, char* _lines[3], CUI_cursorInfo_t * _curInfo)
{
  char tempStr[3];
#ifdef BDB_REPORTING
  zstack_bdbRepChangedAttrValueReq_t Req;
#endif

  switch (_input)
  {
    case CUI_ITEM_INTERCEPT_START:
      break;
      case CUI_INPUT_UP:
        // increase the temperature
        if ( zclSensor_MeasuredValue < zclSampleTemperatureSensor_MaxMeasuredValue )
        {

          zclSensor_MeasuredValue = zclSensor_MeasuredValue + 100;  // considering using whole number value
#ifdef BDB_REPORTING
      Req.attrID = ATTRID_TEMPERATURE_MEASUREMENT_MEASURED_VALUE;
          Req.cluster = ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT;
          Req.endpoint = SENSOR_ENDPOINT;

          Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId,&Req);
#endif
        }
        else if ( zclSensor_MeasuredValue >= zclSampleTemperatureSensor_MaxMeasuredValue )
        {
          zclSensor_MeasuredValue = zclSampleTemperatureSensor_MaxMeasuredValue;
        }
      break;
      case CUI_INPUT_DOWN:
        // decrease the temperature
        if ( zclSensor_MeasuredValue > zclSampleTemperatureSensor_MinMeasuredValue )
        {
          zclSensor_MeasuredValue = zclSensor_MeasuredValue - 100;  // considering using whole number value
#ifdef BDB_REPORTING
      Req.attrID = ATTRID_TEMPERATURE_MEASUREMENT_MEASURED_VALUE;
          Req.cluster = ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT;
          Req.endpoint = SENSOR_ENDPOINT;

          Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId,&Req);
#endif
        }
        else if ( zclSensor_MeasuredValue <= zclSampleTemperatureSensor_MinMeasuredValue )
        {
          zclSensor_MeasuredValue = zclSampleTemperatureSensor_MinMeasuredValue;
        }
      break;
  }

  Util_ltoa( ( zclSensor_MeasuredValue / 100 ), (void *)(tempStr), 10 );
  tempStr[2] = 0;

  strcpy(_lines[1], "TEMP: ");
  strcat(_lines[1],tempStr);
  strcat(_lines[1], " C");

  if (_input != CUI_ITEM_PREVIEW)
  {
    strcpy(_lines[2], " SET LOCAL TEMP ");
  }
}

static void zclSampleTemperatureSensor_InitializeStatusLine(CUI_clientHandle_t gCuiHandle)
{
    /* Request Async Line for Light application Info */
    CUI_statusLineResourceRequest(gCuiHandle, "   APP Info"CUI_DEBUG_MSG_START"1"CUI_DEBUG_MSG_END, false, &gSampleTemperatureSensorInfoLine);

    zclSampleTemperatureSensor_UpdateStatusLine();
}


void zclSampleTemperatureSensor_UpdateStatusLine(void)
{
  char lineFormat[MAX_STATUS_LINE_VALUE_LEN] = {'\0'};

  strcpy(lineFormat, "["CUI_COLOR_YELLOW"Local Temperature"CUI_COLOR_RESET"] ");

  strcat(lineFormat, "%dC");

  CUI_statusLinePrintf(gCuiHandle, gSampleTemperatureSensorInfoLine, lineFormat, (zclSensor_MeasuredValue / 100));
}

#endif




