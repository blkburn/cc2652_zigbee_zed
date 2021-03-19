/** \mainpage Driver Overview
  *
  * \section section_drv_info Driver Information
  * This Sensor Controller Interface driver has been generated by the Texas Instruments Sensor Controller
  * Studio tool:
  * - <b>Project name</b>:     SA AHT10 I2C Temperature and Humidity Sensor
  * - <b>Project file</b>:     C:/Users/BigBob/sensor_controller/aht10_cc2652.scp
  * - <b>Code prefix</b>:      -
  * - <b>Operating system</b>: TI-RTOS
  * - <b>Tool version</b>:     2.7.0.155
  * - <b>Tool patches</b>:     None
  * - <b>Target chip</b>:      CC2652R1F, package QFN48 7x7 RGZ, revision E (2.1)
  * - <b>Created</b>:          2021-03-04 21:30:12.252
  * - <b>Computer</b>:         DESKTOP-MMLJVDE
  * - <b>User</b>:             BigBob
  *
  * No user-provided resource definitions were used to generate this driver.
  *
  * No user-provided procedure definitions were used to generate this driver.
  *
  * Do not edit the generated source code files other than temporarily for debug purposes. Any
  * modifications will be overwritten by the Sensor Controller Studio when generating new output.
  *
  * \section section_drv_modules Driver Modules
  * The driver is divided into three modules:
  * - \ref module_scif_generic_interface, providing the API for:
  *     - Initializing and uninitializing the driver
  *     - Task control (for starting, stopping and executing Sensor Controller tasks)
  *     - Task data exchange (for producing input data to and consume output data from Sensor Controller
  *       tasks)
  * - \ref module_scif_driver_setup, containing:
  *     - The AUX RAM image (Sensor Controller code and data)
  *     - I/O mapping information
  *     - Task data structure information
  *     - Driver setup data, to be used in the driver initialization
  *     - Project-specific functionality
  * - \ref module_scif_osal, for flexible OS support:
  *     - Interfaces with the selected operating system
  *
  * It is possible to use output from multiple Sensor Controller Studio projects in one application. Only
  * one driver setup may be active at a time, but it is possible to switch between these setups. When
  * using this option, there is one instance of the \ref module_scif_generic_interface and
  * \ref module_scif_osal modules, and multiple instances of the \ref module_scif_driver_setup module.
  * This requires that:
  * - The outputs must be generated using the same version of Sensor Controller Studio
  * - The outputs must use the same operating system
  * - The outputs must use different source code prefixes (inserted into all globals of the
  *   \ref module_scif_driver_setup)
  *
  *
  * \section section_project_info Project Description
  * No description entered
  *
  *
  * \subsection section_io_mapping I/O Mapping
  * Task I/O functions are mapped to the following pins:
  * - I2C Temp and Humidity Sensor:
  *     - <b>I2C SCL</b>: DIO4
  *     - <b>I2C SDA</b>: DIO5
  *
  *
  * \section section_task_info Task Description(s)
  * This driver supports the following task(s):
  *
  *
  * \subsection section_task_desc_i2c_temp_and_humidity_sensor I2C Temp and Humidity Sensor
  * No description entered
  *
  */




/** \addtogroup module_scif_driver_setup Driver Setup
  *
  * \section section_driver_setup_overview Overview
  *
  * This driver setup instance has been generated for:
  * - <b>Project name</b>:     SA AHT10 I2C Temperature and Humidity Sensor
  * - <b>Code prefix</b>:      -
  *
  * The driver setup module contains the generated output from the Sensor Controller Studio project:
  * - Location of task control and scheduling data structures in AUX RAM
  * - The AUX RAM image, and the size the image
  * - Task data structure information (location, size and buffer count)
  * - I/O pin mapping translation table
  * - Task resource initialization and uninitialization functions
  * - Hooks for run-time logging
  *
  * @{
  */
#ifndef SCIF_H
#define SCIF_H

#include <Application/sensor_controller/AHT10/scif_framework.h>
#include <Application/sensor_controller/AHT10/scif_osal_tirtos.h>
#include <stdint.h>
#include <stdbool.h>


/// Target chip name
#define SCIF_TARGET_CHIP_NAME_CC2652R1F
/// Target chip package
#define SCIF_TARGET_CHIP_PACKAGE_QFN48_7X7_RGZ

/// Number of tasks implemented by this driver
#define SCIF_TASK_COUNT 1

/// I2C Temp and Humidity Sensor: Task ID
#define SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_TASK_ID 0


/// I2C Temp and Humidity Sensor: HDC2080 I2C address, shifted left one bit
#define SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_I2C_ADDR 112
/// I2C Temp and Humidity Sensor I/O mapping: I2C SCL
#define SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_DIO_I2C_SCL 4
/// I2C Temp and Humidity Sensor I/O mapping: I2C SDA
#define SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_DIO_I2C_SDA 5


// All shared data structures in AUX RAM need to be packed
#pragma pack(push, 2)


/// I2C Temp and Humidity Sensor: Task output data structure
typedef struct {
    uint16_t byte1;  ///< 
    uint16_t byte2;  ///< 
    uint16_t byte3;  ///< 
    uint16_t byte4;  ///< 
    uint16_t byte5;  ///< 
    uint16_t status; ///< 
} SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_OUTPUT_T;


/// I2C Temp and Humidity Sensor: Task state structure
typedef struct {
    uint16_t i2cStatus; ///< I2C master status
} SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_STATE_T;


/// Sensor Controller task data (configuration, input buffer(s), output buffer(s) and internal state)
typedef struct {
    struct {
        SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_OUTPUT_T output;
        SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_STATE_T state;
    } i2cTempAndHumiditySensor;
} SCIF_TASK_DATA_T;

/// Sensor Controller task generic control (located in AUX RAM)
#define scifTaskData    (*((volatile SCIF_TASK_DATA_T*) 0x400E015C))


// Initialized internal driver data, to be used in the call to \ref scifInit()
extern const SCIF_DATA_T scifDriverSetup;


// Restore previous struct packing setting
#pragma pack(pop)


// AUX I/O re-initialization functions
void scifReinitTaskIo(uint32_t bvTaskIds);


// RTC-based tick generation control
void scifStartRtcTicks(uint32_t tickStart, uint32_t tickPeriod);
void scifStartRtcTicksNow(uint32_t tickPeriod);
void scifStopRtcTicks(void);


#endif
//@}


// Generated by DESKTOP-MMLJVDE at 2021-03-04 21:30:12.252