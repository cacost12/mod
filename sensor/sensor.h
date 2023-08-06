/*******************************************************************************
*
* FILE: 
* 		sensor.h
*
* DESCRIPTION: 
*       Contains functions that provide a unified interface to all sensor APIs
*
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SENSOR_H
#define SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"


/*------------------------------------------------------------------------------
Includes 
------------------------------------------------------------------------------*/

/* GCC requires stdint.h for uint_t types */
#ifdef UNIT_TEST
	#include <stdint.h>
#endif


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Max allowed number of sensors for polling */
#define SENSOR_MAX_NUM_POLL     ( 5    )

#if   defined( BASE_FLIGHT_COMPUTER )
	#define NUM_SENSORS         ( 3  )
	#define SENSOR_DATA_SIZE    ( 12 )

#elif defined( FULL_FLIGHT_COMPUTER )
	#define NUM_SENSORS         ( 13 )
	#define IMU_DATA_SIZE       ( 20 )
	#define SENSOR_DATA_SIZE    ( 32 )

#elif defined( LEGACY_FLIGHT_COMPUTER )
	#define NUM_SENSORS         ( 12 )
	#define IMU_DATA_SIZE       ( 20 )
	#define SENSOR_DATA_SIZE    ( 28 )

#elif defined( LEGACY_FLIGHT_COMPUTER_LITE )
	#define NUM_SENSORS         ( 2  )
	#define SENSOR_DATA_SIZE    ( 8  )

#else
	#error "No sensor compatible device specified in Makefile"
#endif

/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Sensor status return codes */
typedef enum 
	{
    SENSOR_OK = 0                ,
	SENSOR_UNRECOGNIZED_OP       ,
	SENSOR_UNSUPPORTED_OP        ,
	SENSOR_IMU_FAIL              ,
	SENSOR_ACCEL_ERROR           ,
    SENSOR_GYRO_ERROR            ,
	SENSOR_MAG_ERROR             ,
	SENSOR_BARO_ERROR            ,
	SENSOR_UNRECOGNIZED_SENSOR_ID,
	SENSOR_POLL_FAIL_TO_START    ,
	SENSOR_POLL_FAIL             ,
	SENSOR_POLL_UNRECOGNIZED_CMD ,
	SENSOR_ADC_POLL_ERROR        ,
    SENSOR_FAIL   
    } SENSOR_STATUS;

/* Sensor poll command codes */
typedef enum
	{
	SENSOR_POLL_START   = 0xF3,
	SENSOR_POLL_REQUEST = 0x51,
	SENSOR_POLL_WAIT    = 0x44,
	SENSOR_POLL_RESUME  = 0xEF,
	SENSOR_POLL_STOP    = 0x74
	} SENSOR_POLL_CMD;

/* Sensor idenification code instance*/
typedef uint8_t SENSOR_ID;

/* Sensor Names/codes */
typedef enum
	{
	#if  defined( BASE_FLIGHT_COMPUTER  )
		SENSOR_PRES  = 0x00,
		SENSOR_TEMP  = 0x01,
		SENSOR_VBAT  = 0x02

	#elif defined( FULL_FLIGHT_COMPUTER )
		SENSOR_ACCX  = 0x00,
		SENSOR_ACCY  = 0x01,
		SENSOR_ACCZ  = 0x02,
		SENSOR_GYROX = 0x03,
		SENSOR_GYROY = 0x04,
		SENSOR_GYROZ = 0x05,
		SENSOR_MAGX  = 0x06,
		SENSOR_MAGY  = 0x07,
		SENSOR_MAGZ  = 0x08,
		SENSOR_IMUT  = 0x09,
		SENSOR_PRES  = 0x0A,
		SENSOR_TEMP  = 0x0B,
		SENSOR_VBAT  = 0x0C

	#elif defined( LEGACY_FLIGHT_COMPUTER )
		SENSOR_ACCX  = 0x00,
		SENSOR_ACCY  = 0x01,
		SENSOR_ACCZ  = 0x02,
		SENSOR_GYROX = 0x03,
		SENSOR_GYROY = 0x04,
		SENSOR_GYROZ = 0x05,
		SENSOR_MAGX  = 0x06,
		SENSOR_MAGY  = 0x07,
		SENSOR_MAGZ  = 0x08,
		SENSOR_IMUT  = 0x09,
		SENSOR_PRES  = 0x0A,
		SENSOR_TEMP  = 0x0B

	#elif defined( LEGACY_FLIGHT_COMPUTER_LITE )
		SENSOR_PRES  = 0x00,
		SENSOR_TEMP  = 0x01

	#else
		#error "No sensor compatible board specified in Makefile"
	#endif
	} SENSOR_IDS;

/* Sensor Data */
typedef struct _SENSOR_DATA 
	{
	#if   defined( BASE_FLIGHT_COMPUTER )
		float     baro_pressure;
		float     baro_temp;
		float     battery_voltage;

	#elif defined( FULL_FLIGHT_COMPUTER )
		IMU_DATA imu_data;
		float    baro_pressure;
		float    baro_temp;	
		float    battery_voltage;
	
	#elif defined( LEGACY_FLIGHT_COMPUTER )
		IMU_DATA imu_data;
		float    baro_pressure;
		float    baro_temp;	

	#elif defined( LEGACY_FLIGHT_COMPUTER_LITE )
		float     baro_pressure;
		float     baro_temp;

	#else
		#error "No sensor compatible board specified in Makefile"

	#endif /* #if defined( BASE_FLIGHT_COMPUTER ) */
	} SENSOR_DATA;

/* Sensor Data sizes and offsets */
typedef struct SENSOR_DATA_SIZE_OFFSETS
	{
	uint8_t offset;  /* Offset of sensor readout in SENSOR_DATA struct  */
	size_t  size;    /* Size of readout in bytes                        */
	} SENSOR_DATA_SIZE_OFFSETS;


/*------------------------------------------------------------------------------
 Public Function Prototypes 
------------------------------------------------------------------------------*/

/* Initialize the sensor module */
void sensor_init 
	(
	void
	);

/* Poll specific sensors on the board */
SENSOR_STATUS sensor_poll
	(
	SENSOR_DATA* sensor_data_ptr,
	SENSOR_ID* sensor_ids_ptr  ,
	uint8_t    num_sensors
	);

/* Dump all sensor readings to console */
SENSOR_STATUS sensor_dump
	(
    SENSOR_DATA* sensor_data_ptr 
    );

/* Sensor ID to size and pointer mapping */
void sensor_map
	(
	SENSOR_DATA* sensor_data_ptr,
	SENSOR_ID    sensor_id      ,
	uint8_t**    sensorid_pptr  ,
	size_t*      sensor_size
	);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/