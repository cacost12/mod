/*******************************************************************************
*
* FILE: 
* 		sensor.c
*
* DESCRIPTION: 
*       Contains functions that provide a unified interface to all sensor APIs
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <string.h>
#include <stdbool.h>
#include <math.h>


/*------------------------------------------------------------------------------
 MCU Pins 
------------------------------------------------------------------------------*/
#if   defined( BASE_FLIGHT_COMPUTER        )
	#include "zav_pin_defines_A0001.h"
#elif defined( FULL_FLIGHT_COMPUTER        )
	#include "zav_pin_defines_A0002.h"
#elif defined( LEGACY_FLIGHT_COMPUTER      )
	#include "zav_pin_defines_A0003.h"
#elif defined( LEGACY_FLIGHT_COMPUTER_LITE )
	#include "zav_pin_defines_A0004.h"
#else
	#error "No sensor module compatible device specified in Makefile"
#endif 


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "baro.h"
#if defined( FULL_FLIGHT_COMPUTER ) || defined( LEGACY_FLIGHT_COMPUTER )
	#include "imu.h"
#endif

#include "sensor.h"


/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/

/* Hash table of sensor readout sizes and offsets */
static SENSOR_DATA_SIZE_OFFSETS sensor_size_offsets_table[ NUM_SENSORS ];


/*------------------------------------------------------------------------------
 API Functions 
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_init                                                            *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Initialize the sensor module                                           *
*                                                                              *
*******************************************************************************/
void sensor_init 
	(
	void
	)
{
/* Setup the sensor id hash table */
#if   defined( BASE_FLIGHT_COMPUTER )
	/* Sensor offsets */
	sensor_size_offsets_table[ 0 ].offset = 0; /* SENSOR_PRES  */
	sensor_size_offsets_table[ 1 ].offset = 4; /* SENSOR_TEMP  */
	sensor_size_offsets_table[ 2 ].offset = 8; /* SENSOR_VBAT  */

	/* Sensor Sizes   */
	sensor_size_offsets_table[ 0 ].size   = 4;  /* SENSOR_PRES  */
	sensor_size_offsets_table[ 1 ].size   = 4;  /* SENSOR_TEMP  */
	sensor_size_offsets_table[ 2 ].size   = 4;  /* SENSOR_VBAT  */

#elif defined( FULL_FLIGHT_COMPUTER )
	/* Sensor offsets */
	sensor_size_offsets_table[ 0  ].offset = 0;  /* SENSOR_ACCX  */
	sensor_size_offsets_table[ 1  ].offset = 2;  /* SENSOR_ACCY  */
	sensor_size_offsets_table[ 2  ].offset = 4;  /* SENSOR_ACCZ  */
	sensor_size_offsets_table[ 3  ].offset = 6;  /* SENSOR_GYROX */
	sensor_size_offsets_table[ 4  ].offset = 8;  /* SENSOR_GYROY */
	sensor_size_offsets_table[ 5  ].offset = 10; /* SENSOR_GYROZ */
	sensor_size_offsets_table[ 6  ].offset = 12; /* SENSOR_MAGX  */
	sensor_size_offsets_table[ 7  ].offset = 14; /* SENSOR_MAGY  */
	sensor_size_offsets_table[ 8  ].offset = 16; /* SENSOR_MAGZ  */
	sensor_size_offsets_table[ 9  ].offset = 18; /* SENSOR_IMUT  */
	sensor_size_offsets_table[ 10 ].offset = 20; /* SENSOR_PRES  */
	sensor_size_offsets_table[ 11 ].offset = 24; /* SENSOR_TEMP  */
	sensor_size_offsets_table[ 12 ].offset = 28; /* SENSOR_VBAT  */

	/* Sensor Sizes   */
	sensor_size_offsets_table[ 0  ].size   = 2;  /* SENSOR_ACCX  */
	sensor_size_offsets_table[ 1  ].size   = 2;  /* SENSOR_ACCY  */
	sensor_size_offsets_table[ 2  ].size   = 2;  /* SENSOR_ACCZ  */
	sensor_size_offsets_table[ 3  ].size   = 2;  /* SENSOR_GYROX */
	sensor_size_offsets_table[ 4  ].size   = 2;  /* SENSOR_GYROY */
	sensor_size_offsets_table[ 5  ].size   = 2;  /* SENSOR_GYROZ */
	sensor_size_offsets_table[ 6  ].size   = 2;  /* SENSOR_MAGX  */
	sensor_size_offsets_table[ 7  ].size   = 2;  /* SENSOR_MAGY  */
	sensor_size_offsets_table[ 8  ].size   = 2;  /* SENSOR_MAGZ  */
	sensor_size_offsets_table[ 9  ].size   = 2;  /* SENSOR_IMUT  */
	sensor_size_offsets_table[ 10 ].size   = 4;  /* SENSOR_PRES  */
	sensor_size_offsets_table[ 11 ].size   = 4;  /* SENSOR_TEMP  */
	sensor_size_offsets_table[ 12 ].size   = 4;  /* SENSOR_VBAT  */

#elif defined( LEGACY_FLIGHT_COMPUTER )
	/* Sensor offsets */
	sensor_size_offsets_table[ 0  ].offset = 0;  /* SENSOR_ACCX  */
	sensor_size_offsets_table[ 1  ].offset = 2;  /* SENSOR_ACCY  */
	sensor_size_offsets_table[ 2  ].offset = 4;  /* SENSOR_ACCZ  */
	sensor_size_offsets_table[ 3  ].offset = 6;  /* SENSOR_GYROX */
	sensor_size_offsets_table[ 4  ].offset = 8;  /* SENSOR_GYROY */
	sensor_size_offsets_table[ 5  ].offset = 10; /* SENSOR_GYROZ */
	sensor_size_offsets_table[ 6  ].offset = 12; /* SENSOR_MAGX  */
	sensor_size_offsets_table[ 7  ].offset = 14; /* SENSOR_MAGY  */
	sensor_size_offsets_table[ 8  ].offset = 16; /* SENSOR_MAGZ  */
	sensor_size_offsets_table[ 9  ].offset = 18; /* SENSOR_IMUT  */
	sensor_size_offsets_table[ 10 ].offset = 20; /* SENSOR_PRES  */
	sensor_size_offsets_table[ 11 ].offset = 24; /* SENSOR_TEMP  */

	/* Sensor Sizes   */
	sensor_size_offsets_table[ 0  ].size   = 2;  /* SENSOR_ACCX  */
	sensor_size_offsets_table[ 1  ].size   = 2;  /* SENSOR_ACCY  */
	sensor_size_offsets_table[ 2  ].size   = 2;  /* SENSOR_ACCZ  */
	sensor_size_offsets_table[ 3  ].size   = 2;  /* SENSOR_GYROX */
	sensor_size_offsets_table[ 4  ].size   = 2;  /* SENSOR_GYROY */
	sensor_size_offsets_table[ 5  ].size   = 2;  /* SENSOR_GYROZ */
	sensor_size_offsets_table[ 6  ].size   = 2;  /* SENSOR_MAGX  */
	sensor_size_offsets_table[ 7  ].size   = 2;  /* SENSOR_MAGY  */
	sensor_size_offsets_table[ 8  ].size   = 2;  /* SENSOR_MAGZ  */
	sensor_size_offsets_table[ 9  ].size   = 2;  /* SENSOR_IMUT  */
	sensor_size_offsets_table[ 10 ].size   = 4;  /* SENSOR_PRES  */
	sensor_size_offsets_table[ 11 ].size   = 4;  /* SENSOR_TEMP  */

#elif defined( LEGACY_FLIGHT_COMPUTER_LITE )
	/* Sensor offsets */
	sensor_size_offsets_table[ 0 ].offset = 0; /* SENSOR_PRES  */
	sensor_size_offsets_table[ 1 ].offset = 4; /* SENSOR_TEMP  */

	/* Sensor Sizes   */
	sensor_size_offsets_table[ 0 ].size   = 4;  /* SENSOR_PRES  */
	sensor_size_offsets_table[ 1 ].size   = 4;  /* SENSOR_TEMP  */
#else
	#error "No sensor compatible device specified in Makefile"
#endif

} /* sensor_init */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_dump                                                            *
*                                                                              *
* DESCRIPTION:                                                                 *
*       reads from all sensors and fill in the sensor data structure           *
*                                                                              *
*******************************************************************************/
SENSOR_STATUS sensor_dump 
	(
    SENSOR_DATA* sensor_data_ptr /* Out: Pointer to output struct */ 
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/

#if   defined( BASE_FLIGHT_COMPUTER )
	BARO_STATUS     press_status;           /* Baro Sensor status codes    */
	BARO_STATUS     temp_status;

#elif defined( FULL_FLIGHT_COMPUTER )
	IMU_STATUS      accel_status;           /* IMU sensor status codes     */       
	IMU_STATUS      gyro_status;
	IMU_STATUS      mag_status; 
	BARO_STATUS     press_status;           /* Baro Sensor status codes    */
	BARO_STATUS     temp_status;

#elif defined( LEGACY_FLIGHT_COMPUTER )
	IMU_STATUS      accel_status;           /* IMU sensor status codes     */       
	IMU_STATUS      gyro_status;
	IMU_STATUS      mag_status; 
	BARO_STATUS     press_status;           /* Baro Sensor status codes    */
	BARO_STATUS     temp_status;

#elif defined( LEGACY_FLIGHT_COMPUTER_LITE )
	BARO_STATUS     press_status;           /* Baro Sensor status codes    */
	BARO_STATUS     temp_status;

#endif

/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
#if  defined( BASE_FLIGHT_COMPUTER )
	press_status = BARO_OK;           
	temp_status  = BARO_OK;

#elif defined( FULL_FLIGHT_COMPUTER )
	accel_status = IMU_OK;         
	gyro_status  = IMU_OK;
	mag_status   = IMU_OK; 
	press_status = BARO_OK;           
	temp_status  = BARO_OK;

#elif defined( LEGACY_FLIGHT_COMPUTER )
	accel_status = IMU_OK;         
	gyro_status  = IMU_OK;
	mag_status   = IMU_OK; 
	press_status = BARO_OK;           
	temp_status  = BARO_OK;

#elif defined( LEGACY_FLIGHT_COMPUTER_LITE )
	press_status = BARO_OK;           
	temp_status  = BARO_OK;

#endif


/*------------------------------------------------------------------------------
 Call sensor API functions 
------------------------------------------------------------------------------*/

/* Poll Sensors  */
#if   defined( BASE_FLIGHT_COMPUTER )
	temp_status  = baro_get_temp    ( &(sensor_data_ptr -> baro_temp     ) );
	press_status = baro_get_pressure( &(sensor_data_ptr -> baro_pressure ) );

#elif defined( FULL_FLIGHT_COMPUTER )
	accel_status = imu_get_accel_xyz( &(sensor_data_ptr->imu_data) ); 
	gyro_status  = imu_get_gyro_xyz ( &(sensor_data_ptr->imu_data) );
	mag_status   = imu_get_mag_xyz  ( &(sensor_data_ptr->imu_data) );
	sensor_data_ptr -> imu_data.temp = 0;     // Figure out what to do with this 
											  // readout, temporarily being used 
											  // as struct padding
	temp_status  = baro_get_temp    ( &(sensor_data_ptr -> baro_temp     ) );
	press_status = baro_get_pressure( &(sensor_data_ptr -> baro_pressure ) );

#elif defined( LEGACY_FLIGHT_COMPUTER )
	accel_status = imu_get_accel_xyz( &(sensor_data_ptr->imu_data) ); 
	gyro_status  = imu_get_gyro_xyz ( &(sensor_data_ptr->imu_data) );
	mag_status   = imu_get_mag_xyz  ( &(sensor_data_ptr->imu_data) );
	sensor_data_ptr -> imu_data.temp = 0;     // Figure out what to do with this 
											  // readout, temporarily being used 
											  // as struct padding
	temp_status  = baro_get_temp    ( &(sensor_data_ptr -> baro_temp     ) );
	press_status = baro_get_pressure( &(sensor_data_ptr -> baro_pressure ) );

#elif defined( LEGACY_FLIGHT_COMPUTER_LITE )
	temp_status  = baro_get_temp    ( &(sensor_data_ptr -> baro_temp     ) );
	press_status = baro_get_pressure( &(sensor_data_ptr -> baro_pressure ) );

#endif


/*------------------------------------------------------------------------------
 Set command status from sensor API returns 
------------------------------------------------------------------------------*/
#if   defined( BASE_FLIGHT_COMPUTER )
	if ( press_status != BARO_OK || temp_status  != BARO_OK  )
		{
		return SENSOR_BARO_ERROR;
		}

#elif defined( FULL_FLIGHT_COMPUTER )
	if      ( accel_status != IMU_OK )
		{
		return SENSOR_ACCEL_ERROR;
		}
	else if ( gyro_status  != IMU_OK )
		{
		return SENSOR_GYRO_ERROR;
		}
	else if ( mag_status   != IMU_OK )
		{
		return SENSOR_MAG_ERROR;	
		}
	else if ( press_status != BARO_OK ||
			temp_status  != BARO_OK  )
		{
		return SENSOR_BARO_ERROR;
		}

#elif defined( LEGACY_FLIGHT_COMPUTER )
	if      ( accel_status != IMU_OK )
		{
		return SENSOR_ACCEL_ERROR;
		}
	else if ( gyro_status  != IMU_OK )
		{
		return SENSOR_GYRO_ERROR;
		}
	else if ( mag_status   != IMU_OK )
		{
		return SENSOR_MAG_ERROR;	
		}
	else if ( press_status != BARO_OK || temp_status  != BARO_OK  )
		{
		return SENSOR_BARO_ERROR;
		}

#elif defined( LEGACY_FLIGHT_COMPUTER_LITE )
	if ( press_status != BARO_OK || temp_status  != BARO_OK  )
		{
		return SENSOR_BARO_ERROR;
		}

#endif /* #elif defined( ENGINE_CONTROLLER )*/

return SENSOR_OK;
} /* sensor_dump */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_poll                                                            *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Poll specific sensors on the board                                     *
*                                                                              *
*******************************************************************************/
SENSOR_STATUS sensor_poll
	(
	SENSOR_DATA* sensor_data_ptr, /* Data Export target               */
	SENSOR_ID*   sensor_ids_ptr , /* Array containing sensor IDS      */
	uint8_t      num_sensors      /* Number of sensors to poll        */
	)
{
/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
SENSOR_ID  sensor_id;        /* ID of sensor currently being polled */
SENSOR_ID* sensor_id_ptr;    /* Pointer to sensor id                */

/* Module return codes */
#if   defined( BASE_FLIGHT_COMPUTER )
	BARO_STATUS     baro_status;     /* Baro module return codes  */

#elif defined( FULL_FLIGHT_COMPUTER )
	IMU_STATUS      imu_status;      /* IMU Module return codes   */ 
	BARO_STATUS     baro_status;     /* Baro module return codes  */

#elif defined( LEGACY_FLIGHT_COMPUTER   )
	IMU_STATUS      imu_status;      /* IMU Module return codes   */ 
	BARO_STATUS     baro_status;     /* Baro module return codes  */

#elif defined( FLIGHT_COMPUTER_LITE )
	BARO_STATUS     baro_status;     /* Baro module return codes  */

#endif

/* Sensor poll memory to prevent multiple calls to same API function */
bool imu_accel_read;
bool imu_gyro_read;
bool imu_mag_read;

/*------------------------------------------------------------------------------
 Initializations 
------------------------------------------------------------------------------*/
sensor_id_ptr     = sensor_ids_ptr;
sensor_id         = *(sensor_id_ptr   );

/* Module return codes */
#if   defined( BASE_FLIGHT_COMPUTER )
	baro_status   = BARO_OK;

#elif defined( FULL_FLIGHT_COMPUTER )
	imu_status    = IMU_OK;
	baro_status   = BARO_OK;

#elif defined( LEGACY_FLIGHT_COMPUTER )
	imu_status    = IMU_OK;
	baro_status   = BARO_OK;

#elif defined( LEGACY_FLIGHT_COMPUTER_LITE )
	baro_status   = BARO_OK;

#endif

/* Sensor poll memory */
imu_accel_read = false;
imu_gyro_read  = false;
imu_mag_read   = false;


/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Iterate over each sensor readout */
for ( int i = 0; i < num_sensors; ++i )
	{
	
	/* Poll sensor */
	switch ( sensor_id )
		{
		#if defined( FULL_FLIGHT_COMPUTER ) || defined( LEGACY_FLIGHT_COMPUTER )
			case SENSOR_ACCX:
				{
				if ( !imu_accel_read )
					{
					imu_status = imu_get_accel_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_ACCEL_ERROR;
						}
					imu_accel_read = true;
					}
				break;
				}

			case SENSOR_ACCY:
				{
				if ( !imu_accel_read )
					{
					imu_status = imu_get_accel_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_ACCEL_ERROR;
						}
					imu_accel_read = true;
					}
				break;
				}

			case SENSOR_ACCZ:
				{
				if ( !imu_accel_read )
					{
					imu_status = imu_get_accel_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_ACCEL_ERROR;
						}
					imu_accel_read = true;
					}
				break;
				}

			case SENSOR_GYROX:
				{
				if ( !imu_gyro_read )
					{
					imu_status = imu_get_gyro_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_GYRO_ERROR;
						}
					imu_gyro_read = true;
					}
				break;
				}

			case SENSOR_GYROY:
				{
				if ( !imu_gyro_read )
					{
					imu_status = imu_get_gyro_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_GYRO_ERROR;
						}
					imu_gyro_read = true;
					}
				break;
				}

			case SENSOR_GYROZ:
				{
				if ( !imu_gyro_read )
					{
					imu_status = imu_get_gyro_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_GYRO_ERROR;
						}
					imu_gyro_read = true;
					}
				break;
				}

			case SENSOR_MAGX:
				{
				if ( !imu_mag_read )
					{
					imu_status = imu_get_mag_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_GYRO_ERROR;
						}
					imu_mag_read = true;
					}
				break;
				}

			case SENSOR_MAGY:
				{
				if ( !imu_mag_read )
					{
					imu_status = imu_get_mag_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_GYRO_ERROR;
						}
					imu_mag_read = true;
					}
				break;
				}

			case SENSOR_MAGZ:
				{
				if ( !imu_mag_read )
					{
					imu_status = imu_get_mag_xyz( &( sensor_data_ptr -> imu_data ) );
					if ( imu_status != IMU_OK )
						{
						return SENSOR_GYRO_ERROR;
						}
					imu_mag_read = true;
					}
				break;
				}

			case SENSOR_IMUT:
				{
				sensor_data_ptr -> imu_data.temp = 0;
				break;
				}
		#endif /* #if defined( FLIGHT_COMPUTER ) */

		case SENSOR_PRES:
			{
			baro_status = baro_get_temp(     &( sensor_data_ptr -> baro_temp     ) );
			if ( baro_status != BARO_OK )
				{
				return SENSOR_BARO_ERROR;
				}
			baro_status = baro_get_pressure( &( sensor_data_ptr -> baro_pressure ) );
			if ( baro_status != BARO_OK )
				{
				return SENSOR_BARO_ERROR;
				}
			break;
			}

		case SENSOR_TEMP:
			{
			baro_status = baro_get_temp( &( sensor_data_ptr -> baro_temp ) );
			if ( baro_status != BARO_OK )
				{
				return SENSOR_BARO_ERROR;
				}
			break;
			}

		default:
			{
			/* Unrecognized sensor id */
			return SENSOR_UNRECOGNIZED_SENSOR_ID; 
			}
		} /* switch( sensor_id ) */

		/* Go to next sensor */
		sensor_id_ptr++;
		sensor_id        = *(sensor_id_ptr   );

	} /*  while( i < num_sensors ) */

return SENSOR_OK;
} /* sensor_poll */


/*------------------------------------------------------------------------------
 Internal procedures 
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   *
* 		sensor_map                                                             *
*                                                                              *
* DESCRIPTION:                                                                 *
*       Sensor ID to size and pointer mapping                                  *
*                                                                              *
*******************************************************************************/
void sensor_map
	(
	SENSOR_DATA* sensor_data_ptr, /* In:  Base pointer to sensor data   */
	SENSOR_ID    sensor_id      , /* In:  Sensor id                    */
	uint8_t**    sensorid_pptr  , /* Out: Pointer to sensor readout in 
	                                      sensor_data_ptr              */
	size_t*      sensor_size_ptr  /* Out: Size of readout in bytes     */
	)
{
/* Lookup sensor offset and size from table */
*sensor_size_ptr = sensor_size_offsets_table[ sensor_id ].size;
*sensorid_pptr   = ( (uint8_t*) sensor_data_ptr ) + 
                   sensor_size_offsets_table[ sensor_id ].offset;

} /*  sensor_map */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/