/*
  xsns_23_me007ula.ino - ME007 ultrasonic sensor support for Tasmota

  Copyright (C) 2022  Mathias Buder

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#define USE_ME007

#ifdef USE_ME007
/*********************************************************************************************\
 * ME007 - Ultrasonic distance sensor
 *
 * Code for ME007 family of ultrasonic distance sensors
 * References:
 * - https://wiki.dfrobot.com/Water-proof%20Ultrasonic%20Sensor%20(ULS)%20%20SKU:%20SEN0300
\*********************************************************************************************/

/*********************************************************************************************/
/* Includes*/
/*********************************************************************************************/
#include <TasmotaSerial.h>

/* Defines */
#define ME007_DEBUG                   /**< Enable/Disable debug-log output */
#define ME007_DEBUG_MSG_TAG           "ME007: "
#define ME007_MQTT_MSG_TAG            "ME007"

#define XSNS_23                       23
#define ME007_LOG_DATA_SIZE           180U /**< Byte */
#ifndef ME007_MAX_SENSOR_DISTANCE
#define ME007_MAX_SENSOR_DISTANCE     800U /**< Maximum measurement distance: 8 m */
#endif

/*********************************************************************************************/
/* Enums */
/*********************************************************************************************/
/**
 * @details I2C error types
 */
enum ME007_SHOW_TYPE
{
    ME007_SHOW_TYPE_JS = 0U,  /**< @details Domain log message tag string */
    ME007_SHOW_TYPE_WS
};

/*********************************************************************************************/
/* Structures */
/*********************************************************************************************/

/**
 * @details I2C error types
 */
struct
{
    bool  sensorDetected_b;
    float distance_f32;
    float temperature_f32;
} me007_data_s;

/*********************************************************************************************/
/* Global Variables */
/*********************************************************************************************/
TasmotaSerial* p_SerialIf = nullptr;



/*********************************************************************************************/
/* Function Definitions */
/*********************************************************************************************/
void Me007Init( void )
{
    DEBUG_SENSOR_LOG( PSTR( ME007_DEBUG_MSG_TAG "Initializing ...") );

    /* Check if sensor pins are selected/used in web-interface */
    if(    ( PinUsed( GPIO_ME007_TRIG ) == false )
        || ( PinUsed( GPIO_ME007_RX )   == false ) )
    {
        DEBUG_SENSOR_LOG( PSTR( ME007_DEBUG_MSG_TAG "Serial interface not configured" ) );
        return;
    }

    DEBUG_SENSOR_LOG( PSTR( ME007_DEBUG_MSG_TAG "Using GPIOs: %i/%i" ), GPIO_ME007_TRIG, GPIO_ME007_RX );



  // GPIO_ME007_TX, GPIO_ME007_RX,        // ME007 Serial interface
}



void Me007ReadValue( void )
{

}


/**
 * @details This function performs a read/write test on the specified I2C device to make sure the
 * low-level interface is working correctly.
 * @param[in] error_t I2C error code
 */
void Me007Show( uint8_t type_u8 )
{
#ifdef ME007_DEBUG
    AddLog( LOG_LEVEL_DEBUG_MORE, PSTR( "ME007:me007Show: %s: %f" ), type_u8 == ME007_SHOW_TYPE_JS ? "JASON" : "WEBSERVER", me007_data_s.distance_f32 );
#endif

    switch ( type_u8 )
    {
    case ME007_SHOW_TYPE_JS:
        ResponseAppend_P( PSTR( ",\"ME007_MQTT_MSG_TAG\":{\"" D_JSON_DISTANCE "\":%1_f}" ), &me007_data_s.distance_f32 );
        ResponseAppend_P( PSTR( ",\"ME007_MQTT_MSG_TAG\":{\"" D_JSON_TEMPERATURE "\":%1_f}" ), &me007_data_s.temperature_f32 );
#ifdef USE_DOMOTICZ
        if ( 0U == TasmotaGlobal.tele_period )
        {
            DomoticzFloatSensor( DZ_COUNT, me007_data_s.distance_f32 );     /**< @details Send distance as Domoticz counter value */
            DomoticzFloatSensor( DZ_TEMP, me007_data_s.temperature_f32 );   /**< @details Send distance as Domoticz temperature value */
        }
#endif   /* USE_DOMOTICZ */
        break;
#ifdef USE_WEBSERVER
    case ME007_SHOW_TYPE_WS:
        WSContentSend_PD( HTTP_SNS_F_DISTANCE_CM,
                          "ME007",
                          &me007_data_s.distance_f32 );

        WSContentSend_PD( HTTP_SNS_F_TEMP,
                          "ME007", Settings->flag2.temperature_resolution,
                          &me007_data_s.temperature_f32,
                          D_UNIT_CELSIUS );
        break;
#endif   /* USE_WEBSERVER */

    default: /* Should never happen */
        break;
    }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns23( uint32_t function )
{
    bool result_b = false;

    switch ( function )
    {
    case FUNC_INIT:
        Me007Init();
    break;
    
    case FUNC_EVERY_SECOND:
        
        result_b = true;
        break;
    case FUNC_JSON_APPEND:
        Me007Show( ME007_SHOW_TYPE_JS );
        break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
        //me007Show( ME007_SHOW_TYPE_WS );
        break;
#endif   // USE_WEBSERVER
    }
    return result_b;
}

#endif   // USE_ME007
