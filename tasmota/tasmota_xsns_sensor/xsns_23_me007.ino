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

#ifdef USE_ME007
/*********************************************************************************************\
 * ME007 - Ultrasonic distance sensor
 *
 * Code for ME007 family of ultrasonic distance sensors
 * References:
 * - https://wiki.dfrobot.com/Water-proof%20Ultrasonic%20Sensor%20(ULS)%20%20SKU:%20SEN0300
\*********************************************************************************************/
/* Includes*/
#include <TasmotaSerial.h>

/* Defines */

#define ME007_DEBUG                   /* Enable/Disable debug-log output */

#define XSNS_23                       23
#define ME007_LOG_DATA_SIZE           180U /* Byte */
#ifndef ME007_MAX_SENSOR_DISTANCE
#define ME007_MAX_SENSOR_DISTANCE     800 /* Maximum measurement distance: 8 m */
#endif


struct
{
    float distance_f32;
    float temperature_f32;
} me007_data_s;

void me007Show( bool json )
{
#ifdef ME007_DEBUG
    AddLog( LOG_LEVEL_DEBUG_MORE, PSTR("ME007:me007Show: %s: %f"), json == 1U ? "JASON" : "WEBSERVER", me007_data_s.distance_f32 );
#endif
    if ( json )
    {
        ResponseAppend_P( PSTR( ",\"ME007\":{\"" D_JSON_DISTANCE "\":%1_f}" ), &me007_data_s.distance_f32 );
        ResponseAppend_P( PSTR( ",\"ME007\":{\"" D_JSON_TEMPERATURE "\":%1_f}" ), &me007_data_s.temperature_f32 );
#ifdef USE_DOMOTICZ
        if ( 0 == TasmotaGlobal.tele_period )
        {
            DomoticzFloatSensor( DZ_COUNT, me007_data_s.distance_f32 );   // Send distance as Domoticz Counter value
        }
#endif   // USE_DOMOTICZ
#ifdef USE_WEBSERVER
    }
    else
    {
        WSContentSend_PD( HTTP_SNS_F_DISTANCE_CM,
                          "ME007",
                          &me007_data_s.distance_f32 );

        WSContentSend_PD( HTTP_SNS_F_TEMP,
                          "ME007", Settings->flag2.temperature_resolution,
                          &me007_data_s.temperature_f32,
                          D_UNIT_CELSIUS );
#endif   // USE_WEBSERVER
    }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns23( uint32_t function )
{
    bool result_b = false;

#ifdef ME007_DEBUG
    AddLog( LOG_LEVEL_DEBUG_MORE, PSTR("ME007:Xsns23:CallbackId: %i"), function );
#endif

    switch ( function )
    {
    case FUNC_EVERY_SECOND:
        // Sr04TReading();
        result_b = true;
        break;
    case FUNC_JSON_APPEND:
        me007Show( 1U );
        break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
        me007Show( 0U );
        break;
#endif   // USE_WEBSERVER
    }
    return result_b;
}

#endif   // USE_ME007
