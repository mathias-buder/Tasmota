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

/*********************************************************************************************/
/* Defines */
/*********************************************************************************************/
#define XSNS_23 23

#define ME007_DEBUG_MSG_TAG "ME007: "
#define ME007_MQTT_MSG_TAG  "ME007"
#define ME007_LOG_DATA_SIZE 180U /**< Byte */
#ifndef ME007_MAX_SENSOR_DISTANCE
#define ME007_MAX_SENSOR_DISTANCE 800U /**< Maximum measurement distance: 8 m */
#endif
#define ME007_SERIAL_IF_BAUD_RATE 9600U
// #define ME007_SERIAL_IF_BUFFER_SIZE 32U  /**< Frame buffer: 32 byte */
#define ME007_SERIAL_SOF                   0xFF /**< Start of frame indicator (header) */
#define ME007_SERIAL_FRAME_SIZE            6U   /**< Total frame size: 6 byte  */
#define ME007_SERIAL_DATA_SIZE             4U   /**< Distance (2 byte) + temperature (2 byte) data: 4 byte */
#define ME007_SERIAL_MAX_WAIT_TIME         61U  /**< Max. wait time for data after trigger signal @unit ms */
#define ME007_SERIAL_MAX_DATA_RECEIVE_TIME 50U  /**< Max. time to receive entire data frame @unit ms */

/*********************************************************************************************/
/* Enums */
/*********************************************************************************************/
/**
 * @details I2C error types
 */
enum ME007_SHOW_TYPE
{
    ME007_SHOW_TYPE_JS = 0U, /**< @details Domain log message tag string */
    ME007_SHOW_TYPE_WS
};

typedef enum ME007_SERIAL_RECEIVE_TYPE_TAG
{
    ME007_SERIAL_RECEIVE_TYPE_SOF = 0U,
    ME007_SERIAL_RECEIVE_TYPE_DATA,
    ME007_SERIAL_RECEIVE_TYPE_CHECKSUM
} ME007_SERIAL_RECEIVE_TYPE;

/**
 * @details ME007 
 */
typedef enum ME007_STATE_TYPE_TAG
{
    ME007_STATE_NOT_DETECTED = 0U,
    ME007_STATE_DETECTED
} ME007_STATE_TYPE;

/*********************************************************************************************/
/* Structures */
/*********************************************************************************************/

/**
 * @details I2C error types
 */
struct
{
    uint8_t          pin_rx_u8;
    uint8_t          pin_trig_u8;
    ME007_STATE_TYPE state_e;
    bool             sensorDetected_b;
    float            distance_f32;
    float            temperature_f32;
} me007_data_s;

/*********************************************************************************************/
/* Global Variables */
/*********************************************************************************************/
TasmotaSerial* p_SerialIf = nullptr; /**< @details Pointer to serial interface object */

/*********************************************************************************************/
/* Function Prototypes */
/*********************************************************************************************/

/**
 * @details This function performs a read/write test on the specified I2C device to make sure the
 * low-level interface is working correctly.
 * @param[in] uint8_t I2C error code
 */
void Me007Init( void );

/**
 * @details This function performs a read/write test on the specified I2C device to make sure the
 * low-level interface is working correctly.
 * @param[in] uint8_t I2C error code
 */
void Me007ReadValue( void );

/**
 * @details This function performs a read/write test on the specified I2C device to make sure the
 * low-level interface is working correctly.
 * @param[in] uint8_t I2C error code
 */
void Me007Show( uint8_t type_u8 );

/*********************************************************************************************/
/* Function Definitions */
/*********************************************************************************************/
void Me007Init( void )
{
    DEBUG_SENSOR_LOG( PSTR( ME007_DEBUG_MSG_TAG "Initializing ..." ) );

    /* Check if sensor pins are selected/used in web-interface */
    if ( ( false == PinUsed( GPIO_ME007_TRIG ) )
         || ( false == PinUsed( GPIO_ME007_RX ) ) )
    {
        DEBUG_SENSOR_LOG( PSTR( ME007_DEBUG_MSG_TAG "Serial/Trigger interface not configured" ) );
        return;
    }

    /* Init global sensor state to ME007_STATE_NOT_DETECTED */
    me007_data_s.state_e = ME007_STATE_NOT_DETECTED;

    /* Store real pin number */
    me007_data_s.pin_rx_u8   = Pin( GPIO_ME007_RX );
    me007_data_s.pin_trig_u8 = Pin( GPIO_ME007_TRIG );

    DEBUG_SENSOR_LOG( PSTR( ME007_DEBUG_MSG_TAG "Using GPIOs: Trigger: %i / Rx: %i)" ), me007_data_s.pin_trig_u8, me007_data_s.pin_rx_u8 );

    /* Configure serial interface  */
    /* Only Rx pin is required as ME007 is controlled using its trigger pin. Therefore, passing value "-1" as transmit_pin */
    p_SerialIf = new TasmotaSerial( me007_data_s.pin_rx_u8, -1, 1U );

    if ( ( nullptr != p_SerialIf )
         && ( true == p_SerialIf->begin( ME007_SERIAL_IF_BAUD_RATE ) ) )
    {
        /* Configure trigger pin as output */
        pinMode( me007_data_s.pin_trig_u8, OUTPUT );
        digitalWrite( me007_data_s.pin_trig_u8, HIGH ); /**< @details Set trigger pin to high-level as it ME007 requires a falling edge to initiate measurement */
    }
    else
    {
        DEBUG_SENSOR_LOG( PSTR( ME007_DEBUG_MSG_TAG "Serial interface unavailable" ) );
    }
}

bool Me007ReadValue( float* p_distance_f32, float* p_temperature_f32 )
{
    uint8_t                   byte_cnt_u8             = 0U;
    ME007_SERIAL_RECEIVE_TYPE state_e                 = ME007_SERIAL_RECEIVE_TYPE_SOF;
    uint32_t                  timestamp_ms_u32        = 0U;
    uint8_t                   data_receive_time_ms_u8 = 0U;
    bool                      status_b                = false; /**< @details Status io sensor reading, false: faulty reading, true: valid reading */
    uint16_t                  distance_data_u16       = 0U;
    uint16_t                  temperature_data_u16    = 0U;

    if ( ( nullptr != p_distance_f32 )
         && ( nullptr != p_temperature_f32 )
         && ( nullptr != p_SerialIf ) )
    {
        /* Trigger sensor reading */
        digitalWrite( me007_data_s.pin_trig_u8, LOW );
        delayMicroseconds( 400 );
        digitalWrite( me007_data_s.pin_trig_u8, HIGH );

        /* Store trigger time */
        timestamp_ms_u32 = millis();

        DEBUG_SENSOR_LOG( PSTR( ME007_DEBUG_MSG_TAG "Sensor reading triggered" ) );

        /* Give sensor some time to take a measurement and send the result */
        /* Max. wait time is T2max + T3max = 61 ms (see https://wiki.dfrobot.com/Water-proof%20Ultrasonic%20Sensor%20(ULS)%20%20SKU:%20SEN0300, section "Serial Output" for details) */
        while ( ( timestamp_ms_u32 + ME007_SERIAL_MAX_WAIT_TIME + data_receive_time_ms_u8 ) >= millis() )
        {
            if ( p_SerialIf->available() )
            {
                DEBUG_SENSOR_LOG( PSTR( ME007_DEBUG_MSG_TAG "Serial: Byte available" ) );
                /* Extend time to receive entire data frame */
                data_receive_time_ms_u8 = ME007_SERIAL_MAX_DATA_RECEIVE_TIME;

                /* Read 1 byte of data */
                uint8_t data_u8 = p_SerialIf->read();

                DEBUG_SENSOR_LOG( PSTR( ME007_DEBUG_MSG_TAG "Serial: Byte received: 0x%x" ), data_u8 );

                switch ( state_e )
                {
                case ME007_SERIAL_RECEIVE_TYPE_SOF:
                    if ( ME007_SERIAL_SOF == data_u8 )
                    {
                        byte_cnt_u8++;
                        state_e = ME007_SERIAL_RECEIVE_TYPE_DATA;
                        DEBUG_SENSOR_LOG( PSTR( ME007_DEBUG_MSG_TAG "Serial: SOF detected" ) );
                    }
                    break;

                case ME007_SERIAL_RECEIVE_TYPE_DATA:
                    if ( ( ME007_SERIAL_DATA_SIZE + 1U ) >= byte_cnt_u8 )
                    {

                        byte_cnt_u8++;
                    }
                    else
                    {
                        state_e = ME007_SERIAL_RECEIVE_TYPE_CHECKSUM;
                    }
                    break;

                case ME007_SERIAL_RECEIVE_TYPE_CHECKSUM:

                    break;
                }
            }
        }
    }

    return status_b;
}

void Me007Show( uint8_t type_u8 )
{
    switch ( type_u8 )
    {
    case ME007_SHOW_TYPE_JS:
        ResponseAppend_P( PSTR( ",\"ME007_MQTT_MSG_TAG\":{\"" D_JSON_DISTANCE "\":%1_f}" ), &me007_data_s.distance_f32 );
        ResponseAppend_P( PSTR( ",\"ME007_MQTT_MSG_TAG\":{\"" D_JSON_TEMPERATURE "\":%1_f}" ), &me007_data_s.temperature_f32 );
#ifdef USE_DOMOTICZ
        if ( 0U == TasmotaGlobal.tele_period )
        {
            DomoticzFloatSensor( DZ_COUNT, me007_data_s.distance_f32 );   /**< @details Send distance as Domoticz counter value */
            DomoticzFloatSensor( DZ_TEMP, me007_data_s.temperature_f32 ); /**< @details Send distance as Domoticz temperature value */
        }
#endif /* USE_DOMOTICZ */
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
#endif /* USE_WEBSERVER */

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
        Me007ReadValue( &me007_data_s.distance_f32, &me007_data_s.temperature_f32 );
        result_b = true;
        break;
    case FUNC_JSON_APPEND:
        Me007Show( ME007_SHOW_TYPE_JS );
        break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
        Me007Show( ME007_SHOW_TYPE_WS );
        break;
#endif   // USE_WEBSERVER
    }
    return result_b;
}

#endif   // USE_ME007
