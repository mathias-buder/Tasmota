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

#define ME007_DEBUG_MSG_TAG                "ME007: "
#define ME007_WS_MQTT_MSG_TAG              "ME007"
#define ME007_SENSOR_ERROR_CNT_CURRENT_TAG "ErrorCounterCurrent"
#define ME007_SENSOR_ERROR_CNT_TOTAL_TAG   "ErrorCounterTotal"


#define ME007_MIN_SENSOR_DISTANCE          30U  /* @unit cm */
#ifndef ME007_MAX_SENSOR_DISTANCE
#define ME007_MAX_SENSOR_DISTANCE          800U /**< Maximum measurement distance: 8 m */
#endif
#define ME007_WS_SCALE_SWITCH_THRESH       100U /* @unit cm */
#define ME007_SERIAL_IF_BAUD_RATE          9600U
#define ME007_SERIAL_SOF                   0xFF /**< Start of frame indicator (header) */
#define ME007_SERIAL_FRAME_SIZE            6U   /**< Total frame size: 6 byte  */
#define ME007_SERIAL_DATA_SIZE             5U   /**< Header (1Byte) + distance (2 byte) + temperature (2 byte): 5 byte */
#define ME007_SERIAL_MAX_WAIT_TIME         61U  /**< Max. wait time for data after trigger signal @unit ms */
#define ME007_SERIAL_MAX_DATA_RECEIVE_TIME 50U  /**< Max. time to receive entire data frame @unit ms */
#define ME007_TRIG_SIG_DURATION            4U   /**< Time duration of trigger low-pulse @unit ms */
#define ME007_MEDIAN_FILTER_SIZE           5U   /**< Median filter samples @unit sample */
#define ME007_SENSOR_NUM_ERROR             10U  /**< Number of tries to detect sensor */



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
    float            distance_cm_f32;
    float            temperature_deg_f32;
    float            buffer_distance_cm_vf32[ME007_MEDIAN_FILTER_SIZE];
    uint8_t          error_cnt_current_u8;
    uint16_t         error_cnt_total_u16;
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
void me007_init( void );

/**
 * @details This function performs a read/write test on the specified I2C device to make sure the
 * low-level interface is working correctly.
 * @param[in] uint8_t I2C error code
 */
bool me007_measure( float* p_distance_cm_f32, float* p_temperature_f32 );

/**
 * @details This function performs a read/write test on the specified I2C device to make sure the
 * low-level interface is working correctly.
 * @param[in] uint8_t I2C error code
 */
void me007_show( uint8_t type_u8 );

/*********************************************************************************************/
/* Function Definitions */
/*********************************************************************************************/
void me007_init( void )
{
    AddLog( LOG_LEVEL_INFO, PSTR( ME007_DEBUG_MSG_TAG "Initializing ..." ) );

    /* Check if sensor pins are selected/used in web-interface */
    if (    ( false == PinUsed( GPIO_ME007_TRIG ) )
         || ( false == PinUsed( GPIO_ME007_RX ) ) )
    {
        AddLog( LOG_LEVEL_ERROR, PSTR( ME007_DEBUG_MSG_TAG "Serial/Trigger interface not configured" ) );
        return;
    }

    /* Init global sensor state to ME007_STATE_NOT_DETECTED */
    me007_data_s.state_e            = ME007_STATE_NOT_DETECTED;
    me007_data_s.error_cnt_current_u8       = 0U;
    me007_data_s.error_cnt_total_u16 = 0U;

    /* Store real pin number */
    me007_data_s.pin_rx_u8   = Pin( GPIO_ME007_RX );
    me007_data_s.pin_trig_u8 = Pin( GPIO_ME007_TRIG );

    DEBUG_SENSOR_LOG( PSTR( ME007_DEBUG_MSG_TAG "Using GPIOs: Trigger: %i / Rx: %i)" ), me007_data_s.pin_trig_u8, me007_data_s.pin_rx_u8 );

    /* Configure serial interface  */
    /* Only Rx pin is required as ME007 is controlled using its trigger pin. Therefore, passing value "-1" as transmit_pin argument */
    p_SerialIf = new TasmotaSerial( me007_data_s.pin_rx_u8, -1, 1U );

    if (    ( nullptr != p_SerialIf )
         && ( true    == p_SerialIf->begin( ME007_SERIAL_IF_BAUD_RATE ) ) )
    {
        if ( true == p_SerialIf->hardwareSerial() )
        {
            ClaimSerial();
        }

        pinMode( me007_data_s.pin_trig_u8, OUTPUT );    /**< @details Configure trigger pin as output */
        digitalWrite( me007_data_s.pin_trig_u8, HIGH ); /**< @details Set trigger pin to high-level as it ME007 requires a falling edge to initiate measurement */

        /* Detect sensor */
        AddLog( LOG_LEVEL_INFO, PSTR( ME007_DEBUG_MSG_TAG "Detecting ..." ) );
        for( uint8_t idx_u8 = 0U; idx_u8 < ME007_SENSOR_NUM_ERROR; ++idx_u8 )
        {
            bool detected_b = me007_measure( &me007_data_s.distance_cm_f32,
                                             &me007_data_s.temperature_deg_f32 );
            if( true == detected_b )
            {
                me007_data_s.state_e = ME007_STATE_DETECTED;
                break;
            }
            else
            {
                DEBUG_SENSOR_LOG( PSTR( ME007_DEBUG_MSG_TAG "Measurement error: %i" ), idx_u8 );
            }
        }
        AddLog( LOG_LEVEL_INFO, PSTR( ME007_DEBUG_MSG_TAG "%s" ),me007_data_s.state_e == ME007_STATE_DETECTED ? "Detected" : "Not detected, Sensor deactivated" );
    }
    else
    {
        AddLog( LOG_LEVEL_ERROR, PSTR( ME007_DEBUG_MSG_TAG "Serial interface unavailable" ) );
    }
}

bool me007_measure( float* p_distance_cm_f32, float* p_temperature_f32 )
{
    bool                      status_b                           = false; /**< @details Status io sensor reading, false: faulty reading, true: valid reading */
    ME007_SERIAL_RECEIVE_TYPE state_e                            = ME007_SERIAL_RECEIVE_TYPE_SOF;
    uint8_t                   buffer_vu8[ME007_SERIAL_DATA_SIZE] = {0U};
    uint8_t                   buffer_idx_u8                      = 0U;
    uint8_t                   data_byte_u8                       = 0U;
    uint32_t                  timestamp_ms_u32                   = 0U;
    uint8_t                   data_receive_time_ms_u8            = 0U;

    if (    ( nullptr != p_distance_cm_f32 )
         && ( nullptr != p_temperature_f32 )
         && ( nullptr != p_SerialIf ) )
    {
        /* Trigger new sensor measurement */
        digitalWrite( me007_data_s.pin_trig_u8, LOW );
        delay( ME007_TRIG_SIG_DURATION );               /**< @details Wait 1ms to give the oin time to go low */
        digitalWrite( me007_data_s.pin_trig_u8, HIGH );

        /* Store trigger time */
        timestamp_ms_u32 = millis();

        DEBUG_SENSOR_LOG( PSTR( ME007_DEBUG_MSG_TAG "Sensor reading triggered" ) );

        /* Give sensor some time to take a measurement and send the result */
        /* Max. wait time is T2max + T3max = 61 ms (see https://wiki.dfrobot.com/Water-proof%20Ultrasonic%20Sensor%20(ULS)%20%20SKU:%20SEN0300, section "Serial Output" for details) */
        while ( ( timestamp_ms_u32 + ME007_SERIAL_MAX_WAIT_TIME + data_receive_time_ms_u8 ) >= millis() )
        {
            if ( 0U < p_SerialIf->available() )
            {
                /* Extend time to receive entire data frame */
                data_receive_time_ms_u8 = ME007_SERIAL_MAX_DATA_RECEIVE_TIME;

                /* Read 1 byte of data */
                data_byte_u8 = p_SerialIf->read();

                DEBUG_SENSOR_LOG( PSTR( ME007_DEBUG_MSG_TAG "Serial: Byte received: 0x%x" ), data_byte_u8 );

                switch ( state_e )
                {
                case ME007_SERIAL_RECEIVE_TYPE_SOF:
                    if ( ME007_SERIAL_SOF == data_byte_u8 )
                    {
                        buffer_vu8[buffer_idx_u8++] = data_byte_u8;
                        state_e = ME007_SERIAL_RECEIVE_TYPE_DATA;
                        DEBUG_SENSOR_LOG( PSTR( ME007_DEBUG_MSG_TAG "(Idx: %i) Serial: SOF detected" ), buffer_idx_u8 );
                    }
                    break;

                case ME007_SERIAL_RECEIVE_TYPE_DATA:
                    buffer_vu8[buffer_idx_u8++] = data_byte_u8; 
                    DEBUG_SENSOR_LOG( PSTR( ME007_DEBUG_MSG_TAG "(Idx: %i) Receiving distance/temperature data" ), buffer_idx_u8 );

                    /* Move to next state if 4 data bytes have been received */
                    if( ME007_SERIAL_DATA_SIZE <= buffer_idx_u8 )
                    {
                        state_e = ME007_SERIAL_RECEIVE_TYPE_CHECKSUM;
                    }
                    break;

                case ME007_SERIAL_RECEIVE_TYPE_CHECKSUM:
                    /* Calculate expected checksum */
                    uint16_t sum_u16 = 0U;
                    for( uint8_t idx_u8 = 0U; idx_u8 < ME007_SERIAL_DATA_SIZE; ++idx_u8 )
                    {
                        sum_u16 += buffer_vu8[idx_u8];
                        DEBUG_SENSOR_LOG( PSTR( ME007_DEBUG_MSG_TAG "Sum[%i]: %i (value: %i)"), idx_u8, sum_u16, buffer_vu8[idx_u8] );
                    }

                    /* Only the the lower 8 bit shall be used */
                    sum_u16 &= 0x00FF;

                    DEBUG_SENSOR_LOG( PSTR( ME007_DEBUG_MSG_TAG "Comparing expected sum %i to checksum %i" ), sum_u16, data_byte_u8 );

                    /* Compare expected and receive checksum */
                    if( sum_u16 == data_byte_u8 )
                    {
                        /* Assemble and scale distance to cm */
                        *p_distance_cm_f32 = ( ( buffer_vu8[1U] << 8U ) + buffer_vu8[2U] ) / 10.0F;

                        /* Apply physical sensor measurement limits */
                        if( ME007_MIN_SENSOR_DISTANCE > *p_distance_cm_f32 )
                        {
                            *p_distance_cm_f32 = 0.0F;
                        }
                        else if( ME007_MAX_SENSOR_DISTANCE < *p_distance_cm_f32 )
                        {
                            *p_distance_cm_f32 = ME007_MAX_SENSOR_DISTANCE;
                        }
                        else
                        {
                            /* Ok: Measurement is within the physical sensor measurement limits */
                        }
                        
                        /* Assemble and scale temperature °C */
                        /* Check sign-bit (MSB) */
                        if( 0x80 == ( buffer_vu8[2U] & 0x80 ) )
                        {
                            buffer_vu8[2U] ^= 0x80;
                        }

                        *p_temperature_f32 = ( ( buffer_vu8[3U] << 8U ) + buffer_vu8[4U] ) / 10.0F;

                        DEBUG_SENSOR_LOG( PSTR( ME007_DEBUG_MSG_TAG "Distance [cm]: %s, Temperature [deg]: %s"),
                                          String( *p_distance_cm_f32, 1U ).c_str(),
                                          String( *p_temperature_f32, 1U ).c_str() );

                        /* Early-return in case measurement is valid */
                        return true;
                    }
                    else
                    {
                        /* status_b is set to false already */
                    }
                    break;
                }
            }
        }
    }
    return status_b;
}

void me007_read_value( void )
{
    /* Record some sensor measurements */
    for ( uint8_t idx_u8 = 0U; idx_u8 < ME007_MEDIAN_FILTER_SIZE; ++idx_u8 )
    {
        bool status_b = me007_measure( &me007_data_s.buffer_distance_cm_vf32[idx_u8],
                                       &me007_data_s.temperature_deg_f32 );

        if ( true == status_b )
        {
            /* Apply histogram filter */
            me007_data_s.distance_cm_f32 = me007_data_s.buffer_distance_cm_vf32[0];
            
            if ( 0U < me007_data_s.error_cnt_current_u8 )
            {
                me007_data_s.error_cnt_current_u8--;
            }
        }
        else
        {
            me007_data_s.error_cnt_current_u8++;

            /* Store total number of measurement errors */
            me007_data_s.error_cnt_total_u16++;
        }

        /* Check error counter and deactivate sensor in case error is present for ME007_SENSOR_NUM_ERROR cycles */
        if( ME007_SENSOR_NUM_ERROR <= me007_data_s.error_cnt_current_u8 )
        {
            AddLog( LOG_LEVEL_ERROR, PSTR( ME007_DEBUG_MSG_TAG "Error @ counter: %i, Sensor deactivated" ), me007_data_s.error_cnt_current_u8 );
            me007_data_s.state_e = ME007_STATE_NOT_DETECTED;
            return;
        }
    }
}


void me007_show( uint8_t type_u8 )
{
    switch ( type_u8 )
    {
    case ME007_SHOW_TYPE_JS:
        ResponseAppend_P( PSTR( ",\"" ME007_WS_MQTT_MSG_TAG "\":{\"" D_JSON_DISTANCE "\":%1_f,\"" D_JSON_TEMPERATURE "\":%1_f,\"" ME007_SENSOR_ERROR_CNT_CURRENT_TAG "\":%i,\"" ME007_SENSOR_ERROR_CNT_TOTAL_TAG "\":%i}" ),
                               &me007_data_s.distance_cm_f32,
                               &me007_data_s.temperature_deg_f32,
                               me007_data_s.error_cnt_current_u8,
                               me007_data_s.error_cnt_total_u16 );
#ifdef USE_DOMOTICZ
        if ( 0U == TasmotaGlobal.tele_period )
        {
            DomoticzFloatSensor( DZ_COUNT, me007_data_s.distance_cm_f32 );    /**< @details Send distance as Domoticz counter value */
            DomoticzFloatSensor( DZ_TEMP, me007_data_s.temperature_deg_f32 ); /**< @details Send distance as Domoticz temperature value */
        }
#endif /* USE_DOMOTICZ */
        break;
#ifdef USE_WEBSERVER
    case ME007_SHOW_TYPE_WS:
        if( ME007_WS_SCALE_SWITCH_THRESH > me007_data_s.distance_cm_f32 )
        {
            WSContentSend_PD( HTTP_SNS_F_DISTANCE_CM,
                              ME007_WS_MQTT_MSG_TAG,
                              &me007_data_s.distance_cm_f32 );
        }
        else
        {
            /* Convert distance to meter */
            float ws_distance_f32 = me007_data_s.distance_cm_f32 / 100.0F;
            WSContentSend_PD( HTTP_SNS_F_DISTANCE_M,
                              ME007_WS_MQTT_MSG_TAG,
                              &ws_distance_f32 );
        }

        WSContentSend_PD( HTTP_SNS_F_TEMP,
                          ME007_WS_MQTT_MSG_TAG,
                          Settings->flag2.temperature_resolution,
                          &me007_data_s.temperature_deg_f32 );
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
        me007_init();
        break;

    case FUNC_EVERY_SECOND:
        if( ME007_STATE_DETECTED == me007_data_s.state_e )
        {
            me007_read_value();
            result_b = true;
        }
        break;

    case FUNC_JSON_APPEND:
        if (ME007_STATE_DETECTED == me007_data_s.state_e )
        {
            me007_show( ME007_SHOW_TYPE_JS );
        }
        break;

#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
        if (ME007_STATE_DETECTED == me007_data_s.state_e )
        {
            me007_show( ME007_SHOW_TYPE_WS );
        }
        break;
#endif   // USE_WEBSERVER
    }
    return result_b;
}

#endif   // USE_ME007
