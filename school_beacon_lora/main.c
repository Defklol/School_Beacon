/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @example examples/ble_peripheral/ble_app_buttonless_dfu
 *
 * @brief Secure DFU Buttonless Service Application main file.
 *
 * This file contains the source code for a sample application using the proprietary
 * Secure DFU Buttonless Service. This is a template application that can be modified
 * to your needs. To extend the functionality of this application, please find
 * locations where the comment "// YOUR_JOB:" is present and read the comments.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nrf_dfu_svci.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"

#include "ble_tps.h"

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "bsp_btn_ble.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_state.h"
#include "ble_dfu.h"
#include "nrf_ble_gatt.h"
#include "fds.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_timer.h"
#include "app_uart.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_bcn.h"
#include "nrf_delay.h"
#include "nrf_drv_uart.h"



#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define DEVICE_NAME                     "NKFUST_LBEACON"                             /**< Name of device. Will be included in the advertising data. */
//#define APP_ADV_INTERVAL                500                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                         /**< The advertising timeout in units of seconds. */

#define APP_BLE_OBSERVER_PRIO           1                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define POWER_LORA_INTERVAL             APP_TIMER_TICKS(2000)
//-------------------------------------------------Beacon----------------------------------------------------------/

#define APP_BEACON_INFO_LENGTH          0x17                              /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                              /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                              /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                              /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x004C                            /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0x01, 0x01                        /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x01, 0x01                        /**< Minor value used to identify Beacons. */
#define APP_BEACON_UUID                 0x01, 0x12, 0x23, 0x34, \
                                        0x45, 0x56, 0x67, 0x78, \
                                        0x89, 0x9a, 0xab, 0xbc, \
                                        0xcd, 0xde, 0xef, 0xf0            /**< Proprietary UUID for Beacon. */

//-----------------------------------------------------------------------------------------------------------------/


#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define LORA_SLEEP_INT                  26                                           /**< LORA Sleep INT Pin. */

#define SAMPLES_IN_BUFFER 1

#define fstorage_use

static char lora_data[14];
static uint32_t fs_array[11] = {1};
static uint8_t                n_address[6]={1,1,1,1,1,1};
static uint8_t                n_uuid[16], n_major[2], n_txpower, n_adv_interval=1;

static void uart_init(void);
static void advertising_init(void);
static void application_timers_start(void);

#ifdef  fstorage_use
static uint32_t fstorage_init(void);
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt); 
#endif


NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */
BLE_TPS_DEF(m_tps);                                                                 /**< Txpower module instance. */
BLE_BCN_DEF(m_bcn);                                                                 /**< Customer Beacon module instance. */
APP_TIMER_DEF(m_power_lora_id);                                                     /**< Timer module instance. */
#ifdef  fstorage_use
NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =                                         /**< Flash Storage module instance. */
{
    .evt_handler  = fstorage_evt_handler,
    .start_addr   = 0x63000,
    .end_addr     = 0x73000,
};
#endif

static ble_gap_addr_t          m_ble_addr;                                          /**< Change MAC address. */
static nrf_saadc_value_t       value;
static const nrf_drv_timer_t   m_timer = NRF_DRV_TIMER_INSTANCE(3);
static nrf_ppi_channel_t       m_ppi_channel;
static uint32_t                m_adc_evt_counter;

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                            /**< Handle of the current connection. */
static void advertising_start(bool erase_bonds);                                    /**< Forward declaration of advertising start function */

static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                         // implementation.
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                         // manufacturer specific data in this implementation.
    APP_BEACON_UUID,     // 128 bit UUID value.
    APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
    APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
    APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in
                         // this implementation.
};

/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            // YOUR_JOB: Get ready to reset into DFU mode
            //
            // If you aren't finished with any ongoing tasks, return "false" to
            // signal to the system that reset is impossible at this stage.
            //
            // Here is an example using a variable to delay resetting the device.
            //
            // if (!m_ready_for_reset)
            // {
            //      return false;
            // }
            // else
            //{
            //
            //    // Device ready to enter
            //    uint32_t err_code;
            //    err_code = sd_softdevice_disable();
            //    APP_ERROR_CHECK(err_code);
            //    err_code = app_timer_stop_all();
            //    APP_ERROR_CHECK(err_code);
            //}
            break;

        default:
            // YOUR_JOB: Implement any of the other events available from the power management module:
            //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
            //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
            //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

//lint -esym(528, m_app_shutdown_handler)
/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);


// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}};


// YOUR_JOB: Update this code if you want to do anything given a DFU event (optional).
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");
            // YOUR_JOB: Disconnect all bonded devices that currently are connected.
            //           This is required to receive a service changed indication
            //           on bootup after a successful (or aborted) Device Firmware Update.
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }
}





static void change_address(void)
{
    uint32_t err_code;
    m_ble_addr.addr_type      = BLE_GAP_ADDR_TYPE_PUBLIC;
    
    for(uint8_t i=0; i<6; i++)
    {
        m_ble_addr.addr[i] = n_address[5-i];
    }
    
    err_code = sd_ble_gap_addr_set(&m_ble_addr);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            //sleep_mode_enter();
            break;

        default:
            break;
    }
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start(false);
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}



static void power_lora_timeout_handler(void* p_context)
{
    static uint32_t    power_lora_times = 0;

    uint32_t    err_code;
    power_lora_times = power_lora_times%10800;
    //power_lora_times = power_lora_times%40;
    power_lora_times++;
    NRF_LOG_INFO("power_lora_time : %d\n\n", power_lora_times);
    
    switch(power_lora_times)
    {
      
      //Read SAADC & decide Voltage.
      case 1:
        NRF_LOG_INFO("case 1");
      
        nrf_gpio_pin_clear(LORA_SLEEP_INT);
        nrf_drv_saadc_sample_convert(0, &value);
        snprintf( lora_data, sizeof(lora_data), "AT+DTX=4,%04d", value);
        m_beacon_info[20] = value>>8;
        m_beacon_info[21] = value&0xFF;
        
        if(value < 720)
        {
            nrf_gpio_cfg_output(LED_4);
            nrf_gpio_pin_clear(LED_4);
        }//Detect battery voltage.<2.5V LED4 blink.
        
        //nrf_gpio_cfg_default(TX_PIN_NUMBER);
        //nrf_gpio_cfg_default(RX_PIN_NUMBER);
        break;
      
      //Init Uart & wake up Lora.
      case 2:
        NRF_LOG_INFO("case 2");
      
        nrf_gpio_pin_set(LORA_SLEEP_INT);
        uart_init();
      break;
       
      //Reset AT Command.
      case 3:
        NRF_LOG_INFO("case 3");
        
        //Reset AT Command
        err_code = app_uart_put(0x0D);
        if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
        {
            NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
            APP_ERROR_CHECK(err_code);
        }else
        {
            NRF_LOG_INFO("\n\n");
        }
      break;
            
      //Send ADC data with LoRa.
      case 4:
        NRF_LOG_INFO("case 4");
        
        //Put ADC data to lora
        for (uint32_t i = 0; i < 13; i++)
        {
            do
            {
                err_code = app_uart_put(lora_data[i]);
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }else
                {
                    NRF_LOG_INFO("%c", lora_data[i]);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
        
        err_code = app_uart_put(0x0D);
        if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
        {
            NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
            APP_ERROR_CHECK(err_code);
        }else
        {
            NRF_LOG_INFO("\n\n");
        }
        
      break;
        
      //Close uart & drop down LoRa sleep pin.
      case 5:
        NRF_LOG_INFO("case 5");
        //app_uart_flush();
        nrf_gpio_pin_clear(LORA_SLEEP_INT);
        app_uart_close();
        //app_uart_flush();

        /*
        nrf_gpio_cfg_output(TX_PIN_NUMBER);
        nrf_gpio_pin_clear(TX_PIN_NUMBER);
        nrf_gpio_cfg_output(RX_PIN_NUMBER);
        nrf_gpio_pin_clear(RX_PIN_NUMBER);
        */
       
      break;
      
      //Reset ADV data.
      case 6:
        NRF_LOG_INFO("case 6");
      
        if(m_conn_handle == BLE_CONN_HANDLE_INVALID)
        {
            sd_ble_gap_adv_stop();
            advertising_init();
            err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
            APP_ERROR_CHECK(err_code);
        }else NRF_LOG_INFO("Is Connecting...\n");
        
      break;
          

      case 10:
        NRF_LOG_INFO("case 10");
        //app_uart_flush();
        break;
      default:
        break;
    }
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t    err_code;
    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_power_lora_id,
                                APP_TIMER_MODE_REPEATED,
                                power_lora_timeout_handler);
    APP_ERROR_CHECK(err_code);
  
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static void on_bcn_evt(ble_bcn_t      * p_bcn_service,
                       ble_bcn_evt_t  * p_evt)
{
    uint32_t err_code;
    uint8_t  txpower_fs_status = 0, address_fs_status = 0, adv_interval_fs_status = 0;         /**< 0 isn't flash; 1 is ok. */
    uint8_t  n_rssi = 0;
    #ifdef  fstorage_use
    err_code = fstorage_init();
    APP_ERROR_CHECK(err_code);
    #endif
  
    switch(p_evt->evt_type)
    {
      case BLE_BCN_EVT_CONNECT:
        break;
      
      case BLE_BCN_EVT_DISCONNECTED:
        break;
      
      //Change Beacon UUID Data
      case BLE_BCN_EVT_UUID_DATA:
        if(p_evt->params.uuid_data.length == 16)
        {
            for(uint8_t i=0; i<p_evt->params.uuid_data.length; i++)
            {
                n_uuid[i] = p_evt->params.uuid_data.p_data[i];
                m_beacon_info[i+2] = n_uuid[i];
                NRF_LOG_INFO("%d-%d-%d", m_beacon_info[i+2], n_uuid[i], p_evt->params.uuid_data.p_data[i]);
            }
            if(p_evt->params.uuid_data.p_data[p_evt->params.uuid_data.length-1] == '\r') NRF_LOG_INFO("\n");
            
            #ifdef  fstorage_use
            //Erase flash data.
            err_code = nrf_fstorage_erase(&fstorage, 0x63000, 1, NULL);
            NRF_LOG_INFO("erase err_code is %d", err_code);
            APP_ERROR_CHECK(err_code);
            
            //Store address data to fs_array.
            fs_array[0] = (uint32_t)n_uuid[0] << 24 | (uint32_t)n_uuid[1] << 16 | (uint32_t)n_uuid[2] << 8 | (uint32_t)n_uuid[3];
            fs_array[1] = (uint32_t)n_uuid[4] << 24 | (uint32_t)n_uuid[5] << 16 | (uint32_t)n_uuid[6] << 8 | (uint32_t)n_uuid[7];
            fs_array[2] = (uint32_t)n_uuid[8] << 24 | (uint32_t)n_uuid[9] << 16 | (uint32_t)n_uuid[10] << 8 | (uint32_t)n_uuid[11];
            fs_array[3] = (uint32_t)n_uuid[12] << 24 | (uint32_t)n_uuid[13] << 16 | (uint32_t)n_uuid[14] << 8 | (uint32_t)n_uuid[15];
            
            //Flash write.
            err_code = nrf_fstorage_write(&fstorage, 0x63000, fs_array, 44, NULL);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Write \"0x%x\" & \"0x%x\" & \"0x%x\" & \"0x%x\"to flash", fs_array[0], fs_array[1], fs_array[2], fs_array[3]);
            #endif
        }
        break;
      
      //Change Timer Reset
        /*
      case BLE_BCN_EVT_RSSI_DATA:
        if(p_evt->params.rssi_data.length == 1)
        {
            for(uint8_t i=0; i<p_evt->params.rssi_data.length; i++)
            {
                n_rssi = p_evt->params.rssi_data.p_data[i];
                NRF_LOG_INFO("%d-%d", m_beacon_info[i+22], p_evt->params.rssi_data.p_data[i]);
            }
            if(p_evt->params.rssi_data.p_data[p_evt->params.rssi_data.length-1] == '\r') NRF_LOG_INFO("\n");
            
            if(n_rssi == 0x01)
            {
                app_timer_resume();
                NRF_LOG_INFO("123");
            }
            
        }
        break;
      */
      //Change Beacon MAJOR Data    
      case BLE_BCN_EVT_MAJOR_DATA:
        if(p_evt->params.major_data.length == 2)
        {
            for(uint8_t i=0; i<p_evt->params.major_data.length; i++)
            {
                n_major[i] = p_evt->params.major_data.p_data[i];
                m_beacon_info[i+18] = n_major[i];
                NRF_LOG_INFO("%d-%d", m_beacon_info[i+18], p_evt->params.major_data.p_data[i]);
            }
            if(p_evt->params.major_data.p_data[p_evt->params.major_data.length-1] == '\r') NRF_LOG_INFO("\n");
            
            #ifdef  fstorage_use
            //Erase flash data.
            err_code = nrf_fstorage_erase(&fstorage, 0x63000, 1, NULL);
            NRF_LOG_INFO("erase err_code is %d", err_code);
            APP_ERROR_CHECK(err_code);
            
            //Store address data to fs_array.
            fs_array[5] = (uint32_t)n_major[0] << 8 | (uint32_t)n_major[1];
            
            //Flash write.
            err_code = nrf_fstorage_write(&fstorage, 0x63000, fs_array, 44, NULL);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Write \"0x%x\" to flash", fs_array[5]);
            #endif
        }
        break;
      
      
      //Change TxPower Data.RANGE -40, -20, -16, -12, -8, -4, 0, and 4 dBm
      case BLE_BCN_EVT_TXPOWER_DATA:
        n_txpower = *p_evt->params.txpower_data.p_data;
        NRF_LOG_INFO("%04x-%04x", n_txpower, *p_evt->params.txpower_data.p_data);
        
        switch(n_txpower)
        {
            case 0xD8:
              txpower_fs_status = 1;
            
              err_code = sd_ble_gap_tx_power_set(n_txpower);
              APP_ERROR_CHECK(err_code);
              
              ble_tps_tx_power_level_set(&m_tps, n_txpower);
            break;
            
            case 0xEC:
              txpower_fs_status = 1;
            
              err_code = sd_ble_gap_tx_power_set(n_txpower);
              APP_ERROR_CHECK(err_code);
              
              ble_tps_tx_power_level_set(&m_tps, n_txpower);
            break;
            
            case 0xF0:
              txpower_fs_status = 1;
            
              err_code = sd_ble_gap_tx_power_set(n_txpower);
              APP_ERROR_CHECK(err_code);
              
              ble_tps_tx_power_level_set(&m_tps, n_txpower);
            break;
            
            case 0xF4:
              txpower_fs_status = 1;
            
              err_code = sd_ble_gap_tx_power_set(n_txpower);
              APP_ERROR_CHECK(err_code);
              
              ble_tps_tx_power_level_set(&m_tps, n_txpower);
            break;
            
            case 0xF8:
              txpower_fs_status = 1;
            
              err_code = sd_ble_gap_tx_power_set(n_txpower);
              APP_ERROR_CHECK(err_code);
              
              ble_tps_tx_power_level_set(&m_tps, n_txpower);
            break;
            
            case 0xFC:
              txpower_fs_status = 1;
            
              err_code = sd_ble_gap_tx_power_set(n_txpower);
              APP_ERROR_CHECK(err_code);
              
              ble_tps_tx_power_level_set(&m_tps, n_txpower);
            break;
            
            case 0x00:
              txpower_fs_status = 1;
            
              err_code = sd_ble_gap_tx_power_set(n_txpower);
              APP_ERROR_CHECK(err_code);
              
              ble_tps_tx_power_level_set(&m_tps, n_txpower);
            break;
            
            case 0x04:
              txpower_fs_status = 1;
            
              err_code = sd_ble_gap_tx_power_set(n_txpower);
              APP_ERROR_CHECK(err_code);
              
              ble_tps_tx_power_level_set(&m_tps, n_txpower);
            break;
            
        }
        
        if(txpower_fs_status == 1)
        {
            #ifdef  fstorage_use
            //Erase flash data.
            err_code = nrf_fstorage_erase(&fstorage, 0x63000, 1, NULL);
            NRF_LOG_INFO("erase err_code is %d", err_code);
            APP_ERROR_CHECK(err_code);
            
            //Store address data to fs_array.
            fs_array[6] = (uint32_t)n_txpower;
            
            //Flash write.
            err_code = nrf_fstorage_write(&fstorage, 0x63000, fs_array, 44, NULL);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Write \"0x%x\" to flash", fs_array[6]);
            #endif
        }
        
        break;
        
      //Change MAC Address
      case BLE_BCN_EVT_ADDRESS_DATA:
        if(p_evt->params.address_data.length == 6)
        {
            if(p_evt->params.address_data.p_data[0] || p_evt->params.address_data.p_data[1] || p_evt->params.address_data.p_data[2] ||
              p_evt->params.address_data.p_data[3] || p_evt->params.address_data.p_data[4] || p_evt->params.address_data.p_data[5])
            {
                address_fs_status = 1;
                for(uint8_t i=0; i<p_evt->params.address_data.length; i++)
                {
                    n_address[i] = p_evt->params.address_data.p_data[i];
                    NRF_LOG_INFO("%02x-%02x", n_address[i], p_evt->params.address_data.p_data[i]);
                }
            }else NRF_LOG_INFO("No 0.0.0.0\n");
            
        }
        
        if(address_fs_status == 1)
        {
            #ifdef  fstorage_use
            //Erase flash data.
            err_code = nrf_fstorage_erase(&fstorage, 0x63000, 1, NULL);
            NRF_LOG_INFO("erase err_code is %d", err_code);
            APP_ERROR_CHECK(err_code);
            
            //Store address data to fs_array.
            fs_array[7] = (uint32_t)n_address[0] << 24 | (uint32_t)n_address[1] << 16 | (uint32_t)n_address[2] << 8 | (uint32_t)n_address[3];
            fs_array[8] = (uint32_t)n_address[4] << 8 | (uint32_t)n_address[5];
            
            //Flash write.
            err_code = nrf_fstorage_write(&fstorage, 0x63000, fs_array, 44, NULL);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Write \"0x%x\" & \"0x%x\" to flash", fs_array[7], fs_array[8]);
            #endif
        }          
                
        break;
        
        //Change ADV Interval.
        case BLE_BCN_EVT_AD_INTERVAL_DATA:
         
          n_adv_interval = (p_evt->params.ad_interval_data.p_data[1]*256 + p_evt->params.ad_interval_data.p_data[0])/100;
          NRF_LOG_INFO("%d-%d", n_adv_interval, p_evt->params.ad_interval_data.p_data[1]*256 + p_evt->params.ad_interval_data.p_data[0]);
         
          if(n_adv_interval<=10 && n_adv_interval>=1)
          {
              adv_interval_fs_status = 1;
          }
          
          if(adv_interval_fs_status == 1)
          {
              #ifdef  fstorage_use
              //Erase flash data.
              err_code = nrf_fstorage_erase(&fstorage, 0x63000, 1, NULL);
              NRF_LOG_INFO("erase err_code is %d", err_code);
              APP_ERROR_CHECK(err_code);
              
              //Store address data to fs_array.
              fs_array[9] = (uint32_t)n_adv_interval;
              
              //Flash write.
              err_code = nrf_fstorage_write(&fstorage, 0x63000, fs_array, 44, NULL);
              APP_ERROR_CHECK(err_code);
              NRF_LOG_INFO("Write \"%d\" to flash", fs_array[9]);
              #endif
          }
          
        break;
        
        //Change LORA updata Interval.
          /*
        case BLE_BCN_EVT_LORA_INTERVAL_DATA:
        if(p_evt->params.lora_interval_data.length == 1 || p_evt->params.lora_interval_data.length == 2)
        {
            for(uint8_t i=0; i<p_evt->params.lora_interval_data.length; i++)
            {
                n_lora_interval[i] = p_evt->params.lora_interval_data.p_data[i];
                
                NRF_LOG_INFO("%d-%d", n_lora_interval[i], p_evt->params.lora_interval_data.p_data[i]);
            }
            if(p_evt->params.uuid_data.p_data[p_evt->params.lora_interval_data.length-1] == '\r') NRF_LOG_INFO("\n");
        }
        break;
        */
        
      default:
        break;
    }
    
    #ifdef  fstorage_use
    err_code = nrf_fstorage_uninit(&fstorage, NULL);
    APP_ERROR_CHECK(err_code);
    #endif
}

//-------------------------------------------------Service Init----------------------------------------------------------/
static void tps_init(void)
{
    uint32_t err_code; 
    ble_tps_init_t tps_init;

    memset(&tps_init, 0, sizeof(tps_init));
    
    tps_init.initial_tx_power_level = 0;    
   	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&tps_init.tps_attr_md.read_perm); 

    err_code = ble_tps_init(&m_tps, &tps_init);
    APP_ERROR_CHECK(err_code);
}

static void bcn_init(void)
{
    uint32_t              err_code;
    ble_bcn_init_t        bcn_init;
  
    memset(&bcn_init, 0, sizeof(bcn_init));
  
    bcn_init.evt_handler  = on_bcn_evt;
  
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bcn_init.bcn_value_char_attr_md.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bcn_init.bcn_value_char_attr_md.read_perm);
  
    err_code  = ble_bcn_init(&m_bcn,  &bcn_init);
    APP_ERROR_CHECK(err_code);
}

static void dfu_init(void)
{
    uint32_t err_code;
    ble_dfu_buttonless_init_t dfus_init =
    {
        .evt_handler = ble_dfu_evt_handler
    };
    
    err_code = ble_dfu_buttonless_async_svci_init();
    APP_ERROR_CHECK(err_code);
     
    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    tps_init();
    bcn_init();
    dfu_init();
}

//-----------------------------------------------------------------------------------------------------------------------/


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    uint32_t    err_code;
    
    err_code = app_timer_start(m_power_lora_id,
                               POWER_LORA_INTERVAL,
                               NULL);
    APP_ERROR_CHECK(err_code);
  
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);

    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
        
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
        
            change_address();
        
            //Restart advertising
            advertising_init();
            err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
            APP_ERROR_CHECK(err_code);
        
            break;

        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        
            //Change TPS Service data.
            ble_tps_tx_power_level_set(&m_tps, n_txpower);
            break;

#if defined(S132)
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;
#endif

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init()
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/** @brief Clear bonding information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            //sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }                                                                
            }
            break; // BSP_EVENT_KEY_0

        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
/*
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;
    ble_advdata_manuf_data_t manuf_specific_data;

    //manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data = m_beacon_info;
    manuf_specific_data.data.size   = sizeof(m_beacon_info);
  
    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;
  
    init.srdata.p_manuf_specific_data    = &manuf_specific_data;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}
*/

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    uint32_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for the Power manager.
 */
static void log_init(void)
{
    uint32_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief   Function for initializing the GATT module.
 * @details The GATT module handles ATT_MTU and Data Length update procedures automatically.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t                    err_code;
    ble_advertising_init_t        init;
    ble_advdata_manuf_data_t      manuf_specific_data;

    manuf_specific_data.company_identifier    = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data           = m_beacon_info;
    manuf_specific_data.data.size             = sizeof(m_beacon_info);

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;
  
    init.srdata.p_manuf_specific_data    = &manuf_specific_data;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = n_adv_interval *100;
    init.config.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_DEBUG("advertising is started");
    }
}
//--------------------------------------------------UART----------------------------------------------------------/

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    uint8_t        data[50];
    uint8_t        index = 0;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            index++;
            UNUSED_VARIABLE(app_uart_get(&data[index]));
            NRF_LOG_INFO("%c", data[index]);
            index = 0;
            break;
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}


static void uart_init(void)
{
    uint32_t err_code;
  
    app_uart_comm_params_t const comm_params = 
    {
        .rx_pin_no      =   RX_PIN_NUMBER,
        .tx_pin_no      =   TX_PIN_NUMBER,
        .rts_pin_no     =   NULL,
        .cts_pin_no     =   NULL,
        .flow_control   =   APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity     =   false,
        .baud_rate      =   UART_BAUDRATE_BAUDRATE_Baud9600
    };
    
    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
    
}


//----------------------------------------------------------------------------------------------------------------/

//-------------------------------------------------SAADC----------------------------------------------------------/
void timer_handler(nrf_timer_event_t event_type, void * p_context)
{

}


void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 400ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 400);
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}


void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);

    APP_ERROR_CHECK(err_code);
}


void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
      /*
        int i;
      
        NRF_LOG_INFO("ADC event number: %d", (int)m_adc_evt_counter);

        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            NRF_LOG_INFO("%04d", p_event->data.done.p_buffer[i]);
        }
        
        sprintf( lora_data, "AT+DTX=4,%04d", p_event->data.done.p_buffer[i]);
        */
        m_adc_evt_counter++;
    }
}


void saadc_init(void)
{
    ret_code_t err_code;
  
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

}

//----------------------------------------------------------------------------------------------------------------/


//-------------------------------------------------FSTORAGE----------------------------------------------------------/

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
}

uint32_t fstorage_init(void)
{
    uint32_t err_code;

    #ifdef  fstorage_use  
    err_code = nrf_fstorage_init(&fstorage, &nrf_fstorage_sd, NULL);
    if(err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    #endif
    
    return NRF_SUCCESS;
}

void fstorsge_get(void)
{
    //Get Beacon array
    m_beacon_info[2] = (fs_array[0] >> 24)&0xFF;
    m_beacon_info[3] = (fs_array[0] >> 16)&0xFF;
    m_beacon_info[4] = (fs_array[0] >>  8)&0xFF;
    m_beacon_info[5] =  fs_array[0]&0xFF;
    m_beacon_info[6] = (fs_array[1] >> 24)&0xFF;
    m_beacon_info[7] = (fs_array[1] >> 16)&0xFF;
    m_beacon_info[8] = (fs_array[1] >>  8)&0xFF;
    m_beacon_info[9] =  fs_array[1]&0xFF;
    m_beacon_info[10] = (fs_array[2] >> 24)&0xFF;
    m_beacon_info[11] = (fs_array[2] >> 16)&0xFF;
    m_beacon_info[12] = (fs_array[2] >>  8)&0xFF;
    m_beacon_info[13] =  fs_array[2]&0xFF;
    m_beacon_info[14] = (fs_array[3] >> 24)&0xFF;
    m_beacon_info[15] = (fs_array[3] >> 16)&0xFF;
    m_beacon_info[16] = (fs_array[3] >>  8)&0xFF;
    m_beacon_info[17] =  fs_array[3]&0xFF;
    
    //Get RSSI array
    m_beacon_info[22] = fs_array[4]&0xFF;
    
    //Get Major array
    m_beacon_info[18] = (fs_array[5] >> 8)&0xFF;
    m_beacon_info[19] = fs_array[5]&0xFF;
    
    //Get Tx Power
    n_txpower = fs_array[6]&0xFF;
    NRF_LOG_INFO("0x%x", n_txpower);
    if(n_txpower == 0x00 || n_txpower == 0x04 || n_txpower == 0xD8 || n_txpower == 0xEC || \
       n_txpower == 0xF0 || n_txpower == 0xF4 || n_txpower == 0xF8 || n_txpower == 0xFC )
    {
        n_txpower = n_txpower;
    }else
    {
      n_txpower = 0x00;
      m_beacon_info[22] = 0xBB;
    }
    NRF_LOG_INFO("0x%x", n_txpower);
    
    
    //Get address array
    n_address[0] = (fs_array[7] >> 24)&0xFF;
    n_address[1] = (fs_array[7] >> 16)&0xFF;
    n_address[2] = (fs_array[7] >> 8)&0xFF;
    n_address[3] = fs_array[7]&0xFF;
    n_address[4] = (fs_array[8] >> 8)&0xFF;
    n_address[5] = fs_array[8]&0xFF;
    
    if(n_address[0] || n_address[1] || n_address[2] ||
       n_address[3] || n_address[4] || n_address[5])
    {
        NRF_LOG_INFO("Address is %02x-%02x-%02x-%02x-%02x-%02x\n", n_address[0], n_address[1], n_address[2], n_address[3], n_address[4], n_address[5]);
    }else
    {
      NRF_LOG_INFO("%s\n", "No 0.0.0.0");
      n_address[5] = 0xFF;
    }
    
    //Get ADV interval
    n_adv_interval = fs_array[9]&0xFF;
    if(n_adv_interval == 0xFF) n_adv_interval = 5;      //adv_interval need 100ms-1000ms.
    
    //Get Lora interval
}

static void power_management_init(void)
{
    uint32_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;
    uint32_t err_code;
    
    //lora sleep mode interrupt pin
    nrf_gpio_cfg_output(LORA_SLEEP_INT);
    nrf_gpio_pin_set(LORA_SLEEP_INT);
    //nrf_gpio_pin_clear(LORA_SLEEP_INT);
  
  
    // Initialize.
    log_init();
    uart_init();
    app_uart_close();
    //uart_uinit();
    saadc_init();
    
    


    timers_init();
    power_management_init();
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    peer_manager_init();
    gap_params_init();
    gatt_init();
  
    #ifdef  fstorage_use
    err_code = fstorage_init();
    APP_ERROR_CHECK(err_code);
        
    err_code = nrf_fstorage_read(&fstorage, 0x63000, fs_array, 44);
    APP_ERROR_CHECK(err_code);
    
    fstorsge_get();
    err_code = 	nrf_fstorage_uninit( &fstorage, NULL);
    APP_ERROR_CHECK(err_code);
    #endif
    
    change_address();
    
    advertising_init();
    services_init();
    conn_params_init();

    NRF_LOG_INFO("Application started\n");

    // Start execution.
    application_timers_start();
    advertising_start(erase_bonds);
    
    err_code = sd_ble_gap_tx_power_set(n_txpower);
    APP_ERROR_CHECK(err_code);

    // Enter main loop.
    for (;;)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            nrf_pwr_mgmt_run();
        }
    }
}

/**
 * @}
 */
