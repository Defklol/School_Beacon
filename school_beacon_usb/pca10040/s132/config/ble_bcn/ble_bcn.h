#ifndef BLE_BCN_H__
#define BLE_BCN_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"


/**@brief   Macro for defining a ble_bcn instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define		BLE_BCN_DEF(_name)                      \
static ble_bcn_t _name;                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,               \
                     BLE_HRS_BLE_OBSERVER_PRIO,   \
                     ble_bcn_on_ble_evt,          \
                     &_name)


//38f7125a-ab92-40c9-994e-5294fd99177d
#define		BCN_SERVICE_UUID_BASE		{0x38, 0xf7, 0x12, 0x5a,    \
                                   0xab, 0x92, 0x40, 0xc9,	\
                                   0x99, 0x4e, 0x52, 0x94,	\
                                   0xfd, 0x99, 0x17, 0x7d}

#define		BCN_SERVICE_UUID 		     	    0x1400
#define		UUID_VALUE_CHAR_UUID      	  0x1401
#define   RSSI_VALUE_CHAR_UUID          0x1402
#define   MAJOR_VALUE_CHAR_UUID         0x1403
#define   MINOR_VALUE_CHAR_UUID         0x1404  
#define   TXPOWER_VALUE_CHAR_UUID       0x1405
#define   ADDRESS_VALUE_CHAR_UUID       0x1406
#define   AD_INTERVAL_VALUE_CHAR_UUID   0x1407
#define   LORA_INTERVAL_VALUE_CHAR_UUID 0x1408                                   

/**@brief   Beacon Service event types. */
typedef enum
{
    BLE_BCN_EVT_UUID_DATA,
    BLE_BCN_EVT_RSSI_DATA,
    BLE_BCN_EVT_MAJOR_DATA,
    BLE_BCN_EVT_MINOR_DATA,
    BLE_BCN_EVT_TXPOWER_DATA,
    BLE_BCN_EVT_ADDRESS_DATA,
    BLE_BCN_EVT_AD_INTERVAL_DATA,
    BLE_BCN_EVT_LORA_INTERVAL_DATA,
    BLE_BCN_EVT_DISCONNECTED,
    BLE_BCN_EVT_CONNECT
}ble_bcn_evt_type_t;

/* Forward declaration of the ble_nus_t type. */
typedef struct ble_bcn_s	ble_bcn_t;


/**@brief   Beacon Service @ref BLE_BCN_EVT_UUID_DATA event data.
 *
 * @details This structure is passed to an event when @ref BLE_BCN_EVT_UUID_DATA occurs.
 */
typedef struct
{
    uint8_t const * p_data;
    uint16_t        length;
}ble_bcn_evt_uuid_data_t;

/**@brief   Beacon Service @ref BLE_BCN_EVT_RSSI_DATA event data.
 *
 * @details This structure is passed to an event when @ref BLE_BCN_EVT_RSSI_DATA occurs.
 */
typedef struct
{
    uint8_t const * p_data;
    uint16_t        length;
}ble_bcn_evt_rssi_data_t;

/**@brief   Beacon Service @ref BLE_BCN_EVT_MAJOR_DATA event data.
 *
 * @details This structure is passed to an event when @ref BLE_BCN_EVT_MAJOR_DATA occurs.
 */
typedef struct
{
    uint8_t const * p_data;
    uint16_t        length;
}ble_bcn_evt_major_data_t;

/**@brief   Beacon Service @ref BLE_BCN_EVT_MINOR_DATA event data.
 *
 * @details This structure is passed to an event when @ref BLE_BCN_EVT_MINOR_DATA occurs.
 */
typedef struct
{
    uint8_t const * p_data;
    uint16_t        length;
}ble_bcn_evt_minor_data_t;


/**@brief   Beacon Service @ref BLE_BCN_EVT_TXPower_DATA event data.
 *
 * @details This structure is passed to an event when @ref BLE_BCN_EVT_TXPower_DATA occurs.
 */
typedef struct
{
    int8_t const * p_data;
    uint16_t        length;
}ble_bcn_evt_txpower_data_t;

/**@brief   Beacon Service @ref BLE_BCN_EVT_ADDRESS_DATA event data.
 *
 * @details This structure is passed to an event when @ref BLE_BCN_EVT_ADDRESS_DATA occurs.
 */
typedef struct
{
    uint8_t const * p_data;
    uint16_t        length;
}ble_bcn_evt_address_data_t;

/**@brief   Beacon Service @ref BLE_BCN_EVT_ADDRESS_DATA event data.
 *
 * @details This structure is passed to an event when @ref BLE_BCN_EVT_ADDRESS_DATA occurs.
 */
typedef struct
{
    uint16_t const * p_data;
    uint16_t        length;
}ble_bcn_evt_ad_interval_data_t;

/**@brief   Beacon Service @ref BLE_BCN_EVT_ADDRESS_DATA event data.
 *
 * @details This structure is passed to an event when @ref BLE_BCN_EVT_ADDRESS_DATA occurs.
 */
typedef struct
{
    uint8_t const * p_data;
    uint16_t        length;
}ble_bcn_evt_lora_interval_data_t;

/**@brief   Beacon Service event structure.
 *
 * @details This structure is passed to an event coming from service.
 */
typedef struct
{
    ble_bcn_evt_type_t evt_type;
    ble_bcn_t * p_bcn;
    union
    {
        ble_bcn_evt_uuid_data_t             uuid_data;
        ble_bcn_evt_rssi_data_t             rssi_data;
        ble_bcn_evt_major_data_t            major_data;
        ble_bcn_evt_minor_data_t            minor_data;
        ble_bcn_evt_txpower_data_t          txpower_data;
        ble_bcn_evt_address_data_t          address_data;
        ble_bcn_evt_ad_interval_data_t      ad_interval_data;
        ble_bcn_evt_lora_interval_data_t    lora_interval_data;
        
    }params;
}ble_bcn_evt_t;

/**@brief   Beacon Service event handler type. */
typedef void (*ble_bcn_evt_handler_t) (ble_bcn_t * p_bcn, ble_bcn_evt_t * p_evt);

/**@brief Battery Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct 
{
  ble_bcn_evt_handler_t             evt_handler;
	uint8_t									          initial_bcn_value;
	ble_srv_cccd_security_mode_t			bcn_value_char_attr_md;
}ble_bcn_init_t;

/**@brief Custom Service structure. This contains various status information for the service. */
struct ble_bcn_s
{
    ble_bcn_evt_handler_t         evt_handler;
    uint16_t                      service_handle;                 /**< Handle of Custom Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      uuid_value_handles;           	/**< Handles related to the UUID Value characteristic. */
    ble_gatts_char_handles_t      rssi_value_handles;           	/**< Handles related to the RSSI Value characteristic. */
    ble_gatts_char_handles_t      major_value_handles;           	/**< Handles related to the MINOR Value characteristic. */
    ble_gatts_char_handles_t      minor_value_handles;           	/**< Handles related to the MAJOR Value characteristic. */
    ble_gatts_char_handles_t      txpower_value_handles;          /**< Handles related to the TX Power Value characteristic. */
    ble_gatts_char_handles_t      address_value_handles;          /**< Handles related to the MAC Address Value characteristic. */
    ble_gatts_char_handles_t      ad_interval_value_handles;      /**< Handles related to the AD Interval Value characteristic. */
    ble_gatts_char_handles_t      lora_interval_value_handles;    /**< Handles related to the LORA Interval Value characteristic. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                       uuid_type; 
};

/**@brief Function for initializing the Custom Service.
 *
 * @param[out]  p_bcn       Custom Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_bcn_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_bcn_init(ble_bcn_t * p_bcn, const ble_bcn_init_t * p_bcn_init);

/**@brief Function for adding the Beacon Value characteristic.
 *
 * @param[in]   p_bcn        Beacon Change Service structure.
 * @param[in]   p_bcn_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t uuid_change_value_char_add(ble_bcn_t * p_cus, const ble_bcn_init_t * p_cus_init);

static uint32_t rssi_change_value_char_add(ble_bcn_t * p_bcn, const ble_bcn_init_t * p_bcn_init);

static uint32_t major_change_value_char_add(ble_bcn_t * p_cus, const ble_bcn_init_t * p_cus_init);

static uint32_t minor_change_value_char_add(ble_bcn_t * p_cus, const ble_bcn_init_t * p_cus_init);

static uint32_t txpower_change_value_char_add(ble_bcn_t * p_cus, const ble_bcn_init_t * p_cus_init);

static uint32_t address_change_value_char_add(ble_bcn_t * p_cus, const ble_bcn_init_t * p_cus_init);

static uint32_t ad_interval_change_value_char_add(ble_bcn_t * p_cus, const ble_bcn_init_t * p_cus_init);

static uint32_t lora_interval_change_value_char_add(ble_bcn_t * p_cus, const ble_bcn_init_t * p_cus_init);



/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Beacon Service.
 *
 * @note 
 *
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 * @param[in]   p_context  Custom Service structure.
 */
void ble_bcn_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context);

#endif
