#include <string.h>
#include "boards.h"
#include "ble_bcn.h"
#include "ble_srv_common.h"
#include "nrf_gpio.h"
#include "sdk_common.h"

#define BLE_BCN_MAX_UUID_CHAR_LEN             16       /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_BCN_MAX_RSSI_CHAR_LEN             1        /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_BCN_MAX_MAJOR_CHAR_LEN            2        /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_BCN_MAX_MINOR_CHAR_LEN            2        /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_BCN_MAX_ADDRESS_CHAR_LEN          6        /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_BCN_MAX_AD_INTERVAL_CHAR_LEN      4
#define BLE_BCN_MAX_LORA_INTERVAL_CHAR_LEN    2




/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_bcn       Beacon Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_bcn_t * p_bcn, ble_evt_t const * p_ble_evt)
{
    p_bcn->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
  
    ble_bcn_evt_t   evt;
    evt.evt_type    = BLE_BCN_EVT_CONNECT;
  
    p_bcn->evt_handler(p_bcn, &evt);
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_bcn_t * p_bcn, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_bcn->conn_handle = BLE_CONN_HANDLE_INVALID;
  
    ble_bcn_evt_t   evt;
    evt.evt_type    = BLE_BCN_EVT_DISCONNECTED;
  
    p_bcn->evt_handler(p_bcn, &evt);
}

static void on_write(ble_bcn_t * p_bcn, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    ble_bcn_evt_t   evt;
    evt.p_bcn  = p_bcn;
  
    if(p_evt_write->handle == p_bcn->uuid_value_handles.value_handle)
    {
        nrf_gpio_pin_toggle(LED_4);
        evt.params.uuid_data.p_data     = p_evt_write->data;
        evt.params.uuid_data.length     = p_evt_write->len;
        evt.evt_type                    = BLE_BCN_EVT_UUID_DATA;
        p_bcn->evt_handler(p_bcn, &evt);
      
    }
    
    if(p_evt_write->handle == p_bcn->rssi_value_handles.value_handle)
    {
        nrf_gpio_pin_toggle(LED_3);
        evt.params.rssi_data.p_data     = p_evt_write->data;
        evt.params.rssi_data.length     = p_evt_write->len;
        evt.evt_type                    = BLE_BCN_EVT_RSSI_DATA;
        p_bcn->evt_handler(p_bcn, &evt);
      
    }
    
    if(p_evt_write->handle == p_bcn->major_value_handles.value_handle)
    {
        evt.params.major_data.p_data     = p_evt_write->data;
        evt.params.major_data.length     = p_evt_write->len;
        evt.evt_type                     = BLE_BCN_EVT_MAJOR_DATA;
        p_bcn->evt_handler(p_bcn, &evt);
      
    }
    
    if(p_evt_write->handle == p_bcn->minor_value_handles.value_handle)
    {
        evt.params.minor_data.p_data     = p_evt_write->data;
        evt.params.minor_data.length     = p_evt_write->len;
        evt.evt_type                     = BLE_BCN_EVT_MINOR_DATA;
        p_bcn->evt_handler(p_bcn, &evt);
      
    }
    
    
    if(p_evt_write->handle == p_bcn->txpower_value_handles.value_handle)
    {
        evt.params.minor_data.p_data     = p_evt_write->data;
        evt.params.minor_data.length     = p_evt_write->len;
        evt.evt_type                     = BLE_BCN_EVT_TXPOWER_DATA;
        p_bcn->evt_handler(p_bcn, &evt);
      
    }
    
    if(p_evt_write->handle == p_bcn->address_value_handles.value_handle)
    {
        evt.params.minor_data.p_data     = p_evt_write->data;
        evt.params.minor_data.length     = p_evt_write->len;
        evt.evt_type                     = BLE_BCN_EVT_ADDRESS_DATA;
        p_bcn->evt_handler(p_bcn, &evt);
      
    }
    
    if(p_evt_write->handle == p_bcn->ad_interval_value_handles.value_handle)
    {
        evt.params.minor_data.p_data     = p_evt_write->data;
        evt.params.minor_data.length     = p_evt_write->len;
        evt.evt_type                     = BLE_BCN_EVT_AD_INTERVAL_DATA;
        p_bcn->evt_handler(p_bcn, &evt);
      
    }
    
    if(p_evt_write->handle == p_bcn->lora_interval_value_handles.value_handle)
    {
        evt.params.minor_data.p_data     = p_evt_write->data;
        evt.params.minor_data.length     = p_evt_write->len;
        evt.evt_type                     = BLE_BCN_EVT_LORA_INTERVAL_DATA;
        p_bcn->evt_handler(p_bcn, &evt);
      
    }
}

void ble_bcn_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
  ble_bcn_t * p_bcn = (ble_bcn_t *)p_context;
  
  if(p_bcn == NULL || p_ble_evt == NULL)
  {
      return;
  }
  
  switch(p_ble_evt->header.evt_id)
  {
    case BLE_GAP_EVT_CONNECTED:
      on_connect(p_bcn, p_ble_evt);
      break;
    
    case BLE_GAP_EVT_DISCONNECTED:
      on_disconnect(p_bcn, p_ble_evt);
      break;
    
    case BLE_GATTS_EVT_WRITE:
      on_write(p_bcn, p_ble_evt);
      break;
    
    default:
      break;
  }
}

uint32_t ble_bcn_init(ble_bcn_t* p_bcn, const ble_bcn_init_t* p_bcn_init)
{
	if(p_bcn == NULL || p_bcn_init == NULL)
	{
		return NRF_ERROR_NULL;
	}

	uint32_t	  err_code;
	ble_uuid_t	ble_uuid;
  

	//Init service structure
  p_bcn->evt_handler      = p_bcn_init->evt_handler;
	p_bcn->conn_handle			= BLE_CONN_HANDLE_INVALID;

	ble_uuid128_t		base_uuid = {BCN_SERVICE_UUID_BASE};
	err_code	= sd_ble_uuid_vs_add(&base_uuid, &p_bcn->uuid_type);
	VERIFY_SUCCESS(err_code);

	ble_uuid.type		= p_bcn->uuid_type;
	ble_uuid.uuid 	= BCN_SERVICE_UUID;

	err_code	= sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_bcn->service_handle);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	err_code = uuid_change_value_char_add(p_bcn, p_bcn_init);
  if(err_code != NRF_SUCCESS)
  {
      return err_code;
  }
  
  err_code = rssi_change_value_char_add(p_bcn, p_bcn_init);
  if(err_code != NRF_SUCCESS)
  {
      return err_code;
  }
  
  err_code = major_change_value_char_add(p_bcn, p_bcn_init);
  if(err_code != NRF_SUCCESS)
  {
      return err_code;
  }
  
  err_code = minor_change_value_char_add(p_bcn, p_bcn_init);
  if(err_code != NRF_SUCCESS)
  {
      return err_code;
  }
  
  err_code = txpower_change_value_char_add(p_bcn, p_bcn_init);
  if(err_code != NRF_SUCCESS)
  {
      return err_code;
  }
  
  err_code = address_change_value_char_add(p_bcn, p_bcn_init);
  if(err_code != NRF_SUCCESS)
  {
      return err_code;
  }
  
  err_code = ad_interval_change_value_char_add(p_bcn, p_bcn_init);
  if(err_code != NRF_SUCCESS)
  {
      return err_code;
  }
  
  err_code = lora_interval_change_value_char_add(p_bcn, p_bcn_init);
  if(err_code != NRF_SUCCESS)
  {
      return err_code;
  }
 
  return NRF_SUCCESS;
}


/**@brief Function for adding the UUID Value characteristic.
 *
 * @param[in]   p_bcn        Beacon Change Service structure.
 * @param[in]   p_bcn_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t uuid_change_value_char_add(ble_bcn_t * p_bcn, const ble_bcn_init_t * p_bcn_init)
{
  ble_gatts_char_md_t       char_md;
  ble_gatts_attr_md_t       cccd_md;
  ble_gatts_attr_t          attr_char_value;
  ble_uuid_t                ble_uuid;
  ble_gatts_attr_md_t       attr_md;
  
  memset(&char_md, 0, sizeof(char_md));
  
  char_md.char_props.read   = 1;
  char_md.char_props.write  = 1;
  char_md.char_props.notify = 0;
  char_md.p_char_user_desc  = NULL;
  char_md.p_char_pf         = NULL;
  char_md.p_user_desc_md    = NULL;
  char_md.p_cccd_md         = NULL;
  char_md.p_sccd_md         = NULL;
  
  
  memset(&attr_md, 0, sizeof(attr_md));
  
  attr_md.read_perm         = p_bcn_init->bcn_value_char_attr_md.read_perm;
  attr_md.write_perm        = p_bcn_init->bcn_value_char_attr_md.write_perm;
  attr_md.vloc              = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth           = 0;
  attr_md.wr_auth           = 0;
  attr_md.vlen              = 0;
  
  ble_uuid.type             = p_bcn->uuid_type;
  ble_uuid.uuid             = UUID_VALUE_CHAR_UUID;
  
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  
  attr_char_value.p_uuid    = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = sizeof(uint8_t);
  attr_char_value.init_offs = 0;
  attr_char_value.max_len   = BLE_BCN_MAX_UUID_CHAR_LEN;
  
  return sd_ble_gatts_characteristic_add(p_bcn->service_handle,
                                         &char_md,
                                         &attr_char_value,
                                         &p_bcn->uuid_value_handles);  
}


/**@brief Function for adding the RSSI Value characteristic.
 *
 * @param[in]   p_bcn        Beacon Change Service structure.
 * @param[in]   p_bcn_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t rssi_change_value_char_add(ble_bcn_t * p_bcn, const ble_bcn_init_t * p_bcn_init)
{
  ble_gatts_char_md_t       char_md;
  ble_gatts_attr_md_t       cccd_md;
  ble_gatts_attr_t          attr_char_value;
  ble_uuid_t                ble_uuid;
  ble_gatts_attr_md_t       attr_md;
  
  memset(&char_md, 0, sizeof(char_md));
  
  char_md.char_props.read   = 1;
  char_md.char_props.write  = 1;
  char_md.char_props.notify = 0;
  char_md.p_char_user_desc  = NULL;
  char_md.p_char_pf         = NULL;
  char_md.p_user_desc_md    = NULL;
  char_md.p_cccd_md         = NULL;
  char_md.p_sccd_md         = NULL;
  
  
  memset(&attr_md, 0, sizeof(attr_md));
  
  attr_md.read_perm         = p_bcn_init->bcn_value_char_attr_md.read_perm;
  attr_md.write_perm        = p_bcn_init->bcn_value_char_attr_md.write_perm;
  attr_md.vloc              = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth           = 0;
  attr_md.wr_auth           = 0;
  attr_md.vlen              = 0;
  
  ble_uuid.type             = p_bcn->uuid_type;
  ble_uuid.uuid             = RSSI_VALUE_CHAR_UUID;
  
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  
  attr_char_value.p_uuid    = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = sizeof(uint8_t);
  attr_char_value.init_offs = 0;
  attr_char_value.max_len   = BLE_BCN_MAX_RSSI_CHAR_LEN;
  
  return sd_ble_gatts_characteristic_add(p_bcn->service_handle,
                                         &char_md,
                                         &attr_char_value,
                                         &p_bcn->rssi_value_handles);  
}

/**@brief Function for adding the MAJOR Value characteristic.
 *
 * @param[in]   p_bcn        Beacon Change Service structure.
 * @param[in]   p_bcn_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t major_change_value_char_add(ble_bcn_t * p_bcn, const ble_bcn_init_t * p_bcn_init)
{
  ble_gatts_char_md_t       char_md;
  ble_gatts_attr_md_t       cccd_md;
  ble_gatts_attr_t          attr_char_value;
  ble_uuid_t                ble_uuid;
  ble_gatts_attr_md_t       attr_md;
  
  memset(&char_md, 0, sizeof(char_md));
  
  char_md.char_props.read   = 1;
  char_md.char_props.write  = 1;
  char_md.char_props.notify = 0;
  char_md.p_char_user_desc  = NULL;
  char_md.p_char_pf         = NULL;
  char_md.p_user_desc_md    = NULL;
  char_md.p_cccd_md         = NULL;
  char_md.p_sccd_md         = NULL;
  
  
  memset(&attr_md, 0, sizeof(attr_md));
  
  attr_md.read_perm         = p_bcn_init->bcn_value_char_attr_md.read_perm;
  attr_md.write_perm        = p_bcn_init->bcn_value_char_attr_md.write_perm;
  attr_md.vloc              = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth           = 0;
  attr_md.wr_auth           = 0;
  attr_md.vlen              = 0;
  
  ble_uuid.type             = p_bcn->uuid_type;
  ble_uuid.uuid             = MAJOR_VALUE_CHAR_UUID;
  
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  
  attr_char_value.p_uuid    = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = sizeof(uint8_t);
  attr_char_value.init_offs = 0;
  attr_char_value.max_len   = BLE_BCN_MAX_MAJOR_CHAR_LEN;
  
  return sd_ble_gatts_characteristic_add(p_bcn->service_handle,
                                         &char_md,
                                         &attr_char_value,
                                         &p_bcn->major_value_handles);  
}

/**@brief Function for adding the MINOR Value characteristic.
 *
 * @param[in]   p_bcn        Beacon Change Service structure.
 * @param[in]   p_bcn_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t minor_change_value_char_add(ble_bcn_t * p_bcn, const ble_bcn_init_t * p_bcn_init)
{
  ble_gatts_char_md_t       char_md;
  ble_gatts_attr_md_t       cccd_md;
  ble_gatts_attr_t          attr_char_value;
  ble_uuid_t                ble_uuid;
  ble_gatts_attr_md_t       attr_md;
  
  memset(&char_md, 0, sizeof(char_md));
  
  char_md.char_props.read   = 1;
  char_md.char_props.write  = 0;
  char_md.char_props.notify = 0;
  char_md.p_char_user_desc  = NULL;
  char_md.p_char_pf         = NULL;
  char_md.p_user_desc_md    = NULL;
  char_md.p_cccd_md         = NULL;
  char_md.p_sccd_md         = NULL;
  
  
  memset(&attr_md, 0, sizeof(attr_md));
  
  attr_md.read_perm         = p_bcn_init->bcn_value_char_attr_md.read_perm;
  //attr_md.write_perm        = p_bcn_init->bcn_value_char_attr_md.write_perm;
  attr_md.vloc              = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth           = 0;
  attr_md.wr_auth           = 0;
  attr_md.vlen              = 0;
  
  ble_uuid.type             = p_bcn->uuid_type;
  ble_uuid.uuid             = MINOR_VALUE_CHAR_UUID;
  
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  
  attr_char_value.p_uuid    = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = sizeof(uint8_t);
  attr_char_value.init_offs = 0;
  attr_char_value.max_len   = BLE_BCN_MAX_MINOR_CHAR_LEN;
  
  return sd_ble_gatts_characteristic_add(p_bcn->service_handle,
                                         &char_md,
                                         &attr_char_value,
                                         &p_bcn->minor_value_handles);  
}


/**@brief Function for adding the MINOR Value characteristic.
 *
 * @param[in]   p_bcn        Beacon Change Service structure.
 * @param[in]   p_bcn_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t txpower_change_value_char_add(ble_bcn_t * p_bcn, const ble_bcn_init_t * p_bcn_init)
{
  ble_gatts_char_md_t       char_md;
  ble_gatts_attr_md_t       cccd_md;
  ble_gatts_attr_t          attr_char_value;
  ble_uuid_t                ble_uuid;
  ble_gatts_attr_md_t       attr_md;
  
  memset(&char_md, 0, sizeof(char_md));
  
  char_md.char_props.read   = 1;
  char_md.char_props.write  = 1;
  char_md.char_props.notify = 0;
  char_md.p_char_user_desc  = NULL;
  char_md.p_char_pf         = NULL;
  char_md.p_user_desc_md    = NULL;
  char_md.p_cccd_md         = NULL;
  char_md.p_sccd_md         = NULL;
  
  
  memset(&attr_md, 0, sizeof(attr_md));
  
  attr_md.read_perm         = p_bcn_init->bcn_value_char_attr_md.read_perm;
  attr_md.write_perm        = p_bcn_init->bcn_value_char_attr_md.write_perm;
  attr_md.vloc              = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth           = 0;
  attr_md.wr_auth           = 0;
  attr_md.vlen              = 0;
  
  ble_uuid.type             = p_bcn->uuid_type;
  ble_uuid.uuid             = TXPOWER_VALUE_CHAR_UUID;
  
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  
  attr_char_value.p_uuid    = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = sizeof(uint8_t);
  attr_char_value.init_offs = 0;
  attr_char_value.max_len   = BLE_BCN_MAX_MINOR_CHAR_LEN;
  
  return sd_ble_gatts_characteristic_add(p_bcn->service_handle,
                                         &char_md,
                                         &attr_char_value,
                                         &p_bcn->txpower_value_handles);  
}


/**@brief Function for adding the ADDRESS Value characteristic.
 *
 * @param[in]   p_bcn        Beacon Change Service structure.
 * @param[in]   p_bcn_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t address_change_value_char_add(ble_bcn_t * p_bcn, const ble_bcn_init_t * p_bcn_init)
{
  ble_gatts_char_md_t       char_md;
  ble_gatts_attr_md_t       cccd_md;
  ble_gatts_attr_t          attr_char_value;
  ble_uuid_t                ble_uuid;
  ble_gatts_attr_md_t       attr_md;
  
  memset(&char_md, 0, sizeof(char_md));
  
  char_md.char_props.read   = 1;
  char_md.char_props.write  = 1;
  char_md.char_props.notify = 0;
  char_md.p_char_user_desc  = NULL;
  char_md.p_char_pf         = NULL;
  char_md.p_user_desc_md    = NULL;
  char_md.p_cccd_md         = NULL;
  char_md.p_sccd_md         = NULL;
  
  
  memset(&attr_md, 0, sizeof(attr_md));
  
  attr_md.read_perm         = p_bcn_init->bcn_value_char_attr_md.read_perm;
  attr_md.write_perm        = p_bcn_init->bcn_value_char_attr_md.write_perm;
  attr_md.vloc              = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth           = 0;
  attr_md.wr_auth           = 0;
  attr_md.vlen              = 0;
  
  ble_uuid.type             = p_bcn->uuid_type;
  ble_uuid.uuid             = ADDRESS_VALUE_CHAR_UUID;
  
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  
  attr_char_value.p_uuid    = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = sizeof(uint8_t);
  attr_char_value.init_offs = 0;
  attr_char_value.max_len   = BLE_BCN_MAX_ADDRESS_CHAR_LEN;
  
  return sd_ble_gatts_characteristic_add(p_bcn->service_handle,
                                         &char_md,
                                         &attr_char_value,
                                         &p_bcn->address_value_handles);  
}

/**@brief Function for adding the AD Interval Value characteristic.
 *
 * @param[in]   p_bcn        Beacon Change Service structure.
 * @param[in]   p_bcn_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t ad_interval_change_value_char_add(ble_bcn_t * p_bcn, const ble_bcn_init_t * p_bcn_init)
{
  ble_gatts_char_md_t       char_md;
  ble_gatts_attr_md_t       cccd_md;
  ble_gatts_attr_t          attr_char_value;
  ble_uuid_t                ble_uuid;
  ble_gatts_attr_md_t       attr_md;
  
  memset(&char_md, 0, sizeof(char_md));
  
  char_md.char_props.read   = 1;
  char_md.char_props.write  = 1;
  char_md.char_props.notify = 0;
  char_md.p_char_user_desc  = NULL;
  char_md.p_char_pf         = NULL;
  char_md.p_user_desc_md    = NULL;
  char_md.p_cccd_md         = NULL;
  char_md.p_sccd_md         = NULL;
  
  
  memset(&attr_md, 0, sizeof(attr_md));
  
  attr_md.read_perm         = p_bcn_init->bcn_value_char_attr_md.read_perm;
  attr_md.write_perm        = p_bcn_init->bcn_value_char_attr_md.write_perm;
  attr_md.vloc              = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth           = 0;
  attr_md.wr_auth           = 0;
  attr_md.vlen              = 0;
  
  ble_uuid.type             = p_bcn->uuid_type;
  ble_uuid.uuid             = AD_INTERVAL_VALUE_CHAR_UUID;
  
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  
  attr_char_value.p_uuid    = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = sizeof(uint8_t);
  attr_char_value.init_offs = 0;
  attr_char_value.max_len   = BLE_BCN_MAX_AD_INTERVAL_CHAR_LEN;
  
  return sd_ble_gatts_characteristic_add(p_bcn->service_handle,
                                         &char_md,
                                         &attr_char_value,
                                         &p_bcn->ad_interval_value_handles);  
}


/**@brief Function for adding the LORA Interval Value characteristic.
 *
 * @param[in]   p_bcn        Beacon Change Service structure.
 * @param[in]   p_bcn_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t lora_interval_change_value_char_add(ble_bcn_t * p_bcn, const ble_bcn_init_t * p_bcn_init)
{
  ble_gatts_char_md_t       char_md;
  ble_gatts_attr_md_t       cccd_md;
  ble_gatts_attr_t          attr_char_value;
  ble_uuid_t                ble_uuid;
  ble_gatts_attr_md_t       attr_md;
  
  memset(&char_md, 0, sizeof(char_md));
  
  char_md.char_props.read   = 1;
  char_md.char_props.write  = 1;
  char_md.char_props.notify = 0;
  char_md.p_char_user_desc  = NULL;
  char_md.p_char_pf         = NULL;
  char_md.p_user_desc_md    = NULL;
  char_md.p_cccd_md         = NULL;
  char_md.p_sccd_md         = NULL;
  
  
  memset(&attr_md, 0, sizeof(attr_md));
  
  attr_md.read_perm         = p_bcn_init->bcn_value_char_attr_md.read_perm;
  attr_md.write_perm        = p_bcn_init->bcn_value_char_attr_md.write_perm;
  attr_md.vloc              = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth           = 0;
  attr_md.wr_auth           = 0;
  attr_md.vlen              = 0;
  
  ble_uuid.type             = p_bcn->uuid_type;
  ble_uuid.uuid             = LORA_INTERVAL_VALUE_CHAR_UUID;
  
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  
  attr_char_value.p_uuid    = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = sizeof(uint8_t);
  attr_char_value.init_offs = 0;
  attr_char_value.max_len   = BLE_BCN_MAX_LORA_INTERVAL_CHAR_LEN;
  
  return sd_ble_gatts_characteristic_add(p_bcn->service_handle,
                                         &char_md,
                                         &attr_char_value,
                                         &p_bcn->lora_interval_value_handles);  
}



