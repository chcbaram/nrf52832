/*
 * bleuart.c
 *
 *  Created on: 2021. 5. 13.
 *      Author: baram
 */


#include "bleuart.h"
#include "led.h"
#include "qbuffer.h"
#include "cli.h"
#include "swtimer.h"
#include "fs.h"

#include "app_timer.h"

#include "nrf_sdh.h"

#include "ble_nus.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_dfu.h"
#include "ble_dis.h"

#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"



#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "Nordic_UART"                               /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                       /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUM                       "nRF52832 DMTECH"                           /**< Model Number string. Will be passed to Device Information Service. */
#define VERSION_NUM                     "V210529R1"
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(15,  UNIT_1_25_MS)            /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */



BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

static bool is_init = false;
static bool is_connect = false;

static ret_code_t err_code_init = NRF_SUCCESS;

static qbuffer_t q_rx;
static qbuffer_t q_tx;

static uint8_t q_buf_rx[BLEUART_MAX_BUF_LEN];
static uint8_t q_buf_tx[BLEUART_MAX_BUF_LEN];

static bool is_q_rx_over = false;
static volatile bool is_ready_to_send = false;


#define BLEUART_TX_POWER_INIT           8   // 4dB
#define BLEUART_TX_POWER_LEVEL_MAX      9
                                                            //    0    1    2    3   4   5  6  7  8
static const int8_t tx_power_tbl[BLEUART_TX_POWER_LEVEL_MAX] = {-40, -20, -16, -12, -8, -4, 0, 3, 4, };
static volatile int8_t ble_conn_tx_power_level = BLEUART_TX_POWER_INIT;

static volatile uint32_t app_timer_count = 0;


static bool ble_stack_init(void);
static bool gap_params_init(void);
static bool gatt_init(void);
static bool services_init(void);
static bool advertising_init(void);
static bool conn_params_init(void);
static bool advertising_start(void);

static void handler_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);
static void handler_gatt_evt(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt);
static void handler_nrf_qwr_error(uint32_t nrf_error);
static void handler_nus_data(ble_nus_evt_t * p_evt);
static void handler_on_adv_evt(ble_adv_evt_t ble_adv_evt);
static void handler_on_conn_params_evt(ble_conn_params_evt_t * p_evt);
static void handler_conn_params_error(uint32_t nrf_error);
static void handler_ble_dfu_evt(ble_dfu_buttonless_evt_type_t event);


static void appTimerISR(void *args);


#ifdef _USE_HW_CLI
static void cliCmd(cli_args_t *args);
#endif


bool bleUartInit(void)
{
  bool ret = true;
  swtimer_handle_t h_timer;

  qbufferCreate(&q_rx, q_buf_rx, BLEUART_MAX_BUF_LEN);
  qbufferCreate(&q_tx, q_buf_tx, BLEUART_MAX_BUF_LEN);

  ret &= ble_stack_init();
  ret &= gap_params_init();
  ret &= gatt_init();
  ret &= services_init();
  ret &= advertising_init();
  ret &= conn_params_init();
  ret &= advertising_start();

  is_init = ret;


  cliOpen(_DEF_UART2, 57600);

  h_timer = swtimerGetHandle();
  swtimerSet(h_timer, 1, LOOP_TIME, appTimerISR, NULL);
  swtimerStart(h_timer);


#ifdef _USE_HW_CLI
  cliAdd("ble", cliCmd);
#endif
  return ret;
}

bool bleUartIsInit(void)
{
  return is_init;
}

bool bleUartIsConnect(void)
{
  return is_connect;
}

bool bleUartGetRssi(ble_uart_rssi_t *p_rssi)
{
  bool ret = false;
  uint32_t err_code;

  if (is_connect == true && m_conn_handle != BLE_CONN_HANDLE_INVALID)
  {
    err_code = sd_ble_gap_rssi_get(m_conn_handle, &p_rssi->rssi, &p_rssi->ch_index);
    if (err_code == NRF_SUCCESS)
    {
      ret = true;
    }
  }

  if (ret != true)
  {
    p_rssi->rssi = 0;
    p_rssi->ch_index = 0;
  }

  return ret;
}

uint32_t bleUartAvailable(void)
{
  return qbufferAvailable(&q_rx);
}

bool bleUartFlush(void)
{
  qbufferFlush(&q_rx);
  qbufferFlush(&q_tx);

  return true;
}

uint8_t  bleUartRead(void)
{
  uint8_t ret = 0;

  qbufferRead(&q_rx, &ret, 1);

  return ret;
}

uint32_t bleUartWrite(uint8_t *p_data, uint32_t length)
{
  uint32_t sent_len;
  uint16_t tx_len;
  uint32_t err_code;
  uint8_t *p_tx_buf;
  uint32_t pre_time;


  if (is_connect != true) return 0;


  sent_len = 0;

  while(sent_len < length)
  {
    p_tx_buf = &p_data[sent_len];
    tx_len = length;
    if (tx_len > m_ble_nus_max_data_len)
    {
      tx_len = m_ble_nus_max_data_len;
    }

    is_ready_to_send = false;
    err_code = ble_nus_data_send(&m_nus, p_tx_buf, &tx_len, m_conn_handle);
    if ((err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_NOT_FOUND))
    {
      break;
    }

    pre_time = millis();
    while(millis()-pre_time < 100)
    {
      if (is_ready_to_send == true)
      {
        break;
      }
      if (is_connect != true)
      {
        break;
      }
    }

    if (is_ready_to_send != true)
    {
      break;
    }
  }

  return sent_len;
}

bool ble_stack_init(void)
{
  err_code_init = nrf_sdh_enable_request();
  if (err_code_init != NRF_SUCCESS) return false;

  // Configure the BLE stack using the default settings.
  // Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  err_code_init = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  if (err_code_init != NRF_SUCCESS) return false;

  // Enable BLE stack.
  err_code_init = nrf_sdh_ble_enable(&ram_start);
  if (err_code_init != NRF_SUCCESS) return false;

  // Register a handler for BLE events.
  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, handler_ble_evt, NULL);

  return true;
}

bool gap_params_init(void)
{
  ble_gap_conn_params_t   gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

  char *bd_name_str = (char *)DEVICE_NAME;
  char bd_name[128];
  int32_t bd_name_size = 0;


  if (err_code_init != NRF_SUCCESS) return false;


  if (fsIsExist("bd_name") == true)
  {
    fs_t fs;

    if (fsFileOpen(&fs, "bd_name") == true)
    {
      bd_name_size = fsFileSize(&fs);

      if (bd_name_size > 0)
      {
        fsFileRead(&fs, (uint8_t *)bd_name, bd_name_size);
        bd_name[bd_name_size] = 0;

        bd_name_str = bd_name;
      }

      fsFileClose(&fs);
    }
  }


  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code_init = sd_ble_gap_device_name_set(&sec_mode,
                                        (const uint8_t *) bd_name_str,
                                        strlen(bd_name_str));
  if (err_code_init != NRF_SUCCESS) return false;

  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency     = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

  err_code_init = sd_ble_gap_ppcp_set(&gap_conn_params);
  if (err_code_init != NRF_SUCCESS) return false;

  return true;
}

bool gatt_init(void)
{
  if (err_code_init != NRF_SUCCESS) return false;

  err_code_init = nrf_ble_gatt_init(&m_gatt, handler_gatt_evt);
  if (err_code_init != NRF_SUCCESS) return false;

  err_code_init = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
  if (err_code_init != NRF_SUCCESS) return false;

  return true;
}

bool services_init(void)
{
  ble_nus_init_t     nus_init;
  nrf_ble_qwr_init_t qwr_init = {0};
  ble_dfu_buttonless_init_t dfus_init = {0};
  ble_dis_init_t     dis_init = {0};


  if (err_code_init != NRF_SUCCESS) return false;

  // Initialize Queued Write Module.
  qwr_init.error_handler = handler_nrf_qwr_error;

  err_code_init = nrf_ble_qwr_init(&m_qwr, &qwr_init);
  if (err_code_init != NRF_SUCCESS) return false;

  // Initialize NUS.
  memset(&nus_init, 0, sizeof(nus_init));

  nus_init.data_handler = handler_nus_data;

  err_code_init = ble_nus_init(&m_nus, &nus_init);
  if (err_code_init != NRF_SUCCESS) return false;


  // Initialize Buttonless DFU.
  dfus_init.evt_handler = handler_ble_dfu_evt;
  err_code_init = ble_dfu_buttonless_init(&dfus_init);
  if (err_code_init != NRF_SUCCESS) return false;


  // Initialize Device Information Service.
  memset(&dis_init, 0, sizeof(dis_init));

  ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
  ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char *)MODEL_NUM);
  ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char *)VERSION_NUM);

  dis_init.dis_char_rd_sec = SEC_OPEN;

  err_code_init = ble_dis_init(&dis_init);
  if (err_code_init != NRF_SUCCESS) return false;

  return true;
}

bool advertising_init(void)
{
  ble_advertising_init_t init;


  if (err_code_init != NRF_SUCCESS) return false;

  memset(&init, 0, sizeof(init));

  init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
  init.advdata.include_appearance = false;
  init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

  init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

  init.config.ble_adv_fast_enabled  = true;
  init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
  init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
  init.evt_handler = handler_on_adv_evt;

  err_code_init = ble_advertising_init(&m_advertising, &init);
  if (err_code_init != NRF_SUCCESS) return false;

  ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

  return true;
}

bool conn_params_init(void)
{
  ble_conn_params_init_t cp_init;


  if (err_code_init != NRF_SUCCESS) return false;

  memset(&cp_init, 0, sizeof(cp_init));

  cp_init.p_conn_params                  = NULL;
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
  cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
  cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
  cp_init.disconnect_on_fail             = false;
  cp_init.evt_handler                    = handler_on_conn_params_evt;
  cp_init.error_handler                  = handler_conn_params_error;

  err_code_init = ble_conn_params_init(&cp_init);
  if (err_code_init != NRF_SUCCESS) return false;

  return true;
}

bool advertising_start(void)
{
  if (err_code_init != NRF_SUCCESS) return false;

  err_code_init = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
  if (err_code_init != NRF_SUCCESS) return false;

  return true;
}

void handler_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
  uint32_t err_code = 0;

  switch (p_ble_evt->header.evt_id)
  {
    case BLE_GAP_EVT_CONNECTED:
      NRF_LOG_INFO("Connected");
      APP_ERROR_CHECK(err_code);
      m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
      err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
      APP_ERROR_CHECK(err_code);

      ledOn(_DEF_LED2);
      is_connect = true;
      err_code = sd_ble_gap_rssi_start(m_conn_handle, 10, 0);
      err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, tx_power_tbl[ble_conn_tx_power_level]);
      break;

    case BLE_GAP_EVT_DISCONNECTED:
      NRF_LOG_INFO("Disconnected");
      // LED indication will be changed when advertising starts.
      err_code = sd_ble_gap_rssi_stop(m_conn_handle);
      m_conn_handle = BLE_CONN_HANDLE_INVALID;
      ledOff(_DEF_LED2);
      is_connect = false;
      break;

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

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
      // Pairing not supported
      err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
      // No system attributes have been stored.
      err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTC_EVT_TIMEOUT:
      // Disconnect on GATT Client timeout event.
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                       BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTS_EVT_TIMEOUT:
      // Disconnect on GATT Server timeout event.
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                       BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;

    default:
      // No implementation needed.
      break;
  }
}

void handler_gatt_evt(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
  if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
  {
      m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
      NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
  }
  NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                p_gatt->att_mtu_desired_central,
                p_gatt->att_mtu_desired_periph);
}

void handler_nrf_qwr_error(uint32_t nrf_error)
{
  APP_ERROR_HANDLER(nrf_error);
}

void handler_nus_data(ble_nus_evt_t * p_evt)
{
  bool ret;

  if (p_evt->type == BLE_NUS_EVT_RX_DATA)
  {
    ret = qbufferWrite(&q_rx, (uint8_t *)p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

    if (ret != true)
    {
      is_q_rx_over = true;
    }
  }

  if (p_evt->type == BLE_NUS_EVT_TX_RDY)
  {
    is_ready_to_send = true;
  }
}

void handler_on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
  uint32_t err_code = 0;

  switch (ble_adv_evt)
  {
    case BLE_ADV_EVT_FAST:
      //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
      APP_ERROR_CHECK(err_code);
      break;
    case BLE_ADV_EVT_IDLE:
      //sleep_mode_enter();
      ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
      break;
    default:
      break;
  }
}

void handler_on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
  uint32_t err_code;

  if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
  {
    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    APP_ERROR_CHECK(err_code);
  }
}

void handler_conn_params_error(uint32_t nrf_error)
{
  APP_ERROR_HANDLER(nrf_error);
}

static void advertising_config_get(ble_adv_modes_config_t * p_config)
{
    memset(p_config, 0, sizeof(ble_adv_modes_config_t));

    p_config->ble_adv_fast_enabled  = true;
    p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
    p_config->ble_adv_fast_timeout  = APP_ADV_DURATION;
}

static void disconnect(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    }
    else
    {
        NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
    }
}

void handler_ble_dfu_evt(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
        {
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");

            // Prevent device from advertising on disconnect.
            ble_adv_modes_config_t config;
            advertising_config_get(&config);
            config.ble_adv_on_disconnect_disabled = true;
            ble_advertising_modes_config_set(&m_advertising, &config);

            // Disconnect all other bonded devices that currently are connected.
            // This is required to receive a service changed indication
            // on bootup after a successful (or aborted) Device Firmware Update.
            uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
            NRF_LOG_INFO("Disconnected %d links.", conn_count);
            break;
        }

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

void appTimerISR(void *args)
{
  app_timer_update();
  app_timer_count++;
}



#ifdef _USE_HW_CLI
void cliCmd(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "info"))
  {
    uint32_t cursor_move = 5;
    ble_uart_rssi_t rssi;


    while(cliKeepLoop())
    {
      bleUartGetRssi(&rssi);

      cliPrintf("BLE Init    : %d\n", is_init);
      cliPrintf("BLE Connect : %d\n", is_connect);
      cliPrintf("BLE Rssi    : %d dB, %d   \n", rssi.rssi, rssi.ch_index);
      cliPrintf("BLE TxPower : %d dB  \n", tx_power_tbl[ble_conn_tx_power_level]);
      cliPrintf("BLE AppTimer: %d     \n", app_timer_count % 10000);
      cliPrintf("\x1B[%dA", cursor_move);
      delay(100);
    }
    cliPrintf("\x1B[%dB", cursor_move);

    ret = true;
  }


  if (ret == false)
  {
    cliPrintf("ble info\n");
  }
}
#endif



