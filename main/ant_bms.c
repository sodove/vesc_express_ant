/*
	ANT BMS BLE bridge for VESC Express

	Connects to ANT BMS (hardcoded MAC C3:5E:E2:61:94:AE) via BLE GATT Client,
	parses battery status data, populates VESC bms_values struct,
	and sends BMS CAN status messages to ESC controllers.

	Uses BLE scanning to find the device, then connects via GATTC.
	Wraps the existing GAP callback to handle scan events while
	forwarding advertising events to comm_ble.

	ANT BMS protocol:
	- Service UUID: 0xFFE0
	- Characteristic UUID: 0xFFE1 (notify + write)
	- Command frame: 7E A1 01 00 00 BE [CRC16-MODBUS 2 bytes] AA 55
	- Response: variable-length frame with CRC16 verification
*/

#include "ant_bms.h"

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_log.h"

#include "nvs_flash.h"
#include "nvs.h"

#include "bms.h"
#include "comm_ble.h"
#include "commands.h"
#include "terminal.h"
#include "conf_general.h"

// ANT BMS BLE UUIDs
#define ANT_SERVICE_UUID   0xFFE0
#define ANT_CHAR_UUID      0xFFE1

// GATTC app ID (GATTS uses 0 in comm_ble.c / custom_ble.c)
#define ANT_GATTC_APP_ID  1

// Defaults
#define ANT_POLL_INTERVAL_DEFAULT  2000
#define ANT_POLL_INTERVAL_MIN      500
#define ANT_POLL_INTERVAL_MAX      10000

// Reconnect delay in ms
#define ANT_RECONNECT_DELAY_MS 3000

// Scan duration in seconds
#define ANT_SCAN_DURATION_S    5

// Response buffer
#define ANT_RX_BUF_SIZE   512

// NVS namespace + keys
#define NVS_NAMESPACE  "ant_bms"
#define NVS_KEY_MAC    "mac"
#define NVS_KEY_POLL   "poll_ms"

// Default MAC: C3:5E:E2:61:94:AE
static const esp_bd_addr_t ant_bms_mac_default = {0xC3, 0x5E, 0xE2, 0x61, 0x94, 0xAE};

// Runtime config (loaded from NVS at init)
static esp_bd_addr_t ant_bms_mac;
static uint32_t m_poll_interval_ms = ANT_POLL_INTERVAL_DEFAULT;

// State machine
typedef enum {
	ANT_STATE_IDLE,
	ANT_STATE_SCANNING,
	ANT_STATE_CONNECTING,
	ANT_STATE_DISCOVERING,
	ANT_STATE_SUBSCRIBING,
	ANT_STATE_POLLING,
} ant_state_t;

// Module state
static volatile ant_state_t m_state = ANT_STATE_IDLE;
static esp_gatt_if_t m_gattc_if = ESP_GATT_IF_NONE;
static uint16_t m_conn_id = 0;
static uint16_t m_char_handle = 0;
static volatile bool m_connected = false;

// Address type discovered from scan
static esp_ble_addr_type_t m_found_addr_type = BLE_ADDR_TYPE_PUBLIC;

// Response accumulator
static uint8_t m_rx_buf[ANT_RX_BUF_SIZE];
static int m_rx_len = 0;

// Timers
static TimerHandle_t m_poll_timer = NULL;
static TimerHandle_t m_reconnect_timer = NULL;
static TimerHandle_t m_heartbeat_timer = NULL;

// ============================================================================
// CRC-16/MODBUS (reflected poly 0xA001)
// ============================================================================

static uint16_t crc16_modbus(const uint8_t *data, int len) {
	uint16_t crc = 0xFFFF;
	for (int i = 0; i < len; i++) {
		crc ^= data[i];
		for (int j = 0; j < 8; j++) {
			if (crc & 1) {
				crc = (crc >> 1) ^ 0xA001;
			} else {
				crc >>= 1;
			}
		}
	}
	return crc;
}

// ============================================================================
// ANT BMS command builder
// ============================================================================

static int ant_build_status_cmd(uint8_t *buf) {
	// Frame: 7E [payload...] [crc_lo] [crc_hi] AA 55
	// CRC-16/MODBUS over payload only (bytes after 0x7E, before CRC)
	buf[0] = 0x7E;
	buf[1] = 0xA1;  // payload starts here
	buf[2] = 0x01;  // func = STATUS
	buf[3] = 0x00;  // addr_lo
	buf[4] = 0x00;  // addr_hi
	buf[5] = 0xBE;  // value

	uint16_t crc = crc16_modbus(&buf[1], 5);  // CRC over payload only (A1 01 00 00 BE)
	buf[6] = crc & 0xFF;         // CRC low byte first (MODBUS LE)
	buf[7] = (crc >> 8) & 0xFF;  // CRC high byte second
	buf[8] = 0xAA;
	buf[9] = 0x55;

	return 10;
}

// ============================================================================
// ANT BMS response parser → populates VESC bms_values
// ============================================================================

// Helper: read uint16 little-endian from buffer
static inline uint16_t rd_u16le(const uint8_t *buf, int off) {
	return (uint16_t)buf[off] | ((uint16_t)buf[off + 1] << 8);
}

// Helper: read int16 little-endian from buffer
static inline int16_t rd_i16le(const uint8_t *buf, int off) {
	uint16_t v = rd_u16le(buf, off);
	return (v >= 0x8000) ? (int16_t)(v - 0x10000) : (int16_t)v;
}

static void ant_parse_status(const uint8_t *buf, int frame_len) {
	// Matches Kotlin parseStatus(): offsets are absolute within the frame
	// Frame: 7E A1 11 XX XX [dataLen] [6..frame header] ... [data] [crc_lo crc_hi] AA 55
	if (frame_len < 50) {
		commands_printf("ANT BMS: frame too short for status (%d < 50)", frame_len);
		return;
	}

	int num_temp = buf[8];
	int num_cells = buf[9];

	// Verbose parse logging removed — only errors printed

	if (num_cells > BMS_MAX_CELLS) num_cells = BMS_MAX_CELLS;
	if (num_temp > BMS_MAX_TEMPS) num_temp = BMS_MAX_TEMPS;

	// Cell voltages start at offset 34, u16LE millivolts
	float v_cell_min = 10.0f;
	float v_cell_max = 0.0f;
	float v_cells[BMS_MAX_CELLS];
	int actual_cells = 0;

	for (int i = 0; i < num_cells; i++) {
		int off = 34 + i * 2;
		if (off + 1 >= frame_len) break;
		uint16_t mv = rd_u16le(buf, off);
		if (mv >= 1 && mv <= 5000) {
			v_cells[actual_cells] = mv / 1000.0f;
			if (v_cells[actual_cells] < v_cell_min) v_cell_min = v_cells[actual_cells];
			if (v_cells[actual_cells] > v_cell_max) v_cell_max = v_cells[actual_cells];
			actual_cells++;
		}
	}

	int pos = 34 + num_cells * 2;

	// Temperatures: u16LE, direct degrees Celsius. 65496 = NaN
	float temps[BMS_MAX_TEMPS];
	float temp_max = -273.0f;
	int actual_temps = 0;

	for (int i = 0; i < num_temp; i++) {
		if (pos + 1 >= frame_len) break;
		uint16_t raw = rd_u16le(buf, pos);
		pos += 2;
		if (raw != 65496 && actual_temps < BMS_MAX_TEMPS) {
			temps[actual_temps] = (float)raw;
			if (temps[actual_temps] > temp_max) temp_max = temps[actual_temps];
			actual_temps++;
		}
	}

	// MOS temperature
	if (pos + 1 < frame_len) {
		uint16_t mos_temp = rd_u16le(buf, pos);
		pos += 2;
		if (mos_temp != 65496 && actual_temps < BMS_MAX_TEMPS) {
			temps[actual_temps] = (float)mos_temp;
			if (temps[actual_temps] > temp_max) temp_max = temps[actual_temps];
			actual_temps++;
		}
	}

	pos += 2; // Skip balancer temp

	// Total voltage: u16LE * 0.01
	float v_tot = 0.0f;
	if (pos + 1 < frame_len) {
		v_tot = (float)rd_u16le(buf, pos) * 0.01f;
		pos += 2;
	}

	// Current: i16LE * 0.1
	float current = 0.0f;
	if (pos + 1 < frame_len) {
		current = (float)rd_i16le(buf, pos) * 0.1f;
		pos += 2;
	}

	// SOC: u16LE (direct percentage, e.g. 85 = 85%)
	float soc = 0.0f;
	if (pos + 1 < frame_len) {
		soc = (float)rd_u16le(buf, pos);
		pos += 2;
	}

	// Write into VESC bms_values
	volatile bms_values *val = bms_get_values();

	val->v_tot = v_tot;
	val->v_charge = 0.0f;
	val->i_in = current;
	val->i_in_ic = current;
	val->soc = soc / 100.0f;  // VESC expects 0.0-1.0
	val->soh = 1.0f;
	val->cell_num = actual_cells;

	for (int i = 0; i < actual_cells; i++) {
		val->v_cell[i] = v_cells[i];
		val->bal_state[i] = false;
	}

	val->temp_adc_num = actual_temps;
	for (int i = 0; i < actual_temps; i++) {
		val->temps_adc[i] = temps[i];
	}

	val->v_cell_min = v_cell_min;
	val->v_cell_max = v_cell_max;
	val->temp_max_cell = temp_max;
	val->is_charging = (current > 0.05f) ? 1 : 0;
	val->is_charge_allowed = 1;
	val->is_balancing = 0;
	val->update_time = xTaskGetTickCount();

	bms_send_status_can();
}

// ============================================================================
// Frame accumulator — BLE notifications may split one response
// ============================================================================

static void ant_process_rx_data(const uint8_t *data, int len) {
	if (m_rx_len + len > ANT_RX_BUF_SIZE) {
		m_rx_len = 0;
	}

	memcpy(m_rx_buf + m_rx_len, data, len);
	m_rx_len += len;

	// Try to parse all complete frames (matches Kotlin tryParseAll)
	while (1) {
		// Find frame start: 7E A1
		int start = -1;
		for (int i = 0; i <= m_rx_len - 2; i++) {
			if (m_rx_buf[i] == 0x7E && m_rx_buf[i + 1] == 0xA1) {
				start = i;
				break;
			}
		}

		if (start < 0) {
			// Keep last byte in case it's 0x7E (partial header)
			if (m_rx_len > 1) {
				m_rx_buf[0] = m_rx_buf[m_rx_len - 1];
				m_rx_len = 1;
			}
			return;
		}

		if (start > 0) {
			m_rx_len -= start;
			memmove(m_rx_buf, m_rx_buf + start, m_rx_len);
		}

		if (m_rx_len < 10) return; // Minimum frame size

		// Data length at byte 5
		int data_len = m_rx_buf[5];
		int frame_len = 6 + data_len + 4; // header(6) + data + crc(2) + trailer(2)

		if (m_rx_len < frame_len) return; // Need more data

		// Verify trailer: AA 55
		if (m_rx_buf[frame_len - 2] != 0xAA || m_rx_buf[frame_len - 1] != 0x55) {
			commands_printf("ANT BMS: bad trailer at flen=%d", frame_len);
			m_rx_len -= 2;
			memmove(m_rx_buf, m_rx_buf + 2, m_rx_len);
			continue;
		}

		// Verify CRC-16/MODBUS over bytes[1 .. frame_len-5] (skip 0x7E, exclude crc+trailer)
		uint16_t crc_expected = rd_u16le(m_rx_buf, frame_len - 4);
		uint16_t crc_computed = crc16_modbus(&m_rx_buf[1], frame_len - 5);

		if (crc_expected != crc_computed) {
			commands_printf("ANT BMS: CRC mismatch (exp=0x%04X got=0x%04X)", crc_expected, crc_computed);
			m_rx_len -= 2;
			memmove(m_rx_buf, m_rx_buf + 2, m_rx_len);
			continue;
		}

		// Dispatch by response code (byte 2)
		int resp_code = m_rx_buf[2];
		if (resp_code == 0x11) {
			ant_parse_status(m_rx_buf, frame_len);
		} else {
			commands_printf("ANT BMS: resp_code=0x%02X (ignored)", resp_code);
		}

		// Consume frame
		if (frame_len < m_rx_len) {
			m_rx_len -= frame_len;
			memmove(m_rx_buf, m_rx_buf + frame_len, m_rx_len);
		} else {
			m_rx_len = 0;
		}
	}
}

// ============================================================================
// BLE scanning + connection
// ============================================================================

static void ant_start_scan(void);
static void ant_try_connect(void);

static void ant_send_poll_cmd(void) {
	if (m_state != ANT_STATE_POLLING || !m_connected) return;

	uint8_t cmd[10];
	int cmd_len = ant_build_status_cmd(cmd);

	esp_ble_gattc_write_char(
		m_gattc_if, m_conn_id, m_char_handle,
		cmd_len, cmd,
		ESP_GATT_WRITE_TYPE_NO_RSP,
		ESP_GATT_AUTH_REQ_NONE
	);
}

static void poll_timer_cb(TimerHandle_t timer) {
	(void)timer;
	ant_send_poll_cmd();
}

static void reconnect_timer_cb(TimerHandle_t timer) {
	(void)timer;
	if (!m_connected && m_state == ANT_STATE_IDLE) {
		ant_start_scan();
	}
}

static void ant_handle_disconnect(void) {
	m_connected = false;
	m_state = ANT_STATE_IDLE;
	m_rx_len = 0;

	if (m_poll_timer) xTimerStop(m_poll_timer, 0);

	commands_printf("ANT BMS: disconnected, reconnecting in %d ms", ANT_RECONNECT_DELAY_MS);

	if (m_reconnect_timer) xTimerStart(m_reconnect_timer, 0);
}

// ============================================================================
// GAP event hook — receives events from comm_ble's GAP handler
// ============================================================================

static void ant_gap_hook(
	esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param
) {
	switch (event) {
	case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
		if (param->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS) {
			commands_printf("ANT BMS: scan params set, starting scan...");
			esp_ble_gap_start_scanning(ANT_SCAN_DURATION_S);
		} else {
			commands_printf("ANT BMS: scan param set failed (%d)", param->scan_param_cmpl.status);
			m_state = ANT_STATE_IDLE;
		}
		break;
	}

	case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: {
		if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
			commands_printf("ANT BMS: scan start failed (%d)", param->scan_start_cmpl.status);
			m_state = ANT_STATE_IDLE;
		}
		break;
	}

	case ESP_GAP_BLE_SCAN_RESULT_EVT: {
		if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
			commands_printf("ANT BMS: scan found %02X:%02X:%02X:%02X:%02X:%02X type=%d rssi=%d",
				param->scan_rst.bda[0], param->scan_rst.bda[1],
				param->scan_rst.bda[2], param->scan_rst.bda[3],
				param->scan_rst.bda[4], param->scan_rst.bda[5],
				param->scan_rst.ble_addr_type,
				param->scan_rst.rssi);

			if (memcmp(param->scan_rst.bda, ant_bms_mac, sizeof(esp_bd_addr_t)) == 0) {
				commands_printf("ANT BMS: FOUND! addr_type=%d, stopping scan and connecting",
					param->scan_rst.ble_addr_type);
				m_found_addr_type = param->scan_rst.ble_addr_type;
				esp_ble_gap_stop_scanning();
				m_state = ANT_STATE_CONNECTING;
				ant_try_connect();
			}
		} else if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT) {
			commands_printf("ANT BMS: scan complete, device not found");
			m_state = ANT_STATE_IDLE;
			if (m_reconnect_timer) xTimerStart(m_reconnect_timer, 0);
		}
		break;
	}

	default:
		break;
	}
}

static const char *state_str(void) {
	switch (m_state) {
		case ANT_STATE_IDLE: return "IDLE";
		case ANT_STATE_SCANNING: return "SCANNING";
		case ANT_STATE_CONNECTING: return "CONNECTING";
		case ANT_STATE_DISCOVERING: return "DISCOVERING";
		case ANT_STATE_SUBSCRIBING: return "SUBSCRIBING";
		case ANT_STATE_POLLING: return "POLLING";
		default: return "???";
	}
}

static void heartbeat_timer_cb(TimerHandle_t timer) {
	(void)timer;
	commands_printf("ANT BMS: [hb v3] state=%s if=%d c=%d",
		state_str(), m_gattc_if, m_connected);
	// Watchdog: if stuck in IDLE, retry scan
	if (m_state == ANT_STATE_IDLE && !m_connected && m_gattc_if != ESP_GATT_IF_NONE) {
		commands_printf("ANT BMS: [hb] retrying scan...");
		ant_start_scan();
	}
}

static void ant_start_scan(void) {
	commands_printf("ANT BMS: ant_start_scan() state=%s gattc_if=%d", state_str(), m_gattc_if);

	if (m_state != ANT_STATE_IDLE || m_gattc_if == ESP_GATT_IF_NONE) {
		commands_printf("ANT BMS: scan skipped (state=%s)", state_str());
		return;
	}

	m_state = ANT_STATE_SCANNING;

	static esp_ble_scan_params_t scan_params = {
		.scan_type = BLE_SCAN_TYPE_ACTIVE,
		.own_addr_type = BLE_ADDR_TYPE_PUBLIC,
		.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
		.scan_interval = 0x50,  // 50ms
		.scan_window = 0x30,    // 30ms
		.scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE,
	};

	esp_err_t err = esp_ble_gap_set_scan_params(&scan_params);
	commands_printf("ANT BMS: set_scan_params returned %d (0=OK)", err);
	if (err != ESP_OK) {
		m_state = ANT_STATE_IDLE;
	}
}

static void ant_try_connect(void) {
	if (m_gattc_if == ESP_GATT_IF_NONE) return;

	commands_printf("ANT BMS: connecting (addr_type=%d)...", m_found_addr_type);

	esp_ble_gattc_open(
		m_gattc_if,
		ant_bms_mac,
		m_found_addr_type,
		true  // direct connection
	);
}

// ============================================================================
// GATTC event handler
// ============================================================================

static void ant_gattc_event_handler(
	esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
	esp_ble_gattc_cb_param_t *param
) {
	// Only handle events for our app interface
	if (gattc_if != ESP_GATT_IF_NONE && gattc_if != m_gattc_if &&
		event != ESP_GATTC_REG_EVT) {
		return;
	}

	switch (event) {
	case ESP_GATTC_REG_EVT: {
		if (param->reg.app_id != ANT_GATTC_APP_ID) break;

		if (param->reg.status == ESP_GATT_OK) {
			m_gattc_if = gattc_if;
			commands_printf("ANT BMS: GATTC registered (if=%d), starting scan...", gattc_if);
			ant_start_scan();
		} else {
			commands_printf("ANT BMS: GATTC register failed (%d)", param->reg.status);
		}
		break;
	}

	case ESP_GATTC_OPEN_EVT: {
		if (param->open.status == ESP_GATT_OK) {
			m_conn_id = param->open.conn_id;
			m_connected = true;
			m_state = ANT_STATE_DISCOVERING;
			commands_printf("ANT BMS: connected (conn_id=%d), discovering svc 0x%04X", m_conn_id, ANT_SERVICE_UUID);
			esp_bt_uuid_t svc_filter = {
				.len = ESP_UUID_LEN_16,
				.uuid.uuid16 = ANT_SERVICE_UUID,
			};
			esp_ble_gattc_search_service(gattc_if, m_conn_id, &svc_filter);
		} else {
			commands_printf("ANT BMS: open failed (%d)", param->open.status);
			m_state = ANT_STATE_IDLE;
			ant_handle_disconnect();
		}
		break;
	}

	case ESP_GATTC_SEARCH_RES_EVT: {
		commands_printf("ANT BMS: svc uuid_len=%d uuid16=0x%04X",
			param->search_res.srvc_id.uuid.len,
			param->search_res.srvc_id.uuid.uuid.uuid16);
		break;
	}

	case ESP_GATTC_SEARCH_CMPL_EVT: {
		commands_printf("ANT BMS: search_cmpl st=%d", param->search_cmpl.status);
		if (m_state != ANT_STATE_DISCOVERING) break;

		// Use stack buffer (no calloc) — only need 1 char result
		esp_gattc_char_elem_t char_result;
		uint16_t count = 1;
		esp_bt_uuid_t char_uuid = {
			.len = ESP_UUID_LEN_16,
			.uuid.uuid16 = ANT_CHAR_UUID,
		};

		esp_gatt_status_t ret = esp_ble_gattc_get_char_by_uuid(
			gattc_if, m_conn_id, 0x0001, 0xFFFF,
			char_uuid, &char_result, &count
		);
		commands_printf("ANT BMS: get_char ret=%d count=%d", ret, count);

		if (ret == ESP_GATT_OK && count > 0) {
			m_char_handle = char_result.char_handle;
			commands_printf("ANT BMS: char h=%d, subscribing", m_char_handle);
			m_state = ANT_STATE_SUBSCRIBING;
			esp_ble_gattc_register_for_notify(gattc_if, ant_bms_mac, m_char_handle);
		} else {
			commands_printf("ANT BMS: char 0xFFE1 not found");
			esp_ble_gattc_close(gattc_if, m_conn_id);
		}
		break;
	}

	case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
		commands_printf("ANT BMS: reg_for_notify status=%d", param->reg_for_notify.status);
		if (param->reg_for_notify.status != ESP_GATT_OK) {
			esp_ble_gattc_close(gattc_if, m_conn_id);
			break;
		}

		// Write CCCD to enable notifications on the remote
		esp_gattc_descr_elem_t descr_result;
		uint16_t count = 1;
		esp_bt_uuid_t cccd_uuid = {
			.len = ESP_UUID_LEN_16,
			.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
		};

		esp_gatt_status_t ret = esp_ble_gattc_get_descr_by_char_handle(
			gattc_if, m_conn_id, m_char_handle,
			cccd_uuid, &descr_result, &count
		);
		commands_printf("ANT BMS: CCCD lookup ret=%d count=%d", ret, count);

		if (ret == ESP_GATT_OK && count > 0) {
			uint16_t notify_en = 0x0001;
			commands_printf("ANT BMS: writing CCCD handle=%d val=0x0001", descr_result.handle);
			esp_ble_gattc_write_char_descr(
				gattc_if, m_conn_id, descr_result.handle,
				sizeof(notify_en), (uint8_t *)&notify_en,
				ESP_GATT_WRITE_TYPE_RSP,
				ESP_GATT_AUTH_REQ_NONE
			);
		} else {
			commands_printf("ANT BMS: CCCD not found! notify may not work");
		}

		m_state = ANT_STATE_POLLING;
		commands_printf("ANT BMS: ready, polling every %lu ms", (unsigned long)m_poll_interval_ms);

		ant_send_poll_cmd();
		if (m_poll_timer) xTimerStart(m_poll_timer, 0);
		break;
	}

	case ESP_GATTC_NOTIFY_EVT: {
		if (param->notify.handle == m_char_handle) {
			ant_process_rx_data(param->notify.value, param->notify.value_len);
		}
		break;
	}

	case ESP_GATTC_WRITE_CHAR_EVT: {
		if (param->write.status != ESP_GATT_OK) {
			commands_printf("ANT BMS: write failed (%d)", param->write.status);
		}
		break;
	}

	case ESP_GATTC_DISCONNECT_EVT: {
		commands_printf("ANT BMS: disconnected (reason=0x%X)", param->disconnect.reason);
		ant_handle_disconnect();
		break;
	}

	default:
		break;
	}
}

// ============================================================================
// NVS persistence
// ============================================================================

static void ant_nvs_load(void) {
	memcpy(ant_bms_mac, ant_bms_mac_default, sizeof(esp_bd_addr_t));
	m_poll_interval_ms = ANT_POLL_INTERVAL_DEFAULT;

	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
		size_t len = sizeof(esp_bd_addr_t);
		nvs_get_blob(h, NVS_KEY_MAC, ant_bms_mac, &len);
		nvs_get_u32(h, NVS_KEY_POLL, &m_poll_interval_ms);
		nvs_close(h);
	}
}

static void ant_nvs_save_mac(void) {
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
		nvs_set_blob(h, NVS_KEY_MAC, ant_bms_mac, sizeof(esp_bd_addr_t));
		nvs_commit(h);
		nvs_close(h);
	}
}

static void ant_nvs_save_poll(void) {
	nvs_handle_t h;
	if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
		nvs_set_u32(h, NVS_KEY_POLL, m_poll_interval_ms);
		nvs_commit(h);
		nvs_close(h);
	}
}

// ============================================================================
// Terminal command: ant_bms [status|mac XX:XX:XX:XX:XX:XX|poll <ms>]
// ============================================================================

static int parse_mac(const char *str, esp_bd_addr_t out) {
	unsigned int b[6];
	if (sscanf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
			&b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) {
		return -1;
	}
	for (int i = 0; i < 6; i++) out[i] = (uint8_t)b[i];
	return 0;
}

static void ant_terminal_cmd(int argc, const char **argv) {
	if (argc < 2 || strcmp(argv[1], "status") == 0) {
		commands_printf("ANT BMS status:");
		commands_printf("  MAC:       %02X:%02X:%02X:%02X:%02X:%02X",
			ant_bms_mac[0], ant_bms_mac[1], ant_bms_mac[2],
			ant_bms_mac[3], ant_bms_mac[4], ant_bms_mac[5]);
		commands_printf("  State:     %s", state_str());
		commands_printf("  Connected: %s", m_connected ? "yes" : "no");
		commands_printf("  Poll:      %lu ms", (unsigned long)m_poll_interval_ms);

		if (m_connected) {
			volatile bms_values *val = bms_get_values();
			commands_printf("  Voltage:   %.2f V", (double)val->v_tot);
			commands_printf("  Current:   %.1f A", (double)val->i_in);
			commands_printf("  SOC:       %.0f %%", (double)(val->soc * 100.0f));
			commands_printf("  Cells:     %d (%.3f - %.3f V)",
				val->cell_num, (double)val->v_cell_min, (double)val->v_cell_max);
			commands_printf("  Temps:     %d (max %.0f C)",
				val->temp_adc_num, (double)val->temp_max_cell);
		}
		return;
	}

	if (strcmp(argv[1], "mac") == 0) {
		if (argc < 3) {
			commands_printf("Usage: ant_bms mac XX:XX:XX:XX:XX:XX");
			return;
		}
		esp_bd_addr_t new_mac;
		if (parse_mac(argv[2], new_mac) < 0) {
			commands_printf("Invalid MAC format. Use XX:XX:XX:XX:XX:XX");
			return;
		}
		memcpy(ant_bms_mac, new_mac, sizeof(esp_bd_addr_t));
		ant_nvs_save_mac();
		commands_printf("MAC set to %02X:%02X:%02X:%02X:%02X:%02X (saved, reboot to apply)",
			ant_bms_mac[0], ant_bms_mac[1], ant_bms_mac[2],
			ant_bms_mac[3], ant_bms_mac[4], ant_bms_mac[5]);
		return;
	}

	if (strcmp(argv[1], "poll") == 0) {
		if (argc < 3) {
			commands_printf("Usage: ant_bms poll <ms> (%d-%d)", ANT_POLL_INTERVAL_MIN, ANT_POLL_INTERVAL_MAX);
			return;
		}
		int val = atoi(argv[2]);
		if (val < ANT_POLL_INTERVAL_MIN || val > ANT_POLL_INTERVAL_MAX) {
			commands_printf("Poll interval must be %d-%d ms", ANT_POLL_INTERVAL_MIN, ANT_POLL_INTERVAL_MAX);
			return;
		}
		m_poll_interval_ms = (uint32_t)val;
		ant_nvs_save_poll();
		// Update running timer period
		if (m_poll_timer) {
			xTimerChangePeriod(m_poll_timer, pdMS_TO_TICKS(m_poll_interval_ms), 0);
		}
		commands_printf("Poll interval set to %lu ms (saved)", (unsigned long)m_poll_interval_ms);
		return;
	}

	commands_printf("Usage: ant_bms [status|mac XX:XX:XX:XX:XX:XX|poll <ms>]");
}

// ============================================================================
// Public API
// ============================================================================

void ant_bms_init(void) {
	ant_nvs_load();

	commands_printf("ANT BMS: init (target %02X:%02X:%02X:%02X:%02X:%02X, poll %lu ms)",
		ant_bms_mac[0], ant_bms_mac[1], ant_bms_mac[2],
		ant_bms_mac[3], ant_bms_mac[4], ant_bms_mac[5],
		(unsigned long)m_poll_interval_ms);

	m_poll_timer = xTimerCreate(
		"ant_poll", pdMS_TO_TICKS(m_poll_interval_ms),
		pdTRUE, NULL, poll_timer_cb
	);

	m_reconnect_timer = xTimerCreate(
		"ant_recon", pdMS_TO_TICKS(ANT_RECONNECT_DELAY_MS),
		pdFALSE, NULL, reconnect_timer_cb
	);

	// Heartbeat timer — prints state every 10s for diagnostics
	m_heartbeat_timer = xTimerCreate(
		"ant_hb", pdMS_TO_TICKS(10000),
		pdTRUE, NULL, heartbeat_timer_cb
	);
	xTimerStart(m_heartbeat_timer, 0);

	// Register terminal command
	terminal_register_command_callback(
		"ant_bms",
		"ANT BMS bridge control",
		"[status|mac XX:XX:XX:XX:XX:XX|poll <ms>]",
		ant_terminal_cmd
	);

	// Install GAP hook into comm_ble's handler for scan events
	comm_ble_set_gap_hook(ant_gap_hook);

	// Register GATTC callback and app
	esp_ble_gattc_register_callback(ant_gattc_event_handler);
	esp_err_t err = esp_ble_gattc_app_register(ANT_GATTC_APP_ID);
	if (err != ESP_OK) {
		commands_printf("ANT BMS: gattc_app_register failed (%d)", err);
	}
}

bool ant_bms_is_connected(void) {
	return m_connected;
}
