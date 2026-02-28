# VESC Express

The is the codebase for the VESC Express, which is a WiFi and Bluetooth-enabled logger and IO-board. At the moment it is tested and runs on the ESP32C3 and ESP32S3 but other ESP32 devices can be added.

## Toolchain

Instructions for how to set up the toolchain can be found here:
[https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/linux-macos-setup.html](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/linux-macos-setup.html)

**Note**  
ESP-IDF version 5.2 or later is required for building this project.

### Get Release 5.2.2

The instructions linked above will install the master branch of ESP-IDF. To install the stable release you can navigate to the installation directory and use the following commands:

```bash
git clone -b v5.2.2 --recursive https://github.com/espressif/esp-idf.git esp-idf-v5.2.2
cd esp-idf-v5.2.2/
./install.sh esp32c3 esp32s3
```

At the moment development is done using the stable 5.2.2-release.

## Building

Set the target chip/architecture with 
```bash
idf.py set-target <target> 
```

where target is esp32c3 or esp32s3. You will need to run a fullclean or remove the build directory when changing targets.

Once the toolchain is set up in the current path, the project can be built with

```bash
idf.py build
```

That will create vesc_express.bin in the build directory, which can be used with the bootloader in VESC Tool. If the ESP32c3 does not come with firmware preinstalled, the USB-port can be used for flashing firmware using the built-in bootloader. That also requires bootloader.bin and partition-table.bin which also can be found in the build directory. This can be done from VESC Tool or using idf.py.

All targets can be built with

```bash
python build_all.py
```

That will create all required firmware files under the build_output directory, with hardware names as child directories. All target switching is handled automatically with the build_all command.

### Custom Hardware Targets

If you wish to build the project with custom hardware config files you should add the hardware config files to the "**main/hwconf**" directory and use the HW_NAME build flag
```bash
idf.py build -DHW_NAME="VESC Express T"
```

**Note:** If you ever change the environment variables, or if when you first start using them, you need to first run `idf.py reconfigure` before building (with the environment variables still set of course!), as the build system unfortunately can't automatically detect this change. Running `idf.py fullclean` has the same effect as this forces cmake to rebuild the build configurations.

### Windows Build Helper

A `build_idf.ps1` script is provided for Windows. It handles the MSYSTEM env var cleanup and Python PATH setup automatically.

```powershell
.\build_idf.ps1 build                              # build with default HW
.\build_idf.ps1 -DHW_NAME="sodovaya express" build  # build with custom HW
.\build_idf.ps1 fullclean                            # clean build
.\build_idf.ps1 flash                                # flash via USB
```

Environment variables (optional):
- `IDF_PATH` — path to ESP-IDF installation (default: `C:\esp\esp-idf-v5.2.2`)
- `PYTHON3_DIR` — path to Python 3.10+ directory (default: `%LOCALAPPDATA%\Programs\Python\Python310`)

## ANT BMS Bridge

This fork adds a BLE GATT Client bridge that connects to an ANT BMS, reads battery data, and forwards it to the VESC ecosystem as standard BMS CAN messages. The VESC ESC sees it as a native BMS — cell voltages, temperatures, current, SOC all appear in VESC Tool's BMS page.

The idea for this bridge was inspired by [kamiyamaQ](https://www.youtube.com/@kamiyamaQ).

**WARNING:** This is experimental custom firmware. It may be unstable, cause unexpected behavior, or brick your device. Flash at your own risk. The authors take no responsibility for any damage to your hardware.

**Tested only with [Flipsky FSESC Express](https://flipsky.net/collections/fsesc-express/products/flipsky-fsesc-express).** No other hardware has been tested.

### How it works

The ESP32-C3 runs three radios simultaneously:
- WiFi — VESC Tool connection (TCP)
- BLE Peripheral (GATTS) — VESC Tool mobile connection
- BLE Central (GATTC) — ANT BMS connection

On boot, the Express scans for the configured ANT BMS MAC address, connects via BLE, subscribes to notifications on the 0xFFE1 characteristic, and polls for status every 2 seconds. Parsed data is written into the VESC `bms_values` struct and sent over CAN via `bms_send_status_can()`.

### Data forwarded

```
Cell voltages (up to 32)    u16LE millivolts @ offset 34
Total voltage               u16LE * 0.01V
Current                     i16LE * 0.1A
SOC                         u16LE (%)
SOH                         u16LE (%)
Cell temperatures           u16LE direct °C (65496 = no sensor)
MOS temperature             u16LE direct °C  ->  Temp PCB in VESC Tool (max of MOS and balancer)
Balancer temperature        u16LE direct °C  ->  Temp Hum in VESC Tool (MOS temp separately)
Charge/discharge switches   u8 flags
Balancer state              u16LE bitmask (1 bit per cell)
Remaining capacity          u32LE * 0.000001 Ah  ->  Ah counter + Wh counter in VESC Tool
```

### Terminal commands

Connect to the Express via VESC Tool > VESC Dev Tools > Terminal:

```
ant_bms                            show status: MAC, state, voltage, current, SOC, SOH,
                                     cells, temps, Temp PCB (MOS/BAL breakdown),
                                     remaining Ah/Wh, balancing state, charge allowed
ant_bms mac XX:XX:XX:XX:XX:XX     change target BMS MAC (saved to NVS, reboot to apply)
ant_bms poll 500                   change poll interval in ms (500-10000, applied immediately, saved to NVS)
```

### Configuration

The default target MAC is `C3:5E:E2:61:94:AE` (hardcoded in `ant_bms.c`). Change it at runtime via the terminal command above — it persists across reboots and OTA updates (stored in NVS flash).

The default poll interval is 2000ms. With WiFi + BLE Peripheral + BLE Central all active, 500ms has been tested stable on ESP32-C3, but 2000ms is the safe default.

### Hardware config

The `hw_sodovaya` config is based on `hw_xp_t` (VESC Express T) with the same pinout. To build:

```bash
idf.py build -DHW_NAME="sodovaya express"
```

### Files

```
main/ant_bms.c              BLE GATTC bridge: scan, connect, discover, subscribe, poll, parse, CAN send
main/ant_bms.h              Public API (ant_bms_init, ant_bms_is_connected)
main/hwconf/hw_sodovaya.h   Hardware config (ESP32-C3, same pinout as Express T)
main/hwconf/hw_sodovaya.c   Hardware init
main/comm_ble.c             Added GAP event hook for GATTC scan coexistence
main/comm_ble.h             Exported comm_ble_set_gap_hook()
main/main.c                 ant_bms_init() call after WiFi init
sdkconfig.defaults          GATTC enabled, BLE MAX_ACT=6, increased BT stack sizes
```

### ANT BMS protocol notes

- BLE service 0xFFE0, characteristic 0xFFE1 (notify + write-no-response)
- Command frame: `7E [payload] [CRC16-MODBUS LE] AA 55` — CRC over payload only (excludes 0x7E header)
- Response frame: `7E A1 [code] XX XX [data_len] [data...] [CRC16 LE] AA 55`
- All multi-byte fields are little-endian
- Status response code: 0x11
