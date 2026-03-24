# umeter
umeter, FreeRTOS, SIM800L

## Application API
HTTP packets (without SSL) with "Authorization" header  
Authorization: payload HMAC SHA256 in base64 encoding

### /api/time
GET  
_At startup, every 24 hours_  
Get current datetime

Response JSON:
|Field|Type|Description|
|-|-|-|
|status|string|"ok" on success|
|ts|uint32|Current datetime (Unix timestamp)|

### /api/info
POST  
_At startup only_  
Send base device information

Request JSON:
|Field|Type|Description|
|-|-|-|
|uid|uint32|Unique device ID|
|ts|uint32|Current datetime (Unix timestamp)|
|name|string|Device type|
|bl_git|string|Bootloader source revision|
|bl_status|uint32|[Bootloader status](#bootloader-status)|
|app_git|string|Application source revision|
|app_ver|uint32|Application version|
|mcu|string|MCU unique ID|
|apn|string|APN for cellular network|
|url_ota|string|OTA server URL|
|url_app|string|Application server URL|
|period_upload|uint32|Communication with application server period (seconds)|
|period_sensors|uint32|Sensors data update period (seconds)|
|period_anemometer|uint32|Measurement time for anemometer (seconds)|

Response JSON:
|Field|Type|Description|
|-|-|-|
|status|string|"ok" on success|

### /api/cnet
POST  
_At startup only_  
Send information about the nearest cellular base station 

Request JSON:
|Field|Type|Description|
|-|-|-|
|uid|uint32|Unique device ID|
|ts|uint32|Current datetime (Unix timestamp)|
|mcc|int32|Mobile Country Code|
|mnc|int32|Mobile Network Code|
|lac|int32|Location Area Code|
|cid|int32|Cell ID|
|lev|int32|The current signal strength (dBm)|

Response JSON:
|Field|Type|Description|
|-|-|-|
|status|string|"ok" on success|

### /api/data
POST `Content-Type: application/octet-stream`  
_Every_ `period_upload` _seconds_  
Send device state and sensor measurements in compact binary format

Request body is raw binary (little-endian throughout):

**Header (14 bytes)**
|Offset|Size|Type|Description|
|-|-|-|-|
|0|4|uint32|Unique device ID|
|4|4|uint32|Current datetime (Unix timestamp)|
|8|4|uint32|System uptime (FreeRTOS ticks)|
|12|1|uint8|Tamper input (0 or 1)|
|13|1|uint8|Number of sensor records (N)|

**Sensor record (20 bytes each, N records)**
|Offset|Size|Type|Description|
|-|-|-|-|
|0|4|uint32|Measurement datetime (Unix timestamp)|
|4|2|uint16|Battery voltage (mV)|
|6|2|int16|Temperature (0.01 °C)|
|8|2|uint16|Humidity (0.01 %RH)|
|10|2|uint16|Pressure (0.1 hPa)|
|12|2|uint16|Wind direction (0.01°)|
|14|2|uint16|Wind speed average (per `period_anemometer` seconds)|
|16|2|uint16|Wind speed minimum (per `period_anemometer` seconds)|
|18|2|uint16|Wind speed maximum (per `period_anemometer` seconds)|

Total payload size: 14 + N × 20 bytes (max N = 50, max payload = 1014 bytes)

Response JSON:
|Field|Type|Description|
|-|-|-|
|status|string|"ok" on success|

### Bootloader status
|Status|Description|
|-|-|
|0|No new firmware|
|1|Successfully updated|
|2|Error: no external storage|
|3|Error: invalid firmware size|
|4|Error: invalid checksum (external storage)|
|5|Error: invalid checksum (internal memory)|
|6|Error: could not erase internal memory|

## OTA API
**_?_**

## Serial API
**_?_**

## Development setup
Init submodules:
```sh
git submodule init
git submodule update
```

Activate git hooks:
```sh
sh install-hooks.sh
```

