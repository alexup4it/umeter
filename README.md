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
|period_app|uint32|Communication with application server period (seconds)|
|period_sen|uint32|Sensors data update period (seconds)|
|mtime_count|uint32|Measurement time for counter (seconds)|
|sens|int32|[Available sensors](#available-sensors) bit mask|

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
POST  
_Every_ `period_app` _seconds_  
Send device state and sensors information

Request JSON:
|Field|Type|Optional|Description|
|-|-|-|-|
|uid|uint32|Always present|Unique device ID|
|ts|uint32|Always present|Current datetime (Unix timestamp)|
|bat|int32|Optional|Battery voltage (mV)|
|count|string [sensor_base64](#sensor_base64)|Optional|Number of counts per `mtime_count` seconds (average value per `period_sen` seconds)|
|count_min|string [sensor_base64](#sensor_base64)|Optional|Number of counts per `mtime_count` seconds (minimal value per `period_sen` seconds)|
|count_max|string [sensor_base64](#sensor_base64)|Optional|Number of counts per `mtime_count` seconds (maximum value per `period_sen` seconds)|
|temp|string [sensor_base64](#sensor_base64)|Optional|Temperature (0,001 °C)|
|hum|string [sensor_base64](#sensor_base64)|Optional|Humidity (0,001%)|
|angle|string [sensor_base64](#sensor_base64)|Optional|Angle (0,001°)|
|tamper|int32|Optional|Digital input value (0 or 1)|

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

### Available sensors
|Bit|Sensor|
|-|-|
|0x01 (xxxx xxx1)|Voltage|
|0x02 (xxxx xx1x)|Counter _(not used)_|
|0x04 (xxxx x1xx)|TMPx75 _(not used)_|
|0x08 (xxxx 1xxx)|AHT20|
|0x10 (xxx1 xxxx)|Distance _(not used)_|
|0x20 (xx1x xxxx)|AS5600|

### sensor_base64
Array of data structures in base64 encoding  
Sensor data structure:
|Offset|Length|Endian|Description|
|-|-|-|-|
|0 bytes|4 bytes|little-endian|Value|
|4 bytes|4 bytes|little-endian|Measurement datetime (Unix timestamp)|

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

