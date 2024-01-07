This is a ABRP livedata client running on the Freematic ONE+ Model B.
Made by https://github.com/spot2000

Hardware do you find here:
* Freematics ONE+ [Model B] (https://freematics.com/products/freematics-one-plus-model-b/) (mainly supported)
* Freematics ONE+ [Model A] (https://freematics.com/products/freematics-one-plus/) (should work)
* Freematics ONE+ [Model H] (https://freematics.com/products/freematics-one-plus-model-h/) (should work)


You can find the latest version here:
https://github.com/spot2000/Freematics-ABRP

Freematic ONE+ Model B hardware info:
-------------------------------------
Features
--------
Dual-core Arduino programmable SoC with built-in WiFi and Bluetooth
Access to all standard OBD-II PIDs, DTC, VIN from vehicle ECU
CAN bus data sniffing
High update rate and accuracy GNSS geolocation
G-force measurement and motion detection
Car battery voltage reading
Massive data storage (microSD up to 32GB)
Real-time data transmission over WiFi and cellular network
Configuring and monitoring from mobile device app via BLE

Specifications
--------------
Espressif ESP32 with 16MB Flash, 8MB PSRAM, 32K RTC
Built-in 802.11 b/g/n Wi-Fi and dual mode Bluetooth (classic and BLE)
Integrated ICM-42627 motion sensor
Integrated u-blox M10 GNSS module and antenna
Integrated SIM7070G global LTE-M cellular module
Integrated buzzer
Enclosure dimensions: 60x48x20mm

Physical Interfaces
-------------------
OBD-II male connector
microUSB port
microSD card slot
SIM card slot
GPIO socket (Molex)

Data Collection
---------------

This project collects the following data:
-----------------------------------------
* Geolocation data (from internal or external GNSS) 
* Accelerometer and gyroscope data (from internal MEMS motion sensor)
* Cellular or WiFi network signal level
* Device temperature
* EV vehicle data via OBD port (See ABRP data list below)

List of ABRP data that is collected:
------------------------------------

High priority parameters:
-------------------------
utc [s]: Current UTC timestamp (epoch) in seconds (note, not milliseconds!)
soc [SoC %]: State of Charge of the vehicle (what's displayed on the dashboard of the vehicle is preferred)
power [kW]: Instantaneous power output/input to the vehicle. Power output is positive, power input is negative (charging)
speed [km/h]: Vehicle speed
lat [°]: Current vehicle latitude
lon [°]: Current vehicle longitude
is_charging [bool or 1/0]: Determines vehicle state. 0 is not charging, 1 is charging
is_dcfc [bool or 1/0]: If is_charging, indicate if this is DC fast charging
is_parked [bool or 1/0]: If the vehicle gear is in P (or the driver has left the car)

The lower priority parameters (and expected units) are:
-------------------------------------------------------
capacity [kWh]: Estimated usable battery capacity (can be given together with soh, but usually not)
kwh_charged [kWh]: Measured energy input while charging. Typically a cumulative total, but also supports individual sessions.
soh [%]: State of Health of the battery. 100 = no degradation
heading [°]: Current heading of the vehicle. This will take priority over phone heading, so don't include if not accurate.
elevation [m]: Vehicle's current elevation. If not given, will be looked up from location (but may miss 3D structures)
ext_temp [°C]: Outside temperature measured by the vehicle
batt_temp [°C]: Battery temperature
voltage [V]: Battery pack voltage
current [A]: Battery pack current (similar to power: output is positive, input (charging) is negative.)
odometer [km]: Current odometer reading in km.
est_battery_range [km]: Estimated remaining range of the vehicle (according to the vehicle)

Data storage:
-------------
Collected data are stored in a circular buffer in ESP32's IRAM or PSRAM. When PSRAM is enabled, hours of data can be buffered in case of temporary network outage and transmitted when network connection resumes.

Configuration handling:
-----------------------
Configuration is done in the config.cfg file that is located in the config folder of the SD-card.
See config file for more information.


Data Transmission
-----------------

Data transmission over UDP and HTTP(s) protocols are implemented for the followings.

* WiFi (ESP32 built-in)
* 3G WCDMA (SIM5360)
* 4G LTE CAT-4 (SIM7600)
* 4G LTE CAT-M (SIM7070)

Seamless WiFi and cellular network co-working is implemented. When defined WiFi hotspot is available, data is transmitted via WiFi and cellular module is switched off. When no WiFi hotspot can be reached, cellular module is switched on for data transmission until WiFi hotspot available again.

Data Storage
------------
Following types of data storage are supported.

* MicroSD card storage
* ESP32 built-in Flash memory storage (SPIFFS)

BLE & App
---------
Planned support in the future for ability to link phone to Freematic adapter to view data in a easy way. (not ready yet)
