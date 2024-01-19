# RF210 SB

### Additional Information
RF210_SB firmware ATC
version 0.1.0

Supported sensors:
- [Tri-Axis Accelerometer - KX023-1025](https://www.rohm.com/products/sensors-mems/accelerometer-ics/kx023-1025-product)
- [Ambient Light Sensor - LTR-303ALS-01](https://optoelectronics.liteon.com/en-global/Led/led-component/Detail/926)
- [Indicator for indoor air quality - SCD40](https://sensirion.com/products/catalog/SCD40)
- [High Sensitivity Digital PIR Modlue - SB412](https://www.senbasensor.com/pir-sensor-module/sb412.html)

  
Command general format: ```ATC+<cmd>=?```

- ```ATC+VER=?``` : return version of the firmware
- ```ATC+SCDCO2=?``` : Return the value of CO2 of SCD41, return 0 if not available.
- ```ATC+SCDTEMP=?``` : Return the temperature value with 0.01° resolution, return 0 if not available.
- ```ATC+SCDHUM=?``` : Return the humidity value with 1% resolution, return 0 if not available.
- ```ATC+BMEGAS=?``` : Return the gas resistivity in ohmable, return 0 if not available.
- ```ATC+BMETEMP=?``` : Return the temperature value with 0.01° resolution, return 0 if not available.
- ```ATC+BMEHUM=?``` : Return the humidity value with 1% resolution, return 0 if not available.
- ```ATC+BMEBAR=?``` : Return the pressure value in mbar, return 0 if not available.
- ```ATC+TEMP=?``` : Return the temperature value with 0.01° resolution, return 0 if not available.
- ```ATC+HUM=?``` : Return the humidity value with 1% resolution, return 0 if not available.
- ```ATC+KX023=?``` : Return the status of the KX023 sensor. 1 if available, return 0 if not available.
- ```ATC+AX=?``` : Return the value of X acceleration with 0.01G resolution, return 0 if not available.
- ```ATC+AY=?``` : Return the value of Y acceleration with 0.01G resolution, return 0 if not available.
- ```ATC+AZ=?``` : Return the value of Z acceleration with 0.01G resolution, return 0 if not available.
- ```ATC+LTR=?``` : Return the status of the LTR-303 sensor. 1 if available, return 0 if not available.
- ```ATC+LUMCH0=?``` : Return the CHANNEL0 value of the LTR-303 sensor, return 0 if not available.
- ```ATC+LUMCH1=?``` : Return the CHANNEL1 value of the LTR-303 sensor, return 0 if not available.
- ```ATC+LUM=?``` : Return the CHANNEL1 value of the LTR-303 sensor, return 0 if not available.
- ```ATC+BAT=?``` : Return battery voltage in mV. Return 0 if not available.
- ```ATC+LDO=?``` : Return LDO voltage in mV. Return 0 if not available.
 
## Getting Started

### Hardware

- USB to UART Converter
- RFThings RF210_SB Board

### Sortware

- Arduino IDE (version v1.8.13 or above is recommended)
- RUI3 lastest firmware for RAK3172: [RAK3172-E_latest_final.hex](https://downloads.rakwireless.com/RUI/RUI3/Image/RAK3172-E_latest_final.hex)
- (STM32CubeProgammer)[https://www.st.com/en/development-tools/stm32cubeprog.html]
  
### Additional Libraries

- Zanshin_BME680.h
- LTR303.h
- SensirionI2CScd4x.h
- Kionix_KX023.h
  
## Installation

### Hardware connection:

<!-- ![image](https://github.com/XuanMinh201/RF210/assets/75436464/ea7faa22-7082-44b6-a6ea-442fadfd687f) -->
<img src="https://github.com/XuanMinh201/RF210/assets/75436464/ea7faa22-7082-44b6-a6ea-442fadfd687f" height="450">

### On the RF210_SB board:

<!-- ![image](https://github.com/XuanMinh201/SMART-BUILDING/assets/75436464/54157f08-2bba-46f8-a7ce-686ccdbba386) -->
<img src = "https://github.com/XuanMinh201/SMART-BUILDING/assets/75436464/54157f08-2bba-46f8-a7ce-686ccdbba386" height="450">


### In STM32CubeProgammer:
  -  Hold the **B_RAK (boot)** button and press **R_RAK (reset)** button and release the **B_RAK (boot)** button to enter bootmode.
  -  Select UART, Baudrate 115200 and press Connect.
  -  Open RAK3172-E_latest_final.hex
  -  Select the address as in following image if needed
  -  Press Download to upload firmwave
  -  After download success, press **R_RAK (reset)** button to exit the bootmode
  
<!-- ![image](https://github.com/XuanMinh201/RF210/assets/75436464/55f5c5ab-d69a-4a25-94da-563d1e52a172) -->
<img src="https://github.com/XuanMinh201/RF210/assets/75436464/55f5c5ab-d69a-4a25-94da-563d1e52a172" height="450">
 

### In Arduino IDE:
  -  Add this JSON in Additional Boards Manager URLs [\(Show me how?\)](https://support.arduino.cc/hc/en-us/articles/360016466340-Add-third-party-platforms-to-the-Boards-Manager-in-Arduino-IDE):

```  
https://raw.githubusercontent.com/RAKWireless/RAKwireless-Arduino-BSP-Index/main/package_rakwireless.com_rui_index.json
```

  -  Go to **Tool -> Board -> Boards Manager**, search & install **RAKwireless RUI STM32 Boards**
  -  Open ```ATC_Command_RF210.ino``` sketch, seletc **WisDou RAK3172-T Board** from **Tool** menu
  -  Plug in your board and upload


<!-- ![image](https://github.com/XuanMinh201/RF210/assets/75436464/141710ed-1294-46ea-9951-63bea73622ed) -->
<img src="https://github.com/XuanMinh201/RF210/assets/75436464/141710ed-1294-46ea-9951-63bea73622ed" height="450">

  -  Select **Tool -> Board -> RAKwireless RUI STM32 Modules -> WisDuo RAK3172 Evaluation Board**
    
<!-- ![image](https://github.com/XuanMinh201/RF210/assets/75436464/146c570a-ec82-45bc-ada0-89544624b861) -->
<img src="https://github.com/XuanMinh201/RF210/assets/75436464/146c570a-ec82-45bc-ada0-89544624b861" height="450">
