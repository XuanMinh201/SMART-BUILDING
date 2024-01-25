/**

   RRRR  FFFF  22   11   000
   R   R F    2  2 111  0  00
   RRRR  FFF    2   11  0 0 0
   R R   F     2    11  00  0
   R  RR F    2222 11l1  000


   @file ATC_Command_RF210.ino
   @author XuanMinh201, FabienFerrero

   @brief This sketch add custom ATC command to RFThings RF210 board. These commands help controlling on-board sensors,
   GNSS & measuring battery level. For detail description, please visit: https://github.com/XuanMinh201/RF210

   @version 0.1.1
   @date 2023-10-26

   @copyright Copyright (c) 2023

*/

#define DATA_INTERVAL 5000 //ms

#include <Arduino.h>
#include <SensirionI2CScd4x.h> // Sensirion SCD40
#include "Zanshin_BME680.h" // https://github.com/Zanduino/BME680/tree/master
#include <Wire.h>
//#include "Adafruit_SHTC3.h"         // http://librarymanager/All#Adafruit_SHTC3
#include <Kionix_KX023.h>           // TO-DO: Add this original
//#include "Adafruit_LTR329_LTR303.h" // http://librarymanager/All#Adafruit_LTR329_LTR303
#include <LTR303.h> // https://github.com/automote/LTR303     // update lib with : value = (high << 8) + low;

#define boot_button PH3
#define LED PA0
#define PIR PB5
#define DATA_INTERVAL 5000 // ms
#define ADC_AREF 3.3f
#define BATVOLT_R1 1.0f
#define BATVOLT_R2 2.0f
#define BATVOLT_PIN PB4


//Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();
KX023 myIMU;
volatile bool flag;
LTR303 light;
SensirionI2CScd4x scd4x;
BME680_Class BME680;  ///< Create an instance of the BME680 class

uint16_t error;
int pirState = LOW;             // we start, assuming no motion detected
int val = 0;
unsigned char gain = 0;   // Gain setting, values = 0-7
unsigned char integrationTime = 0; // Integration ("shutter") time in milliseconds
uint16_t voltage_adc;
uint16_t voltage;
float kx_x, kx_y, kx_z;
//sensors_event_t hum, temp;
bool valid;
bool ltr_status;
unsigned int visible, infrared;
double lux;
unsigned long currentMillis = 0, getSensorDataPrevMillis = 0;
//bool sht_status;

uint16_t co2 = 0;
float temperature = 0.0f;
float humidity = 0.0f;
int it=0; // iteration for PIR
uint8_t PIR_array[100];
uint8_t Sound_array[100];


void printUint16Hex(uint16_t value) {
  Serial.print(value < 4096 ? "0" : "");
  Serial.print(value < 256 ? "0" : "");
  Serial.print(value < 16 ? "0" : "");
  Serial.print(value, HEX);
}

void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
  Serial.print("Serial: 0x");
  printUint16Hex(serial0);
  printUint16Hex(serial1);
  printUint16Hex(serial2);
  Serial.println();
}
void scd41()
{
  Wire.begin();

  uint16_t error;
  char errorMessage[256];

  scd4x.begin(Wire);


  // stop potentially previously started measurement
  error = scd4x.stopPeriodicMeasurement();
  if (error) {
    // Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    // Serial.println(errorMessage);
  }

  uint16_t serial0;
  uint16_t serial1;
  uint16_t serial2;
  error = scd4x.getSerialNumber(serial0, serial1, serial2);
  if (error) {
    // Serial.print("Error trying to execute getSerialNumber(): ");
    errorToString(error, errorMessage, 256);
    //Serial.println(errorMessage);
  } else {
    printSerialNumber(serial0, serial1, serial2);
  }

  // Start Measurement
  error = scd4x.startPeriodicMeasurement();
  if (error) {
    // Serial.print("Error trying to execute startPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    //Serial.println(errorMessage);
  }

  // Serial.println("Waiting for first measurement... (5 sec)");
}
void scd41_get() {
  
  char errorMessage[256];

  //  delay(100);

  // Read Measurement

  bool isDataReady = false;
  error = scd4x.getDataReadyFlag(isDataReady);
  if (error) {
    //Serial.print("Error trying to execute getDataReadyFlag(): ");
    errorToString(error, errorMessage, 256);
    // Serial.println(errorMessage);
    return;
  }
  if (!isDataReady) {
    return;
  }
  error = scd4x.readMeasurement(co2, temperature, humidity);
  if (error) {
    //Serial.print("Error trying to execute readMeasurement(): ");
    errorToString(error, errorMessage, 256);
    // Serial.println(errorMessage);
  } else if (co2 == 0) {
    // Serial.println("Invalid sample detected, skipping.");
  } else {
 
  }
}


int ATC_Ver(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    if (param->argc == 0) {
      Serial.print(cmd);
      Serial.print("=");

      Serial1.print(cmd);
      Serial1.print("=");
    }

    Serial.println("0.1.2");
    Serial1.println("0.1.2");
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

// bool getSHTstatus()
// {
//   if (!shtc3.begin())
//   {
//     return 0;
//   }
//   return 1;
// }


int scd41_co2(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {


    if (param->argc == 0) {
      Serial1.print(cmd);
      Serial1.print("=");
      Serial.print(cmd);
      Serial.print("=");
    }
    scd41_get();
    Serial.println(co2);
    Serial1.println(co2);


  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

int SCD_temp(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {


    if (param->argc == 0) {
      Serial1.print(cmd);
      Serial1.print("=");
      Serial.print(cmd);
      Serial.print("=");
    }
    scd41_get();
    Serial.println(temperature);
    Serial1.println(temperature);

  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

int SCD_humi(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {


    if (param->argc == 0) {
      Serial.print(cmd);
      Serial.print("=");
      Serial1.print(cmd);
      Serial1.print("=");
    }
    scd41_get();
    Serial.println(int(humidity));
    Serial1.println(int(humidity));
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

int bme_temp(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {


    if (param->argc == 0) {
      Serial1.print(cmd);
      Serial1.print("=");
      Serial.print(cmd);
      Serial.print("=");
    }

    static int32_t  temp, humidity, pressure, gas;  // BME readings
    BME680.getSensorData(temp, humidity, pressure, gas);  // Get readings
    Serial.println(temp / 100);
    Serial1.println(temp / 100);
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

int bme_hum(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {


    if (param->argc == 0) {
      Serial1.print(cmd);
      Serial1.print("=");
      Serial.print(cmd);
      Serial.print("=");
    }

    static int32_t  temp, humidity, pressure, gas;  // BME readings
    BME680.getSensorData(temp, humidity, pressure, gas);  // Get readings
    Serial.println(humidity / 1000);
    Serial1.println(humidity / 1000);
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

int bme_bar(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {


    if (param->argc == 0) {
      Serial1.print(cmd);
      Serial1.print("=");
      Serial.print(cmd);
      Serial.print("=");
    }

    static int32_t  temp, humidity, pressure, gas;  // BME readings
    BME680.getSensorData(temp, humidity, pressure, gas);  // Get readings
    Serial.println(pressure);
    Serial1.println(pressure);
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

int bme_gas(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {


    if (param->argc == 0) {
      Serial1.print(cmd);
      Serial1.print("=");
      Serial.print(cmd);
      Serial.print("=");
    }

    static int32_t  temp, humidity, pressure, gas;  // BME readings
    BME680.getSensorData(temp, humidity, pressure, gas);  // Get readings
    Serial.println(gas / 10);
    Serial1.println(gas / 10);
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}



bool kx023_status;

bool getKX023status()
{
  if (myIMU.begin())
  {
    return 0;
  }
  else
  {
      myIMU.configAsynchronousReadBackAccelerationData(KX023_ACCLERATION_RANGE_2G, KX023_ODR_25HZ);
      myIMU.setOperatingMode();
    return 1;
  }
}

int KX023_init(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
    Serial.println(kx023_status ? "1" : "0");
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

int KX023_AX(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    
      myIMU.configAsynchronousReadBackAccelerationData(KX023_ACCLERATION_RANGE_2G, KX023_ODR_25HZ);
      myIMU.setOperatingMode();
      myIMU.readAsynchronousReadBackAccelerationData(&kx_x, &kx_y, &kx_z);
      if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
      Serial.println(kx_x);
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

int KX023_AY(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
  
      myIMU.configAsynchronousReadBackAccelerationData(KX023_ACCLERATION_RANGE_2G, KX023_ODR_25HZ);
      myIMU.setOperatingMode();
      myIMU.readAsynchronousReadBackAccelerationData(&kx_x, &kx_y, &kx_z);
      if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
      Serial.println(kx_y);
    
  
    
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

int KX023_AZ(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
   
      myIMU.configAsynchronousReadBackAccelerationData(KX023_ACCLERATION_RANGE_2G, KX023_ODR_25HZ);
      myIMU.setOperatingMode();
      myIMU.readAsynchronousReadBackAccelerationData(&kx_x, &kx_y, &kx_z);
      if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
      Serial.println(kx_z);    
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}

int KX023_tap(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    Serial.println("KX023-1025 Activate Tap Function");
       DirectionInfoParams_t tap_direction;
  tap_direction.x_negative = false;
  tap_direction.x_positive = false;
  tap_direction.y_negative = false;
  tap_direction.y_positive = false;
  tap_direction.z_negative = true;
  tap_direction.z_positive = true;
  myIMU.configActivateTapFunction(KX023_ACCLERATION_RANGE_2G, KX023_DTODR_400HZ, tap_direction, KX023_TAP_MODE_BOTH);

  PhysicalInterruptParameter_t params;
  params.polarity = KX023_INTERRUPT_POLARITY_ACTIVE_HIGH;
  params.signal_type = KX023_INTERRUPT_TYPE_PULSE;
  params.events.tap_function_interrupt = true;
  myIMU.configPhysicalInterruptPin(1, params);
  flag = false;  

  myIMU.setOperatingMode();

      if (param->argc == 0){
     
    }
    else
    {
      if (param->argc == 0){
     Serial.print(cmd);
     Serial.print("=");}
      Serial.println("0");
    }
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}


int KX023_read(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
 Serial.println("KX023-1025 Activate Continuous Reading");

 myIMU.configSynchronousHardwareInterruptReadBackAccelerationData(KX023_ACCLERATION_RANGE_2G, KX023_ODR_6_25HZ);

  PhysicalInterruptParameter_t params;
  params.polarity = KX023_INTERRUPT_POLARITY_ACTIVE_HIGH;
  params.signal_type = KX023_INTERRUPT_TYPE_PULSE;
  params.events.data_ready_interrupt = true;
  myIMU.configPhysicalInterruptPin(1, params);
   myIMU.setOperatingMode();
  flag = false;
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}
bool getLTRstatus()
{
  unsigned char ID;

  if (light.getPartID(ID)) {
    light.setControl(gain, false, false);
    light.setMeasurementRate(1, 3);
    return 1;
  }
  // Most library commands will return true if communications was successful,
  // and false if there was a problem. You can ignore this returned value,
  // or check whether a command worked correctly and retrieve an error code:
  else {
    return 0;
  }
}

unsigned char getLTRID()
{
  unsigned char ID;

  if (light.getPartID(ID)) {
    return ID;
  }
  // Most library commands will return true if communications was successful,
  // and false if there was a problem. You can ignore this returned value,
  // or check whether a command worked correctly and retrieve an error code:
  else {
    return 0;
  }
}

int LTR_init(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    //light.setPowerUp();
    ltr_status = getLTRstatus();
    if (param->argc == 0) {
      Serial.print(cmd);
      Serial.print("=");
      Serial1.print(cmd);
      Serial1.print("=");
    }
    Serial.println(ltr_status ? "1" : "0");
    Serial.print("LTR Sensor Part ID: 0X");
    Serial.println(getLTRID(), HEX);

    Serial1.println(ltr_status ? "1" : "0");
    Serial1.print("LTR Sensor Part ID: 0X");
    Serial1.println(getLTRID(), HEX);

    //light.setPowerDown();
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

int LTR_ch0(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    //ltr_status = getLTRstatus();
    //light.setPowerUp();
    //delay(5000);
    if (light.getData(visible, infrared)) {
      if (param->argc == 0) {
        Serial.print(cmd);
        Serial.print("=");
        Serial1.print(cmd);
        Serial1.print("=");
      }
      Serial.println(visible);
      Serial1.println(visible);
      // light.setPowerDown();
    }
    else
    {
      if (param->argc == 0) {
        Serial.print(cmd);
        Serial.print("=");
        Serial1.print(cmd);
        Serial1.print("=");
      }
      Serial.println("0");
      Serial1.println("0");
    }
  }


  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

int LTR_ch1(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {

    //ltr_status = getLTRstatus();
    //light.setPowerUp();
    //delay(200);
    if (light.getData(visible, infrared))
    {
      if (param->argc == 0) {
        Serial.print(cmd);
        Serial.print("=");
        Serial1.print(cmd);
        Serial1.print("=");
      }
      Serial.println(infrared);
      Serial1.println(infrared);
      // light.setPowerDown();
    }
    else
    {
      if (param->argc == 0) {
        Serial.print(cmd);
        Serial.print("=");
        Serial1.print(cmd);
        Serial1.print("=");
      }
      Serial.println("0");
      Serial1.println("0");
    }
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

int LTR_lux(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    //ltr_status = getLTRstatus();
    // light.setPowerUp();
    //delay(5000);
    if (light.getData(visible, infrared))
    {
      if (param->argc == 0) {
        Serial.print(cmd);
        Serial.print("=");
        Serial1.print(cmd);
        Serial1.print("=");
      }

      valid = light.getLux(gain, integrationTime, visible, infrared, lux);
      Serial.println(lux);
      Serial1.println(lux);
      // light.setPowerDown();
    }

    else
    {
      if (param->argc == 0) {
        Serial.print(cmd);
        Serial.print("=");
        Serial1.print(cmd);
        Serial1.print("=");
      }
      Serial.println("0");
      Serial1.println("0");
    }
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}



int battery(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {

    voltage_adc = (uint16_t)analogRead(BATVOLT_PIN);
    voltage = (uint16_t)((ADC_AREF / 1.024) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)voltage_adc);
    if (param->argc == 0) {
      Serial.print(cmd);
      Serial.print("=");
      Serial1.print(cmd);
      Serial1.print("=");
    }
    Serial.println(voltage);
    Serial1.println(voltage);
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}

int ldo_read(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    if (param->argc == 0) {
      Serial.print(cmd);
      Serial.print("=");
      Serial1.print(cmd);
      Serial1.print("=");
    }
    Serial.println(api.system.bat.get() * 1000);
    Serial1.println(api.system.bat.get() * 1000);
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}
void PIR_statr() {
  val = digitalRead(PIR);  // read input value
  if (val == 1) {            // check if the input is HIGH
    digitalWrite(LED, HIGH);  // turn LED ON
    if (pirState == LOW) {
      // we have just turned on
     // Serial.println("1");
     // Serial1.println("1");
      // We only want to print on the output change, not state
      pirState = HIGH;
    }
  } else {
    digitalWrite(LED, LOW); // turn LED OFF
    if (pirState == HIGH) {
      // we have just turned of
     // Serial.println("0");
     // Serial1.println("0");
      // We only want to print on the output change, not state
      pirState = LOW;
    }
  }
}

int sound_meas() {  
  digitalWrite(PB2,HIGH);
  delay(50);
int sound =0;

for (int i=0;i<25;i++){
 sound = sound + digitalRead(PB3);
 delay(5);
  }
  digitalWrite(PB2,LOW);
  return sound;     
}

int sound(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    if (param->argc == 0) {
      Serial.print(cmd);
      Serial.print("=");
      Serial1.print(cmd);
      Serial1.print("=");
    }
    int snd=sound_meas();
    Serial.println(snd);
    Serial1.println(snd);
  }
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0) {
    return AT_OK;
  }
}


uint32_t led_status;
int led_handle(SERIAL_PORT port, char *cmd, stParam *param) {
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    Serial.println(led_status ? "HIGH" : "LOW");
  } else if (param->argc == 1) {
    for (int i = 0 ; i < strlen(param->argv[0]) ; i++) {
      if (!isdigit(*(param->argv[0] + i))) {
        return AT_PARAM_ERROR;
      }
    }
    led_status = strtoul(param->argv[0], NULL, 10);
    if (led_status != 0 && led_status != 1) {
      return AT_PARAM_ERROR;
    }
    digitalWrite(LED, (led_status == 1) ? HIGH : LOW);

  } else {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
}
bool PIR_STATUS = 1;
int PIR_ACTIVE(SERIAL_PORT port, char *cmd, stParam *param) {
  digitalWrite(LED, LOW);
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    Serial.println(PIR_STATUS ? "HIGH" : "LOW");
  } else if (param->argc == 1) {
    for (int i = 0 ; i < strlen(param->argv[0]) ; i++) {
      if (!isdigit(*(param->argv[0] + i))) {
        return AT_PARAM_ERROR;
      }
    }
    PIR_STATUS = strtoul(param->argv[0], NULL, 10);
    if (PIR_STATUS != 0 && PIR_STATUS != 1) {
      return AT_PARAM_ERROR;
    }

  } else {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
}

float altitude(const int32_t press, const float seaLevel = 1013.25);
float altitude(const int32_t press, const float seaLevel) {
  /*!
    @brief     This converts a pressure measurement into a height in meters
    @details   The corrected sea-level pressure can be passed into the function if it is known,
             otherwise the standard atmospheric pressure of 1013.25hPa is used (see
             https://en.wikipedia.org/wiki/Atmospheric_pressure) for details.
    @param[in] press    Pressure reading from BME680
    @param[in] seaLevel Sea-Level pressure in millibars
    @return    floating point altitude in meters.
  */
  static float Altitude;
  Altitude =
    44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903));  // Convert into meters
  return (Altitude);
}  // of method altitude()


int send(SERIAL_PORT port, char *cmd, stParam *param)
{
  if ((param->argc == 0) || (param->argc == 1 && (strcmp(param->argv[0], "?") == 0)))
  {
    if (param->argc == 0){
     Serial.println(cmd);
     }
     

    voltage_adc = (uint16_t)analogRead(BATVOLT_PIN);
    voltage = (uint16_t)((ADC_AREF / 1.024) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)voltage_adc);   
    int16_t b = voltage;
    Serial.print("Bat=");
    Serial.println(b);
    
    light.getData(visible, infrared);
    valid = light.getLux(gain,integrationTime,visible,infrared,lux);
     int16_t l = lux;
         Serial.print("Lum=");
    Serial.println(l);

 

    myIMU.readAsynchronousReadBackAccelerationData(&kx_x, &kx_y, &kx_z);
     int8_t x = 50*kx_x;
    int8_t y = 50*kx_y;
    int8_t z = 50*kx_z;
     Serial.print("Acc=");
    Serial.println(x);

   error = scd4x.readMeasurement(co2, temperature, humidity);
   int16_t t_scd=(int16_t) 100*temperature; // return temperature in tens of degree
    uint8_t h_scd=(uint8_t)2*humidity; // return humidity in percent
    int16_t CO2_scd=(int16_t) co2; // return temperature in tens of degree
         Serial.print("SCDtemp=");
    Serial.println(t_scd);
          Serial.print("SCDhum=");
    Serial.println(h_scd);    
       Serial.print("SCDCO2=");
    Serial.println(CO2_scd);


    static int32_t  temp_bme, hum_bme, press_bme, gas_bme;  // BME readings
    BME680.getSensorData(temp_bme, hum_bme, press_bme, gas_bme);  // Get readings
    int16_t t_bme=(int16_t) temp_bme; // return temperature in tens of degree
    uint8_t h_bme=(uint8_t)(hum_bme/500); // return humidity in percent
    int32_t p_bme=(int32_t) press_bme; // return temperature in tens of degree
    int16_t g_bme=(int16_t) gas_bme; // return temperature in tens of degree

     Serial.print("BMEtemp=");
    Serial.println(t_bme);
      Serial.print("BMEhum=");
    Serial.println(h_bme);
    Serial.print("BMEpress=");
    Serial.println(p_bme);
     Serial.print("BMEgas=");
    Serial.println(g_bme);

    int pir_tot=0;
    for(int i=0; i<100;i++){
      pir_tot=pir_tot+PIR_array[i];
    }
    Serial.print("PIR=");
    Serial.println(pir_tot);

    int sound_tot=0;
    for(int i=0; i<100;i++){
      sound_tot=sound_tot+Sound_array[i];
    }
    Serial.print("Sound=");
    Serial.println(sound_tot);
   

    int i=0;
    unsigned char mydata[64];

   
    mydata[i++] = t_scd >> 8;
    mydata[i++] = t_scd & 0xFF;
    mydata[i++] = h_scd;
    mydata[i++] = CO2_scd >> 8;
    mydata[i++] = CO2_scd & 0xFF;
    mydata[i++] = x;    
    mydata[i++] = y;    
    mydata[i++] = z;   
    mydata[i++] = l >> 8;
    mydata[i++] = l & 0xFF;
    mydata[i++] = b >> 8;
    mydata[i++] = b & 0xFF;
    mydata[i++] = t_bme >> 8;
    mydata[i++] = t_bme & 0xFF;
    mydata[i++] = h_bme;
    mydata[i++] = p_bme >> 32;
    mydata[i++] = p_bme >> 16;
    mydata[i++] = p_bme >> 8;
    mydata[i++] = p_bme & 0xFF;
    mydata[i++] = g_bme >> 8;
    mydata[i++] = g_bme & 0xFF;
    mydata[i++] = pir_tot;
    mydata[i++] = sound_tot >> 8;
    mydata[i++] = sound_tot & 0xFF;
        
    api.lorawan.send(i, mydata, 5);
     Serial.println("Send LoRa");
    
  }
  
  else
  {
    return AT_PARAM_ERROR;
  }
  if (param->argc == 0){
  return AT_OK;}
}


void setup()
{
  Serial1.begin(115200, RAK_AT_MODE);
  Serial.begin(115200);  
  pinMode(PIR, INPUT);
  pinMode(PB3, INPUT);
  pinMode(boot_button, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  pinMode(PB2, OUTPUT);

  scd41();

  light.begin();
  light.setPowerUp();
 

  while (!BME680.begin(I2C_STANDARD_MODE)) {  // Start BME680 using I2C, use first device found
    Serial.print(F("-  Unable to find BME680. Trying again in 5 seconds.\n"));
    delay(5000);
  }  // of loop until device is located
  BME680.setOversampling(TemperatureSensor, Oversample16);  // Use enumerated type values
  BME680.setOversampling(HumiditySensor, Oversample16);     // Use enumerated type values
  BME680.setOversampling(PressureSensor, Oversample16);     // Use enumerated type values
  BME680.setIIRFilter(IIR4);  // Use enumerated type values
  BME680.setGas(320, 150);  // 320�c for 150 milliseconds

  KX023_Status_t status = myIMU.begin();
  myIMU.configAsynchronousReadBackAccelerationData(KX023_ACCLERATION_RANGE_2G, KX023_ODR_25HZ);
  myIMU.setOperatingMode();


  api.system.atMode.add("VER", "Return firmware version", "VER", ATC_Ver, RAK_ATCMD_PERM_READ);

  api.system.atMode.add("LED", "1 - LED Active // 0 - LED Inactive", "LED", led_handle, RAK_ATCMD_PERM_WRITE | RAK_ATCMD_PERM_READ);
  api.system.atMode.add("PIRACT", "1 - PIR Active // 0 - PIR Inactive", "PIRACT", PIR_ACTIVE, RAK_ATCMD_PERM_WRITE | RAK_ATCMD_PERM_READ);

  api.system.atMode.add("SCDCO2", "Return the value CO2 of SCD41.", "SCDCO2", scd41_co2, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("SCDTEMP", "Return the temperature value with 0.01° resolution", "SCDTEMP", SCD_temp, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("SCDHUM", "Return the humidity value with 1% resolution", "SCDHUM", SCD_humi, RAK_ATCMD_PERM_READ);

  api.system.atMode.add("BMEGAS", "Return the gas resistivity in ohmable.", "BMEGAS", bme_gas, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("BMETEMP", "Return the temperature value with 0.01° resolution", "BMETEMP", bme_temp, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("BMEHUM", "Return the humidity value with 1% resolution", "BMEHUM", bme_hum, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("BMEBAR", "Return the pressure value in mbar ", "BMEHUM", bme_bar, RAK_ATCMD_PERM_READ);

  api.system.atMode.add("TEMP", "Return the temperature value with 0.01° resolution", "TEMP", SCD_temp, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("HUM", "Return the humidity value with 1% resolution", "HUM", SCD_humi, RAK_ATCMD_PERM_READ);

  api.system.atMode.add("KX023", "Return the status of the KX023 sensor. 1 if available.", "KX023", KX023_init, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("AX", "Return the value of X acceleration with 0.01G resolution", "AX", KX023_AX, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("AY", "Return the value of Y acceleration with 0.01G resolution", "AY", KX023_AY, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("AZ", "Return the value of Z acceleration with 0.01G resolution", "AZ", KX023_AZ, RAK_ATCMD_PERM_READ);

  api.system.atMode.add("LTR", "Return the status of the LTR-303 sensor. 1 if available.", "LTR", LTR_init, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("LUMCH0", "Return the CHANNEL0 value of the LTR-303 sensor", "LUMCH0", LTR_ch0, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("LUMCH1", "Return the CHANNEL1 value of the LTR-303 sensor", "LUMCH1", LTR_ch1, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("LUM", "Return the CHANNEL1 value of the LTR-303 sensor", "LUM", LTR_lux, RAK_ATCMD_PERM_READ);

  api.system.atMode.add("SOUND", "Return the Sound value averaged over 1s", "SOUND", sound, RAK_ATCMD_PERM_READ);

  api.system.atMode.add("SENSOR", "Send LoRaWan packet with sensor", "SENSOR", send, RAK_ATCMD_PERM_READ);


  api.system.atMode.add("BAT", "Return battery voltage in mV | Return 0 if not available", "BAT", battery, RAK_ATCMD_PERM_READ);
  api.system.atMode.add("LDO", "Return LDO voltage in mV | Return 0 if not available", "LDO", ldo_read, RAK_ATCMD_PERM_READ);

   digitalWrite(LED, HIGH);  // turn LED ON
delay(1000);
    digitalWrite(LED, LOW);  // turn LED ON

}

void loop()
{
  if (PIR_STATUS)
  {
    PIR_statr();
  }
  currentMillis = millis();  
      
      if (currentMillis - getSensorDataPrevMillis > DATA_INTERVAL){
        getSensorDataPrevMillis = currentMillis;
        PIR_array[it] = digitalRead(PIR);
        Sound_array[it] = sound_meas();
        it++;
        if(it==100){it=0;}
  }
  //api.system.sleep.all(500);


}
