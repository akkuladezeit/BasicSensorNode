////Globale Config
//#define DO_DEBUG
#define BATTERY_USE
//sensors
//#define BME680      
//#define BMP180
#define SHT30

//nodeconfig
String PubPath  = "home/sensors";
String SensorNr = "sensor1";
#define DEEPSLEEP_SECONDS 120

////Headers
//Basic
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <Wire.h>
#include <Ticker.h>
//Sensor
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include "Adafruit_BME680.h"
//mqtt
#include <PubSubClient.h>


//Dirty Public WIFI config
WiFiClient client;
const char* ssid = "Hier-Druecken-fuer-WLAN";
const char* password = "1234567890";
const char* wificlienthostname = SensorNr.c_str();
IPAddress ip(192,168,178,99);
IPAddress dns(192,168,178,1);
IPAddress gateway(192,168,178,1);   
IPAddress subnet(255,255,255,0);
String ipAddress="000.000.000.000";

//Dirty Public Mqtt config
PubSubClient mqttclient(client);
const char* mqtt_server = "192,168,178,1";
const int mqtt_port = 1883;
const char * mqtt_node_name = SensorNr.c_str();

//Dirty Public Batt Data
#if defined(BATTERY_USE)
String lipoVoltage = "0";
#endif

//Dirty Public Sensor Data
#if defined(BME680)
//angepassste lib!!
Adafruit_BME680 bme680;
#define SEALEVELPRESSURE_HPA 1013.25 // Luftdruck auf Meereshoehe
String bme680temperature;
String bme680pressure;
String bme680humidity;
String bme680gas;
String bme680altitude;
#endif

#if defined(BMP180)
Adafruit_BMP085 bmp180;
#define BMP180_SEALEVELPRESSURE 101325 // Luftdruck auf Meereshoehe
String bmp180temperature;
String bmp180pressure;
String bmp180altitude;
#endif

#if defined(SHT30)
uint8_t sht30address = 0x44;
String sht30temperature;
String sht30humidity;
#endif

void mqtt_reconnect() 
{
  // Loop until we're reconnected
  while (!mqttclient.connected()) 
  {
    #if defined(DO_DEBUG)
    Serial.print("Attempting MQTT connection...");
    #endif
    
    // Attempt to connect
    if (mqttclient.connect(mqtt_node_name)) 
    {  
      #if defined(DO_DEBUG)
      // Once connected, publish an announcement...
      Serial.println("connected");
      mqttclient.publish((PubPath + "/" + SensorNr + "/" + "nodes").c_str(), ("Started:" + SensorNr  + ipAddress).c_str());
      #endif
    } 
    else
    {
      #if defined(DO_DEBUG)

      Serial.print("failed, rc=");
      Serial.print(mqttclient.state());
      Serial.println(" try again in 1 seconds");
      #endif
      
      // Wait 1 seconds before retrying
      delay(1000);
    }
  }
}

void mqtt_publishValue(String topic, String value)
{
  #if defined(DO_DEBUG)  
  Serial.println("Publish Topic: " + String(topic));
  Serial.println("Publish Value: " + String(value));
  #endif
  
  mqttclient.publish(topic.c_str(),value.c_str());
}

//Basic System setup 
// - wifi
// - Mqtt
// - sensor init if needed
void setup(void)
{
  #if defined(DO_DEBUG)  
  Serial.begin(115200);
  #endif
  
  //Config and start WLAN frist
  WiFi.hostname(wificlienthostname);
  WiFi.config(ip, dns, gateway, subnet);
  WiFi.mode(WIFI_STA);
  #if defined(DO_DEBUG)  
  Serial.println("Wifi MAC: " + WiFi.macAddress());
  #endif
  WiFi.begin(ssid, password);delay(1000);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(100);
  }
  ipAddress=WiFi.localIP().toString();
  #if defined(DO_DEBUG)  
  Serial.println("Wifi Connection established: " + String(ipAddress));
  #endif

  //Mqtt init
  mqttclient.setServer(mqtt_server,  mqtt_port);
    
  //Init Sensors
  #if defined(BME680)
  if(!bme680.begin())
  {
    #if defined(DO_DEBUG)  
    Serial.print("Ooops, no BME680 detected ... Check your wiring or I2C ADDR!");
    #endif
    while(1);
  }
  
  // Initialisierung von  Oversampling und Filter
  bme680.setTemperatureOversampling(BME680_OS_8X);
  bme680.setHumidityOversampling(BME680_OS_2X);
  bme680.setPressureOversampling(BME680_OS_4X);
  bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme680.setGasHeater(320, 150); // 320*C for 150 ms
  #endif
  
  #if defined(BMP180)
  if(!bmp180.begin())
  {
    #if defined(DO_DEBUG)   
    Serial.print("Ooops, no BMP085 detected ... ChSensorNreck your wiring or I2C ADDR!");
    #endif
    while(1);
  }
  #endif
}

#if defined(BATTERY_USE)
void pwr_watch(void)
{
  //Wemos d1 mini pro measure Vbat add 130k between Vbat and ADC
  //Voltage divider of 130k+220k over 100k (100/450k  Serial.println(String(data[0]));)
  //-> 4.5V -> 1Voltsht30_read
  //Max input on A0=1Volt->1023
  //4.5*(Raw/1023)=Vbat
  
  unsigned int raw=0;
  float volt=0.0;
  
  pinMode(A0, INPUT);
  raw = analogRead(A0);
  volt=raw/1023.0;
  lipoVoltage=volt*4.5;
}
#endif

#if defined(SHT30)
void sht30_read(void)
{
  float cTemp = 0;
  float fTemp = 0;
  float humidity = 0;  
  unsigned int data[6] = {0};
  
  Wire.begin();
  // Start I2C Transmission 
  Wire.beginTransmission(sht30address);
  // Send measurement command 
  Wire.write(0x2C);
  Wire.write(0x06);
  // Stop I2C transmission 
  if (Wire.endTransmission() != 0);

  delay(100);
  // Request 6 bytes of data Adafruit_SHT31
  Wire.requestFrom(sht30address, 6);
  // Read 6 bytes of data 
  // cTemp msb, cTemp lsb, cTemp crc, humidity msb, humidity lsb, humidity crc 
  for (int i = 0; i < 6; i++) {
    data[i] = Wire.read(); 
  };
  delay(10); 
  if (Wire.available() != 0); //currently no error handling 
   // Convert the data 
  cTemp = ((((data[0] * 256.0) + data[1]) * 175) / 65535.0) - 45;
  fTemp = (cTemp * 1.8) + 32;
  humidity = ((((data[3] * 256.0) + data[4]) * 100) / 65535.0);
  sht30temperature = cTemp;
  sht30humidity = humidity;
}
#endif

#if defined(BME680)
void bme680_read(void)
{
  //delay to ensure measurement result is present.
  if(!bme680.performReading()) 
  {
    #if defined(DO_DEBUG) 
    Serial.println("Error reading from BME680");
    #endif
    return;
  }
  
  // Werte ermitteln:
  bme680temperature   = bme680.temperature;
  bme680pressure      = bme680.pressure / 100.0;
  bme680humidity      = bme680.humidity;
  bme680gas           = bme680.gas_resistance / 1000.0;
  bme680altitude      = bme680.readAltitude(SEALEVELPRESSURE_HPA);
}
#endif

#if defined(BMP180)
void bmp180_read(void)
{
  //float temperature;
  //bmp180.readTemperature(&temperature);
  bmp180temperature=bmp180.readTemperature();
  
  //int32_t pressure;
  //bmp180.readPressure(&pressure);
  bmp180pressure=bmp180.readPressure();

  //float altitude;
  bmp180altitude=bmp180.readAltitude(BMP180_SEALEVELPRESSURE);
}
#endif


//performe measuremnt of all sensors
void measurement(void)
{
  #if defined(BME680)
  bme680_read();
  #endif
  
  #if defined(BMP180)
  bmp180_read();
  #endif

  #if defined(SHT30)
  sht30_read();
  #endif
}

void publishresult(void)
{
  #if defined(BME680)
  mqtt_publishValue( PubPath + "/" + SensorNr + "/" + "bme680_temperature",  bme680temperature);
  delay(50);
  mqtt_publishValue( PubPath + "/" + SensorNr + "/" + "bme680_pressure",     bme680pressure);
  delay(50);  
  mqtt_publishValue( PubPath + "/" + SensorNr + "/" + "bme680_humidity",     bme680humidity);
  delay(50);
  mqtt_publishValue( PubPath + "/" + SensorNr + "/" + "bme680_gas",          bme680gas);
  delay(50);  
  mqtt_publishValue( PubPath + "/" + SensorNr + "/" + "bme680_altitude",     bme680altitude);
  delay(50);
  #endif
  
  #if defined(BMP180)
  mqtt_publishValue( PubPath + "/" + SensorNr + "/" + "bmp180_temperature",  bmp180temperature);
  delay(50);
  mqtt_publishValue( PubPath + "/" + SensorNr + "/" + "bmp180_pressure",     bmp180pressure);
  delay(50);
  mqtt_publishValue( PubPath + "/" + SensorNr + "/" + "bmp180_altitude",     bmp180altitude);
  delay(50);
  #endif

  #if defined(SHT30)
  mqtt_publishValue( PubPath + "/" + SensorNr + "/" + "sht30_temperature",   sht30temperature);
  delay(50);
  mqtt_publishValue( PubPath + "/" + SensorNr + "/" + "sht30_humidity",      sht30humidity);
  delay(50);
  #endif

  #if defined(BATTERY_USE)
  mqtt_publishValue( PubPath + "/" + SensorNr + "/" + "batt",     lipoVoltage);
  delay(50);
  #endif
}


void loop()
{
  //performe measurement of configured sensors
  measurement();      
  
  #if defined(BATTERY_USE)
  pwr_watch();
  #endif

  if (!mqttclient.connected()) 
  {
    mqtt_reconnect();
  }
  delay(100);

  //handle mqtt
  mqttclient.loop();

  //publish measurement results
  publishresult();

  //save the planet go to sleep
  ESP.deepSleep(DEEPSLEEP_SECONDS * 1000000);
}
