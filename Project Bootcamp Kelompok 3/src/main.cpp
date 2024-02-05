#include <Arduino.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <LittleFS.h>
#include <SD.h>
#include <WiFi.h>
#include <Update.h>
#include <PubSubClient.h>
#include <Wire.h>

const char* wifiSsid = "Wokwi-GUEST";
const char* wifiPassword = "";
const char* mqttServer = "broker.emqx.io";
const int mqttPort = 1883;

const char* temperatureTopic = "ESP32/temperature";
const char* humidityTopic = "ESP32/humidity";
const char* statusTopic = "ESP32/status";
const char* controlTopic = "ESP32/control";
const char* waterTopic = "ESP32/waterTemp";
const char* phTopic = "ESP32/ph";
const char* doTopic = "ESP32/do";
const char* waterLevelTopic = "ESP32/waterLevel";

const unsigned long interval = 1000;
bool activeStatus = false;
unsigned long previousMillis = 0;

#define DHT_SENSOR_PIN 21 
#define DHT_SENSOR_TYPE DHT22

#define TRIG_PIN 20 
#define ECHO_PIN 19 

#define TEMP_PIN 1 
#define PH_PIN 4
#define DO_PIN 5
#define SERVO_ADD_WATER_PIN 14
#define SERVO_ADD_PHUP_PIN 16
#define SERVO_ADD_PHDOWN_PIN 17
#define ledPin 13
LiquidCrystal_I2C lcd(0x27,20,4);
Servo servoAddWater;
Servo servoAddpHUp;
Servo servoAddpHDown;

WiFiClient espClient;
PubSubClient client(espClient);
float duration_us, distance_cm;
DHT dhtSensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

OneWire oneWire(TEMP_PIN);
DallasTemperature DS18B20(&oneWire);
float watertempC;
float Ph;
float tempUpp = 32;
float tempLow = 27;
float LowestpH = 5.5;
float HighestpH = 6.5;

float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void showStatus(bool status) {
    String statusString;
    if (status) {
        digitalWrite(ledPin, HIGH);
        statusString = "ON";
    } else {
        digitalWrite(ledPin, LOW);
        statusString = "OFF";
    }
    Serial.printf("Status: %s\r\n", statusString.c_str());

    char statusMessage[5];
    snprintf(statusMessage, 5, "%s", statusString.c_str());
    client.publish(statusTopic, statusMessage);

}

void mqttcallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived: ");
  Serial.println(topic);

  char messageChar = (char)payload[0];
  if (messageChar == '1') {
    activeStatus = true;
  } else if (messageChar == '0') {
    activeStatus = false;
  }
  showStatus(activeStatus);
}

void mqttConnect() {
    while (!client.connected()) {
        Serial.print("MQTT connecting ... ");
        char clientId[20];
        sprintf(clientId, "ESP32Client-%ld", random(1000));
        if (client.connect(clientId)) {
            Serial.print("Connected with id: ");
            Serial.println(clientId);
            client.subscribe(controlTopic);
        } else {
            Serial.print("failed, state: ");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}
void wifiConnect() {
  Serial.println("Connecting to WiFi");
  WiFi.begin(wifiSsid, wifiPassword);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to the WiFi network");
}


void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  dhtSensor.begin(); 
  wifiConnect();
  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqttcallback);
  Wire.begin(15,2);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  lcd.init();
  lcd.backlight();
  DS18B20.begin();    
  servoAddWater.attach(SERVO_ADD_WATER_PIN);
  servoAddWater.write(0);
  servoAddpHUp.attach(SERVO_ADD_PHUP_PIN);
  servoAddpHUp.write(0);
  servoAddpHDown.attach(SERVO_ADD_PHDOWN_PIN);
  servoAddpHDown.write(0);
}

void loop() {
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration_us = pulseIn(ECHO_PIN, HIGH);

  distance_cm = 0.017 * duration_us;

  Serial.print("distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  delay(500);


  float humi  = dhtSensor.readHumidity();
  float tempC = dhtSensor.readTemperature();
  float tempF = dhtSensor.readTemperature(true);

  if ( isnan(tempC) || isnan(tempF) || isnan(humi)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {
    Serial.print("Humidity: ");
    Serial.print(humi);
    Serial.print("%");

    Serial.print("  |  ");

    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.print("°C  ~  ");
    Serial.print(tempF);
    Serial.println("°F");
  }

  DS18B20.requestTemperatures();       
  watertempC = DS18B20.getTempCByIndex(0);  
  Serial.print("Water Temperature: ");
  Serial.print(watertempC);
  Serial.println("°C");

  int analogPh = analogRead(PH_PIN);

  Ph = floatMap (analogPh, 0, 4095, 0, 14);

  lcd.clear();

  lcd.setCursor(0,0);
  lcd.print("water t:");
  lcd.print(watertempC,2); 
  lcd.print((char)223);
  lcd.print("C");

  lcd.setCursor(0,1);
  lcd.print("pH:");
  lcd.print(Ph);

  lcd.setCursor(0,2);
  lcd.print("Hum:");
  lcd.print(humi);
  lcd.print("%");

  lcd.setCursor(0,3);
  lcd.print("t:");
  lcd.print(tempC);
  lcd.print((char)223);
  lcd.print("C");

  if(watertempC>tempUpp){
    servoAddWater.write(90);
  } else if(watertempC<tempLow){
    servoAddWater.write(0);
  }
  if(Ph<LowestpH){
    servoAddpHUp.write(90);
    delay(1000);
    servoAddpHUp.write(0);
    servoAddWater.write(90);
    delay(3000);
    servoAddWater.write(0);
  } else if(Ph>HighestpH){
    servoAddpHDown.write(90);
    delay(1000);
    servoAddpHDown.write(0);
    servoAddWater.write(90);
    delay(3000);
    servoAddWater.write(0);
  }
  
  if (!client.connected()) {
    mqttConnect();
  }
  client.loop();

  unsigned long currentTime = millis();

  bool shouldPublish = (currentTime - previousMillis) > interval;
  if (shouldPublish && activeStatus) {
    previousMillis = currentTime;
    
    char statusHum[6];
    char statusTemp[6];
    char statusWater[6];
    char statusPh[6];
    char statusWaterLevel[6];

    snprintf(statusHum, 6, "%.2f", humi);
    snprintf(statusTemp, 6, "%.2f", tempC);
    snprintf(statusWater, 6, "%.2f", DS18B20.getTempCByIndex(0));
    snprintf(statusPh, 6, "%.2f", Ph);
    snprintf(statusWaterLevel, 6, "%.2f", distance_cm);
    client.publish(humidityTopic, statusHum);
    client.publish(temperatureTopic, statusTemp);
    client.publish(waterTopic, statusWater);
    client.publish(phTopic, statusPh);
    client.publish(waterLevelTopic, statusWaterLevel);
  }

    delay(500);
  }
