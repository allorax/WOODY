#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>

#define DHTPIN 2
#define DHTTYPE DHT22

Adafruit_BMP280 bmp; 
DHT dht(DHTPIN, DHTTYPE);

const char* WIFI_SSID = "i=q/t#Current";
const char* WIFI_PASS = "2022#ElectronFlow";

const char* MQTT_SERVER = "192.168.0.192"; 

const int MQTT_PORT = 1883;
const char* MQTT_TOPIC = "sensor/data";

const unsigned long PUBLISH_INTERVAL = 900000;


WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastPublish = 0;


void setupWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}


void mqttReconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP8266Client")) {
      Serial.println("connected!");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 2 seconds");
      delay(2000);
    }
  }
}


void publishData() {
  // float temp = bmp.readTemperature();
  float temp_c = dht.readTemperature();
  float temp_f = dht.readTemperature(true);
  float humidity = dht.readHumidity();
  float hif = dht.computeHeatIndex(temp_f, humidity);
  float hic = dht.computeHeatIndex(temp_c, humidity, false);
  float pressure = bmp.readPressure() / 100.0F;
  
  if (isnan(humidity) || isnan(temp_c) || isnan(temp_f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  
  StaticJsonDocument<180> doc;

  // Calculate dew point
  // float dewPoint = temp_c - ((100 - humidity) / 5.0f);

  // Magnus formula to calculate dew point
  float a = 17.62;
  float b = 243.5;
  float gamma = (a * temp_c / (b + temp_c)) + log(humidity / 100.0);
  float dewPoint = (b * gamma) / (a - gamma);

  doc["temperature_c"] = temp_c;
  doc["temperature_f"] = temp_f;
  doc["humidity"] = humidity;
  doc["heatIndex_c"] = hic;
  doc["heatIndex_f"] = hif;
  doc["pressure"] = pressure;
  doc["dewPoint_c"] = dewPoint;
  
  char buffer[180];
  serializeJson(doc, buffer);

  client.publish(MQTT_TOPIC, buffer);
  Serial.print("Published: ");
  Serial.println(buffer);
}

void setup() {
  Serial.begin(115200);
  setupWiFi();

  if (!bmp.begin(0x76)) {  
    Serial.println("Could not find BMP280 sensor at 0x76!");
    while (1);  
  } 
  else {
    Serial.println("BMP280 initialized!");
  
  }

  dht.begin();

  client.setServer(MQTT_SERVER, MQTT_PORT);
  delay(2000);
  publishData();
}


void loop() {
  if (!client.connected()) {
    mqttReconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastPublish >= PUBLISH_INTERVAL) {
    lastPublish = now;
    publishData();
  }
}
