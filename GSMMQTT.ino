#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

#define SerialMon Serial
#define MODEM_RX 32
#define MODEM_TX 33
HardwareSerial sim800(1);

#define TINY_GSM_DEBUG SerialMon
#define DUMP_AT_COMMANDS

const char apn[] = "internet"; // Ganti sesuai provider SIM
const char gprsUser[] = "";
const char gprsPass[] = "";
const char* broker = "test.mosquitto.org";
const int port = 1883;
const char* topic = "stick/stick_3/data";
const char* stick_id = "stick_3";

TinyGsm modem(sim800);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

#define echoPin 27
#define trigPin 26
#define buzzerPin 25
#define waterSensorPin 34
#define emergencyButtonPin 33

long duration, distance;
unsigned long lastSendTime = 0;
const unsigned long readInterval = 2000;
int waterValue = 0;
double lat = 0.0, lng = 0.0;
bool emergencySent = false;

TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // RX=16, TX=17 

void mqttReconnect() {
  while (!mqtt.connected()) {
    SerialMon.print("Menghubungkan ke MQTT...");
    if (mqtt.connect("ESP32Stick3Client")) {
      SerialMon.println("Berhasil!");
    } else {
      SerialMon.print("Gagal: ");
      SerialMon.println(mqtt.state());
      delay(2000);
    }
  }
}

void setup() {
  SerialMon.begin(115200);
  delay(10);
  
  sim800.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);  // RX=32, TX=33
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(emergencyButtonPin, INPUT_PULLUP);

  SerialMon.println("Memulai SIM800...");
  modem.restart();
  delay(3000);

  SerialMon.print("Menghubungkan GPRS...");
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println("Gagal konek GPRS!");
    while (true);
  }
  SerialMon.println("Terhubung!");

  mqtt.setServer(broker, port);
}

void loop() {
  if (!mqtt.connected()) mqttReconnect();
  mqtt.loop();

  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
    if (gps.location.isUpdated()) {
      lat = gps.location.lat();
      lng = gps.location.lng();
    }
  }

  waterValue = analogRead(waterSensorPin);

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration / 58.2;

  if (digitalRead(emergencyButtonPin) == LOW && !emergencySent) {
    emergencySent = true;
    String emergencyPayload = "{";
    emergencyPayload += "\"id\":\"" + String(stick_id) + "\",";
    emergencyPayload += "\"emergency\":1,";
    emergencyPayload += "\"message\":\"Tombol darurat ditekan\"";
    emergencyPayload += "}";
    mqtt.publish(topic, emergencyPayload.c_str());
    SerialMon.println("⚠️ Emergency sent: " + emergencyPayload);
  }

  if (digitalRead(emergencyButtonPin) == HIGH) {
    emergencySent = false;
  }

  if (millis() - lastSendTime >= readInterval) {
    lastSendTime = millis();
    String payload = "{";
    payload += "\"id\":\"" + String(stick_id) + "\",";
    payload += "\"distance\":" + String(distance) + ",";
    payload += "\"water_level\":" + String(waterValue) + ",";
    payload += "\"longitude\":" + String(lng, 6) + ",";
    payload += "\"latitude\":" + String(lat, 6) + ",";
    payload += "\"emergency\":0}";
    mqtt.publish(topic, payload.c_str());
    SerialMon.println("MQTT Sent: " + payload);
  }
}
