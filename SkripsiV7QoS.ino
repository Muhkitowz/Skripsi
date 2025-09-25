// ========== Library ==========
#include <Wire.h>
#include <Adafruit_INA219.h>

#include <WiFi.h>
#include <ArduinoMqttClient.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// ========== PIN DEFINISI ==========
#define echoPin 27
#define trigPin 26
#define buzzerPin 25
#define waterSensorPin 34
#define emergencyButtonPin 33

// GPS (HardwareSerial 2)
HardwareSerial serialGPS(2);  // RX=16, TX=17
TinyGPSPlus gps;

// INA219 (I2C pada SDA=21, SCL=22)
Adafruit_INA219 ina219;

// ========== MQTT & TOPIC ==========
const char* stick_id = "0001";
const char* topic_data = "stick/0001/data";
const char* topic_emergency = "stick/0001/emergency";
const char* topic_callback = "stick/0001/callback";
const char* willTopic = "stick/0001/status";
const char* topic_battery = "stick/0001/battery";
const char* topic_ampere = "stick/0001/ampere";

const char* mqtt_server = "54.169.178.246";
const int mqtt_port = 1883;
const char* mqtt_user = "iotuser";
const char* mqtt_pass = "Password123#";

// Koneksi WiFi
const char* ssid = "Afz";
const char* password = "totskuy1";

// ========== Variabel Global ==========
bool emergencySent = false;

long duration_us, distance_cm;
unsigned long lastUltrasonicRead = 0;
unsigned long lastWaterRead = 0;
unsigned long lastSendTime = 0;
const unsigned long readInterval = 2000;

bool ultrasonicActive = false;
bool waterActive = false;

int beepDelay = 1000;
int waterBeepDelay = 1000;
int waterValue = 0;

double lat = 0.0, lng = 0.0;

float batteryVoltage = 0;
int batteryPercent = 0;
int lastBatteryPercent = -1;

unsigned long buttonPressTime = 0;
bool emergencyLatched = false;
const unsigned long EMERGENCY_HOLD_MS = 2500;

// ========== WiFi Client ==========
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient); // menggantikan PubSubClient client
// Jika ada referensi ke objek bernama "client" sebelumnya, diperbarui ke "mqttClient".

// ========== Helper ==========
int getBeepDelay(int distance) {
  int minDelay = 50;
  int maxDelay = 1000;
  int maxDistance = 125;
  int delayVal = map(distance, 0, maxDistance, minDelay, maxDelay);
  return constrain(delayVal, minDelay, maxDelay);
}

// Nada callback (lebih tinggi)
void playMelodyJemput() {
  int melody[] = { 2200, 2400, 2600 };  // Nada tinggi naik
  int noteDuration = 300;
  for (int i = 0; i < 3; i++) {
    tone(buzzerPin, melody[i], noteDuration);
    delay(noteDuration + 100 );
    noTone(buzzerPin);
    delay(100);
  }
}
void playMelodyTolak() {
  int melody[] = { 1000, 700, 500 };  // Nada tinggi turun
  int noteDuration = 400;
  for (int i = 0; i < 3; i++) {
    tone(buzzerPin, melody[i]);
    delay(noteDuration + 100);
    noTone(buzzerPin);
    delay(100);
  }
}

// ========== MQTT Callback ==========
void onMqttMessage(int messageSize) {
  // Ambil topik dan isi pesan dari mqttClient (Stream interface)
  Serial.print("Pesan masuk di topic: ");
  Serial.println(mqttClient.messageTopic());
  Serial.print("Payload: ");
  String msg = "";
  while (mqttClient.available()) {
    msg += (char)mqttClient.read();
  }
  Serial.println(msg);

  // logic lama:
  if (msg == "jemput") playMelodyJemput();
  else if (msg == "tolak") playMelodyTolak();
}


// ========== WiFi Connect ==========
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Menghubungkan ke WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi terhubung, IP: " + WiFi.localIP().toString());
}

// ========== MQTT Connect ==========
void connectMQTT() {
  
  // Konfigurasi ID & kredensial untuk ArduinoMqttClient
  mqttClient.setId("ESP32ClientStick_WIFI");
  mqttClient.setUsernamePassword(mqtt_user, mqtt_pass);
  // Jika ingin menggunakan Last Will & Testament, uncomment baris berikut (jika library mendukung)
  // mqttClient.setWill(willTopic, "offline", true, 0);

  Serial.print("Menghubungkan ke MQTT broker...");
  if (!mqttClient.connect(mqtt_server, mqtt_port)) {
    Serial.print("Gagal! Error: ");
    Serial.println(mqttClient.connectError());
    // jangan loop tak berujung di sini, kembalikan ke caller untuk retry
    return;
  }
  Serial.println("Berhasil.");
  // Publish status online (retained) agar subscriber tahu kita online
  mqttClient.beginMessage(willTopic, (unsigned long)6, true, 1); // payload size 6 = len("online"), retained=true
  mqttClient.print("online");
  mqttClient.endMessage();

  // Set handler pesan dan subscribe
  mqttClient.onMessage(onMqttMessage);
  mqttClient.subscribe(topic_callback);

}

// ========== Emergency Button ==========
void handleEmergencyButton() {
  int state = digitalRead(emergencyButtonPin);

  if (state == LOW) {
    if (buttonPressTime == 0) buttonPressTime = millis();
    else if (!emergencyLatched && (millis() - buttonPressTime >= EMERGENCY_HOLD_MS)) {
      emergencyLatched = true;

      String emergencyPayload = "{";
      emergencyPayload += "\"id\":\"" + String(stick_id) + "\",";
      emergencyPayload += "\"emergency\":1,";
      emergencyPayload += "\"latitude\":" + String(lat, 6) + ",";
      emergencyPayload += "\"longitude\":" + String(lng, 6) + ",";
      emergencyPayload += "\"message\":\"Tombol darurat ditekan >=2.5s\"";
      emergencyPayload += "}";

      mqttClient.beginMessage(topic_emergency, (unsigned long)String(emergencyPayload.c_str()).length(), false, 1);
      mqttClient.print(emergencyPayload.c_str());
      mqttClient.endMessage();
      Serial.println("Emergency dikirim");

      for (int i = 0; i < 3; i++) {
        tone(buzzerPin, 4000, 300);  // nada tinggi & durasi lebih lama
        delay(350);
      }
    }
  } else {
    buttonPressTime = 0;
    emergencyLatched = false;
  }
}

// ========== Setup ==========
void setup() {
  Serial.begin(115200);

  serialGPS.begin(9600, SERIAL_8N1, 16, 17);
  Wire.begin(21, 22);
  if (!ina219.begin()) Serial.println("INA219 tidak terdeteksi.");

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(emergencyButtonPin, INPUT_PULLUP);

  connectWiFi();
  connectMQTT();
}

// ========== Loop ==========
void loop() {
  if (!mqttClient.connected()) connectMQTT();
  mqttClient.poll();

  unsigned long currentTime = millis();

  // --- GPS update ---
  while (serialGPS.available()) {
    gps.encode(serialGPS.read());
    if (gps.location.isUpdated()) {
      lat = gps.location.lat();
      lng = gps.location.lng();
    }
  }

  // --- Water level ---
  if (currentTime - lastWaterRead >= readInterval) {
    lastWaterRead = currentTime;
    waterValue = analogRead(waterSensorPin);
    waterActive = waterValue > 2000;
    if (waterActive) {
      waterBeepDelay = map(waterValue, 2000, 4095, 1000, 100);
      waterBeepDelay = constrain(waterBeepDelay, 50, 1000);
      ultrasonicActive = false;
    }
  }

  // --- Ultrasonik ---
  if (!waterActive && currentTime - lastUltrasonicRead >= readInterval) {
    lastUltrasonicRead = currentTime;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration_us = pulseIn(echoPin, HIGH, 30000UL);
    if (duration_us > 0) {
      distance_cm = duration_us / 58.2;
      if (distance_cm <= 125 && distance_cm >= 0) {
        beepDelay = getBeepDelay(distance_cm);
        ultrasonicActive = true;
      } else ultrasonicActive = false;
    } else ultrasonicActive = false;
  }

  // --- Buzzer control ---
  if (waterActive) {
    tone(buzzerPin, 3000);  // nada tinggi
    delay(100);
    noTone(buzzerPin);
    delay(waterBeepDelay);
  } else if (ultrasonicActive) {
    tone(buzzerPin, 2000);  // nada tinggi
    delay(100);
    noTone(buzzerPin);
    delay(beepDelay);
  }

  // Emergency
  handleEmergencyButton();

  // --- Baca baterai (INA219) ---
    // Tegangan total = busVoltage + shuntVoltage
    float busV = ina219.getBusVoltage_V();
    float shuntV = ina219.getShuntVoltage_mV() / 1000.0;
    batteryVoltage = busV + shuntV;

    // Estimasi persen Li-ion 1S (3.20V kosong – 4.20V penuh)
    batteryPercent = (int)roundf(constrain((batteryVoltage - 3.20f) / (4.20f - 3.20f) * 100.0f, 0.0f, 100.0f));

    // --- Baca arus dari INA219 ---
    float current_mA = ina219.getCurrent_mA();  // hasil dalam mA
    float current_A = current_mA / 1000.0;      // konversi ke Ampere

    // --- Kirim data sensor ke MQTT tiap 2 detik (tanpa battery) ---
    static unsigned long lastSendTime = 0;
    if (millis() - lastSendTime >= 2000) {
      lastSendTime = millis();

      String payload = "{";
      payload += "\"latitude\":" + String(lat, 6) + ",";
      payload += "\"longitude\":" + String(lng, 6) + ",";
      payload += "\"ultrasonic\":" + String(distance_cm) + ",";
      payload += "\"waterLevel\":" + String(waterValue);
      payload += "}";

      if (mqttClient.connected()) {
        mqttClient.beginMessage(topic_data);
        mqttClient.print(payload.c_str());
        mqttClient.endMessage();
        Serial.println("Data terkirim (topic data): " + payload);
      }
    }

    // --- Kirim persentase baterai setiap 20 detik ---
    static unsigned long lastBatterySend = 0;
    const unsigned long batteryInterval = 20000;  // 20 detik

    if (millis() - lastBatterySend >= batteryInterval) {
      lastBatterySend = millis();
      lastBatteryPercent = batteryPercent;  // update terakhir dikirim

      if (mqttClient.connected()) {
        String batteryStr = String(batteryPercent);  // angka saja, tanpa JSON
        mqttClient.beginMessage(topic_battery);
        mqttClient.print(batteryStr.c_str());
        mqttClient.endMessage();
        Serial.println("Battery% dikirim tiap 20s (topic battery): " + batteryStr);
      }
    }

    // --- Kirim data arus ke MQTT setiap 20 detik bersamaan dengan data baterai ---
    static unsigned long lastAmpereSend = 0;
    const unsigned long ampereInterval = 20000; // 20 detik
    if (millis() - lastAmpereSend >= ampereInterval) {
      lastAmpereSend = millis();

      if (mqttClient.connected()) {
        String ampereStr = String(current_A, 3); // tampilkan 3 angka di belakang koma
        mqttClient.beginMessage(topic_ampere);
        mqttClient.print(ampereStr.c_str());
        mqttClient.endMessage();
        Serial.println("🔌 Arus dikirim ke MQTT (topic ampere): " + ampereStr + " A");
      }
    }


  // Jaga loop MQTT kalau konek
  if (mqttClient.connected()) mqttClient.poll();
}