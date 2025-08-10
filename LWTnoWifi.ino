#include <WiFi.h>
#include <PubSubClient.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

#define echoPin 27
#define trigPin 26
#define buzzerPin 25
#define waterSensorPin 34
#define emergencyButtonPin 33

const char* stick_id = "0001";
const char* topic_data = "stick/0001/data";
const char* topic_emergency = "stick/0001/emergency";
const char* topic_callback = "stick/0001/callback";
const char* willTopic = "stick/0001/status";

bool emergencySent = false;

long duration, distance;
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

const char* ssid = "Afz";
const char* password = "totskuy1";
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);
TinyGPSPlus gps;
HardwareSerial serialGPS(2);  // RX=16, TX=17

// --------------- WiFi ---------------
void setup_wifi() {
  delay(100);
  Serial.print("Menghubungkan ke WiFi");
  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 5000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nTerhubung ke WiFi");
  } else {
    Serial.println("\nGagal terhubung ke WiFi. Melanjutkan tanpa koneksi.");
  }
}

// --------------- Callback MQTT Response ---------------
void playMelodyJemput() {
  int melody[] = { 262, 294, 330 }; // Do, Re, Mi
  int noteDuration = 300;

  for (int i = 0; i < 3; i++) {
    tone(buzzerPin, melody[i]);
    delay(noteDuration);
    noTone(buzzerPin);
    delay(100);
  }
}

void playMelodyTolak() {
  int melody[] = { 330, 294, 262 }; // Mi, Re, Do
  int noteDuration = 300;

  for (int i = 0; i < 3; i++) {
    tone(buzzerPin, melody[i]);
    delay(noteDuration);
    noTone(buzzerPin);
    delay(100);
  }
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Pesan diterima dari topik: ");
  Serial.println(topic);

  String msg;
  for (int i = 0; i < length; i++) {
    msg += (char)message[i];
  }

  Serial.println("Isi pesan response: " + msg);

  // Nada respons dinamis
  if (msg == "jemput") {
    playMelodyJemput(); // nada terima
  } else if (msg == "tolak") {
    playMelodyTolak(); // nada penolakan
  }
}

// --------------- MQTT Reconnect ---------------
void reconnect() {
  while (!client.connected()) {
    Serial.print("Menghubungkan ke MQTT...");

    // LWT setup
    const char* willMessage = "offline";
    const char* onlineMessage = "online";

    if (client.connect("ESP32ClientStick1", willTopic, 0, true, willMessage)) {
      Serial.println("Berhasil");

      // Publish status ONLINE setelah koneksi berhasil
      client.publish(willTopic, onlineMessage, true);  // retained = true

      // Subscribe ke topik callback
      client.subscribe(topic_callback);
      Serial.println("Subscribed ke: " + String(topic_callback));
    } else {
      Serial.print("Gagal, rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

// --------------- Dinamis Buzzer ---------------
int getBeepDelay(int distance) {
  int minDelay = 50;
  int maxDelay = 1000;
  int maxDistance = 125;

  int delayVal = map(distance, 0, maxDistance, minDelay, maxDelay);
  return constrain(delayVal, minDelay, maxDelay);
}


// --------------- Setup ---------------
void setup() {
  Serial.begin(115200);
  serialGPS.begin(9600, SERIAL_8N1, 16, 17);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(emergencyButtonPin, INPUT_PULLUP);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  reconnect();
}

// --------------- Loop ---------------
void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) reconnect();
    client.loop();
  }

  unsigned long currentTime = millis();

  while (serialGPS.available()) {
    gps.encode(serialGPS.read());
    if (gps.location.isUpdated()) {
      lat = gps.location.lat();
      lng = gps.location.lng();
    }
  }

  if (currentTime - lastWaterRead >= readInterval) {
    lastWaterRead = currentTime;
    waterValue = analogRead(waterSensorPin);

    if (waterValue > 300) {
      waterBeepDelay = map(waterValue, 300, 4095, 1000, 100);
      waterBeepDelay = constrain(waterBeepDelay, 50, 1000);
      waterActive = true;
      ultrasonicActive = false;
    } else {
      waterActive = false;
    }
  }

  if (!waterActive && currentTime - lastUltrasonicRead >= readInterval) {
    lastUltrasonicRead = currentTime;

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    duration = pulseIn(echoPin, HIGH);
    distance = duration / 58.2;

    // if (distance <= 125) {
    //   beepDelay = map(distance, 0, 180, 50, 1000);
    //   beepDelay = constrain(beepDelay, 50, 1000);
    //   ultrasonicActive = true;
    // } else {
    //   ultrasonicActive = false;
    //   noTone(buzzerPin);
    // }
    
    if (distance <= 125) {
      beepDelay = getBeepDelay(distance);
      ultrasonicActive = true;
    } else {
      ultrasonicActive = false;
      noTone(buzzerPin);
    }
  }

  if (waterActive) {
    tone(buzzerPin, 3000);
    delay(100);
    noTone(buzzerPin);
    delay(waterBeepDelay);
  } else if (ultrasonicActive) {
    tone(buzzerPin, 2000);
    delay(100);
    noTone(buzzerPin);
    delay(beepDelay);
  }

  // ---- Tombol Darurat ----
  if (digitalRead(emergencyButtonPin) == LOW && !emergencySent) {
    emergencySent = true;

    String emergencyPayload = "{";
    emergencyPayload += "\"id\":\"" + String(stick_id) + "\",";
    emergencyPayload += "\"emergency\":1,";
    emergencyPayload += "\"latitude\":" + String(lat, 6) + ",";
    emergencyPayload += "\"longitude\":" + String(lng, 6) + ",";
    emergencyPayload += "\"message\":\"Tombol darurat ditekan\"";
    emergencyPayload += "}";

    if (WiFi.status() == WL_CONNECTED) {
      client.publish(topic_emergency, emergencyPayload.c_str());
      Serial.println("⚠️ Pesan darurat dikirim ke topic emergency");
    }
  }

  if (digitalRead(emergencyButtonPin) == HIGH) {
    emergencySent = false;
  }

  // ---- Kirim Data Sensor ke MQTT ----
  if (millis() - lastSendTime >= 2000) {
    lastSendTime = millis();
    String payload = "{";
    payload += "\"latitude\":" + String(lat, 6) + ",";
    payload += "\"longitude\":" + String(lng, 6) + ",";
    payload += "\"ultrasonic\":" + String(distance) + ",";
    payload += "\"waterLevel\":" + String(waterValue) + ",";
    payload += "}";

    client.publish(topic_data, payload.c_str());
    Serial.println("✅ Data terkirim ke MQTT: " + payload);
  }
}
