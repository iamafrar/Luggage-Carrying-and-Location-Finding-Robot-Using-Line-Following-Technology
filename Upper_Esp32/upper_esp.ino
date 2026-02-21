#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

#include <SPI.h>
#include <MFRC522.h>
#include "HX711.h"

#define WIFI_SSID     "Nasrullah"
#define WIFI_PASSWORD "87654321"

const char* RTDB_BASE =
  "https://luggage-robot-dashboard-default-rtdb.asia-southeast1.firebasedatabase.app";


#define UART_TX 17   
#define UART_RX 16   
HardwareSerial ToLower(2);  

#define HX711_DOUT 19
#define HX711_SCK  18

#define RFID_SCK   14
#define RFID_MISO  26
#define RFID_MOSI  27
#define RFID_SS    25
#define RFID_RST   33

const byte UID_LOC1[4] = {0x83, 0x3C, 0xD6, 0x05};
const byte UID_LOC2[4] = {0xCA, 0xFC, 0x69, 0x05};
const byte UID_LOC3[4] = {0xC2, 0x43, 0x14, 0xAC};

HX711 scale;
const float CAL_FACTOR = -1737.35f;//<<<<<<<<< 
const float LIMIT_G = 800.0f;
const float EMPTY_BAND_G = 100.0f;
const int WEIGHT_SAMPLES = 30;
const int EMPTY_STABLE_COUNT = 5;

MFRC522 mfrc522(RFID_SS, RFID_RST);

enum State { //<<<<<<<<<<<<<
  WAIT_CARD,
  WAIT_LUGGAGE,
  CHECK_WEIGHT,
  WAIT_REMOVE
};

State state = WAIT_CARD;
int selectedLocation = 0;
int emptyStable = 0; //box is empty

bool uidMatches(const MFRC522::Uid &uid, const byte *known, byte knownSize) {
  if (uid.size != knownSize) return false;
  for (byte i = 0; i < knownSize; i++) {
    if (uid.uidByte[i] != known[i]) return false;
  }
  return true;
} //>>>>>>> Compare scanned UID with a known UID and return true if they are the same.

void printUID(const MFRC522::Uid &uid) {
  for (byte i = 0; i < uid.size; i++) {
    if (uid.uidByte[i] < 0x10) Serial.print("0");
    Serial.print(uid.uidByte[i], HEX);
    if (i < uid.size - 1) Serial.print(" ");
  }
}

float readWeightGrams(int samples = WEIGHT_SAMPLES) {
  if (!scale.is_ready()) return NAN;
  return scale.get_units(samples);
}

bool isEmptyBox(float w) {
  return (w > -EMPTY_BAND_G && w < EMPTY_BAND_G);
}

const char* stateName(State s) {
  switch (s) {
    case WAIT_CARD:    return "WAIT_CARD";
    case WAIT_LUGGAGE: return "WAIT_LUGGAGE";
    case CHECK_WEIGHT: return "CHECK_WEIGHT";
    case WAIT_REMOVE:  return "WAIT_REMOVE";
    default:           return "UNKNOWN";
  }
}

String destinationName(int loc) {
  if (loc == 1) return "Location 1";
  if (loc == 2) return "Location 2";
  if (loc == 3) return "Location 3";
  return "--";
}

bool putRawJson(const String& path, const String& jsonPayload) {
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient https;
  String url = String(RTDB_BASE) + path + ".json";

  if (!https.begin(client, url)) return false;
  https.addHeader("Content-Type", "application/json");

  int code = https.PUT(jsonPayload);
  https.end();

  return (code >= 200 && code < 300);
}

bool putNumber(const String& path, const String& numberJson) {
  return putRawJson(path, numberJson);
}

bool putString(const String& path, const String& value) {
  String json = "\"";
  for (size_t i = 0; i < value.length(); i++) {
    char c = value[i];
    if (c == '\"') json += "\\\"";
    else if (c == '\\') json += "\\\\";
    else if (c == '\n') json += "\\n";
    else if (c == '\r') json += "\\r";
    else json += c;
  }
  json += "\"";
  return putRawJson(path, json);
}

void pushStateToFirebase(float weightG, const String& statusOverride = "") {
  if (WiFi.status() != WL_CONNECTED) return;

  if (!isnan(weightG)) {
    float weightKg = weightG / 1000.0f;
    putNumber("/robot/state/weightKg", String(weightKg, 3));
  }

  putNumber("/robot/state/updatedAt", String((unsigned long long)(millis())));

  if (statusOverride.length()) putString("/robot/state/statusText", statusOverride);
  else putString("/robot/state/statusText", String(stateName(state)));

  putString("/robot/state/mode", "AUTO");
  putString("/robot/state/destination", destinationName(selectedLocation));

  if (selectedLocation == 0) putString("/robot/state/rfidStatus", "NOT SCANNED");
  else putString("/robot/state/rfidStatus", "OK");
}

void sendToLower(const String& msg) {
  ToLower.println(msg);
  Serial.print("[UART->LOWER] ");
  Serial.println(msg);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  ToLower.begin(115200, SERIAL_8N1, UART_RX, UART_TX);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) { Serial.print("."); delay(300); }
  Serial.println("\nWiFi connected!");

  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SS);
  mfrc522.PCD_Init();
  delay(50);

  scale.begin(HX711_DOUT, HX711_SCK);
  scale.set_scale(CAL_FACTOR);

  Serial.println("System starting...");
  Serial.println("Keep the box EMPTY. Warming up 10 seconds...");
  delay(10000);

  Serial.println("Taring with EMPTY BOX...");
  scale.tare(30);
  Serial.println("Tare done.\n");

  Serial.println("=== READY ===");
  Serial.println("Scan the RFID card...");

  pushStateToFirebase(readWeightGrams(10), "READY");

  sendToLower("READY");
}

unsigned long lastPush = 0;
const unsigned long PUSH_INTERVAL_MS = 600;

void loop() {
  if (millis() - lastPush >= PUSH_INTERVAL_MS) {
    lastPush = millis();
    pushStateToFirebase(readWeightGrams(10));
  }

  switch (state) {

    case WAIT_CARD: {
      if (!mfrc522.PICC_IsNewCardPresent()) break;
      if (!mfrc522.PICC_ReadCardSerial()) break;

      Serial.print("Card UID: ");
      printUID(mfrc522.uid);
      Serial.println();

      int loc = 0;
      if (uidMatches(mfrc522.uid, UID_LOC1, 4)) loc = 1;
      else if (uidMatches(mfrc522.uid, UID_LOC2, 4)) loc = 2;
      else if (uidMatches(mfrc522.uid, UID_LOC3, 4)) loc = 3;

      mfrc522.PICC_HaltA();
      mfrc522.PCD_StopCrypto1();
      delay(50);

      if (loc == 0) {
        Serial.println("UNAUTHORIZED CARD!");
        selectedLocation = 0;
        pushStateToFirebase(readWeightGrams(10), "UNAUTHORIZED");

        sendToLower("UNAUTHORIZED");

        delay(800);
        break;
      }

      selectedLocation = loc;
      Serial.print("Valid Card -> Location ");
      Serial.println(selectedLocation);

      pushStateToFirebase(readWeightGrams(10), "CARD_OK");

      sendToLower("DEST:" + String(selectedLocation));

      Serial.println("Put the luggage in the box...");
      state = WAIT_LUGGAGE;

      delay(400);
      break;
    }

    case WAIT_LUGGAGE: {
      float w = readWeightGrams();
      if (isnan(w)) {
        Serial.println("HX711 not ready.");
        pushStateToFirebase(NAN, "HX711_ERROR");
        delay(300);
        break;
      }

      if (!isEmptyBox(w)) {
        Serial.println("Luggage detected. Checking weight...");
        state = CHECK_WEIGHT;
        pushStateToFirebase(w, "LUGGAGE_DETECTED");
      }

      delay(300);
      break;
    }

    case CHECK_WEIGHT: {
      float w = readWeightGrams();
      if (isnan(w)) {
        Serial.println("HX711 not ready.");
        pushStateToFirebase(NAN, "HX711_ERROR");
        delay(300);
        break;
      }

      if (w <= LIMIT_G) {
        Serial.print("WEIGHT OK. Destination Location ");
        Serial.println(selectedLocation);
        pushStateToFirebase(w, "WEIGHT_OK");
        sendToLower("WEIGHT_OK:" + String((int)w) + ",DEST:" + String(selectedLocation));

      } else {
        Serial.println("OVERWEIGHT! Reduce luggage (<= 800g).");
        pushStateToFirebase(w, "OVERWEIGHT");

        sendToLower("OVERWEIGHT:" + String((int)w));
      }

      Serial.println("Remove the luggage to continue...");
      emptyStable = 0;
      state = WAIT_REMOVE;

      delay(500);
      break;
    }

    case WAIT_REMOVE: {
      float w = readWeightGrams();
      if (isnan(w)) {
        Serial.println("HX711 not ready.");
        pushStateToFirebase(NAN, "HX711_ERROR");
        delay(300);
        break;
      }

      if (isEmptyBox(w)) emptyStable++;
      else emptyStable = 0;

      pushStateToFirebase(w, "REMOVE_LUGGAGE");

      if (emptyStable >= EMPTY_STABLE_COUNT) {
        Serial.println("Box empty. Ready for next user.\n");
        selectedLocation = 0;
        state = WAIT_CARD;
        pushStateToFirebase(w, "READY_NEXT");

        sendToLower("READY");
      }

      delay(300);
      break;
    }
  }
}
