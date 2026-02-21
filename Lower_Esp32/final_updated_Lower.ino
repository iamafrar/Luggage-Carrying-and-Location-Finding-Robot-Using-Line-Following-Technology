#include <HardwareSerial.h>

#define UART_RX 16
#define UART_TX 17
HardwareSerial FromUpper(2);

// ===== IR sensors =====
const int IR_L = 35;
const int IR_C = 39;
const int IR_R = 34;

const int BLACK = 1;

// ===== Motor driver pins =====
const int ENA = 14;
const int ENB = 32;
const int IN1 = 27;
const int IN2 = 26;
const int IN3 = 25;
const int IN4 = 33;

// ===== Speeds =====
int FWD  = 45;
int TURN = 60;
int KICK = 170;
int KICK_MS = 80;

// ===== Navigation variables =====
int destination = 0;
bool allowMove = false;

int leftCount = 0;
int rightCount = 0;
bool turnedToDest = false;

enum DriveState { IDLE, GOING, ARRIVED, RETURNING, HOME };
DriveState state = IDLE;

// Marker (111) hold time
unsigned long markStart = 0;
const unsigned long MARK_MS = 250;

bool onBlack(int v) { return v == BLACK; }

// ---------------- Motor helpers ----------------
void setLeft(int spd) {
  if (spd >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else          { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); spd = -spd; }
  spd = constrain(spd, 0, 255);
  ledcWrite(ENA, spd);
}

void setRight(int spd) {
  if (spd >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else          { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); spd = -spd; }
  spd = constrain(spd, 0, 255);
  ledcWrite(ENB, spd);
}

void stopMotors() {
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void forwardKickSlow() {
  setLeft(KICK); setRight(KICK);
  delay(KICK_MS);
  setLeft(FWD); setRight(FWD);
}

void slightLeft() {   // correction left
  setLeft(FWD);
  setRight(TURN);
}

void slightRight() {  // correction right
  setLeft(TURN);
  setRight(FWD);
}

void rotateLeft() {   // in-place rotate left
  setLeft(-TURN);
  setRight(TURN);
}

void rotateRight() {  // in-place rotate right
  setLeft(TURN);
  setRight(-TURN);
}

// ---------------- Junction / Marker detect ----------------
bool isLeftJunction() {
  bool LB = onBlack(digitalRead(IR_L));
  bool CB = onBlack(digitalRead(IR_C));
  return (LB && CB);
}

bool isRightJunction() {
  bool RB = onBlack(digitalRead(IR_R));
  bool CB = onBlack(digitalRead(IR_C));
  return (RB && CB);
}

bool isMarker111() {
  bool LB = onBlack(digitalRead(IR_L));
  bool CB = onBlack(digitalRead(IR_C));
  bool RB = onBlack(digitalRead(IR_R));
  return (LB && CB && RB);
}

// ---------------- Line follow ----------------
void lineFollowStep() {
  bool LB = onBlack(digitalRead(IR_L));
  bool CB = onBlack(digitalRead(IR_C));
  bool RB = onBlack(digitalRead(IR_R));

  if (!LB && CB && !RB)      forwardKickSlow();  // 010
  else if (LB && !CB && !RB) slightLeft();       // 100
  else if (RB && !CB && !LB) slightRight();      // 001
  else if (CB)               forwardKickSlow();  // center black with others
  else                        stopMotors();      // all white
}

// =======================================================
// ✅ UPDATED TURNING (NO TIME LIMIT) — only 센터 BLACK stops
// =======================================================
void turnLeftToLine() {
  // kick to start turning
  rotateLeft();
  setLeft(-KICK); setRight(KICK);
  delay(KICK_MS);

  // rotate until center sensor sees black
  while (digitalRead(IR_C) != BLACK) {
    rotateLeft();
    delay(1);      // watchdog safe
    yield();

    // allow UART to stop turning if needed (simple safety)
    if (FromUpper.available()) return;
  }

  stopMotors();
  delay(60);
}

void turnRightToLine() {
  rotateRight();
  setLeft(KICK); setRight(-KICK);
  delay(KICK_MS);

  while (digitalRead(IR_C) != BLACK) {
    rotateRight();
    delay(1);
    yield();

    if (FromUpper.available()) return;
  }

  stopMotors();
  delay(60);
}

// 180 turn (time based) — keep as your tuned value
const unsigned long TURN180_MS = 900;
void turn180Left() {
  rotateLeft();
  delay(TURN180_MS);
  stopMotors();
  delay(120);
}

// ---------------- UART read ----------------
void readUART() {
  if (!FromUpper.available()) return;

  String msg = FromUpper.readStringUntil('\n');
  msg.trim();
  if (msg.length() == 0) return; //ignore empty msg

  Serial.print("[FROM UPPER] ");
  Serial.println(msg);

  if (msg.startsWith("OVERWEIGHT")) {
    allowMove = false;
    destination = 0;
    state = IDLE;
    stopMotors();
    return;
  }

  if (msg == "READY") {
    if (state == ARRIVED) {
      state = RETURNING;
      turn180Left();
    } else if (state == HOME || state == IDLE) {
      state = IDLE;
    }
    return;
  }

  if (msg.indexOf("WEIGHT_OK") >= 0 && msg.indexOf("DEST:") >= 0) {
    int p = msg.indexOf("DEST:");
    int d = msg.substring(p + 5).toInt();

    if (d >= 1 && d <= 3) {
      destination = d;
      allowMove = true;

      leftCount = 0;
      rightCount = 0;
      turnedToDest = false;

      state = GOING;
    }
    return;
  }
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  delay(200);

  FromUpper.begin(115200, SERIAL_8N1, UART_RX, UART_TX);

  pinMode(IR_L, INPUT);
  pinMode(IR_C, INPUT);
  pinMode(IR_R, INPUT);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  ledcAttach(ENA, 1000, 8);
  ledcAttach(ENB, 1000, 8);

  stopMotors();
  Serial.println("LOWER ESP32 READY (UART navigation) - NO turn timeout");
}

// ---------------- Loop ----------------
void loop() {
  readUART();

  if (state == IDLE) {
    stopMotors();
    delay(20);
    return;
  }

  if (state == GOING) {
    if (!allowMove || destination == 0) {
      stopMotors();
      state = IDLE;
      return;
    }

    // ARRIVAL marker detection (111)
    if (turnedToDest && isMarker111()) {
      if (markStart == 0) markStart = millis();
      if (millis() - markStart >= MARK_MS) {
        stopMotors();
        state = ARRIVED;
        markStart = 0;
        Serial.println("ARRIVED. Waiting unload (READY from upper)...");
        return;
      }
    } else {
      markStart = 0;
    }

    // before turning to destination
    if (!turnedToDest) {

      if (isLeftJunction()) {
        leftCount++;
        Serial.printf("Left junction #%d\n", leftCount);

        if (destination == 1 && leftCount == 1) {
          stopMotors(); delay(60);
          turnLeftToLine();
          turnedToDest = true;
          delay(200);
          return;
        } else {
          forwardKickSlow();
          delay(120);
          return;
        }
      }

      if (isRightJunction()) {
        rightCount++;
        Serial.printf("Right junction #%d\n", rightCount);

        if ((destination == 3 && rightCount == 1) ||
            (destination == 2 && rightCount == 2)) {
          stopMotors(); delay(60);
          turnRightToLine();
          turnedToDest = true;
          delay(200);
          return;
        } else {
          forwardKickSlow();
          delay(120);
          return;
        }
      }
    }

    lineFollowStep();
    delay(10);
    return;
  }

  if (state == ARRIVED) {
    stopMotors();
    delay(50);
    return;
  }

  if (state == RETURNING) {

    // HOME marker (111)
    if (isMarker111()) {
      if (markStart == 0) markStart = millis();
      if (millis() - markStart >= MARK_MS) {
        stopMotors();
        state = HOME;

        allowMove = false;
        destination = 0;

        Serial.println("HOME reached. Stopped.");
        markStart = 0;
        return;
      }
    } else {
      markStart = 0;
    }

    lineFollowStep();
    delay(10);
    return;
  }

  if (state == HOME) {
    stopMotors();
    delay(100);
    return;
  }
}
