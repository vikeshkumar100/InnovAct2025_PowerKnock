#include <SPI.h>
#include <Adafruit_PN532.h>
#include <ESP32Servo.h>

// --- RFID SPI pins ---
#define PN532_SCK  33
#define PN532_MOSI 13
#define PN532_MISO 35
#define PN532_SS   25

Adafruit_PN532 nfc(PN532_SS);

// --- Servo ---
Servo myServo;
const int servoPin = 27;

// --- LED ---
const int ledPin = 4;

// --- Knock Sensor ---
const int knockSensor = 36;
int threshold = 800;
const unsigned long debounceTime = 250;
bool knockActive = false;
unsigned long lastKnockTime = 0;
int knockCount = 0;
const unsigned long interKnockTimeout = 2500;
unsigned long lastKnockDetected = 0;

// --- Servo timing ---
bool accessGranted = false;
unsigned long accessStartTime = 0;
const unsigned long accessDuration = 10000;

// --- RFID UIDs and knock patterns ---
uint8_t allowedUIDs[2][4] = {
  {0xCD, 0x22, 0x0C, 0x06},
  {0x3A, 0xE8, 0x1B, 0x06}
};
int knockPatterns[2] = {3, 5};
int currentRequiredKnocks = 0;
bool rfidMatched = false;
int currentCardIndex = -1;

// --- IR Sensors ---
const int irA = 12;
const int irB = 14;
int totalPersons = 0;
bool aFirst = false;
bool bFirst = false;
bool counted = false;
unsigned long lastTriggerTime = 0;
const unsigned long IR_TIMEOUT = 2000;

// --- Motor Relay ---
const int motorRelayPin = 26;
const bool relayActiveLow = true;

void setup() {
  Serial.begin(115200);

  // Servo setup
  myServo.attach(servoPin);
  myServo.write(180); // Closed

  // LED setup
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Knock baseline
  long sum = 0;
  for(int i=0; i<20; i++){
    sum += analogRead(knockSensor);
    delay(5);
  }
  int baseline = sum / 20;
  threshold += baseline;
  Serial.print("Knock threshold set to: "); Serial.println(threshold);

  // Initialize PN532
  SPI.begin(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);
  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.println("PN532 not found!");
    while(1);
  }
  nfc.SAMConfig();
  Serial.println("Waiting for RFID card...");

  // IR sensors
  pinMode(irA, INPUT);
  pinMode(irB, INPUT);

  // Motor relay
  pinMode(motorRelayPin, OUTPUT);
  if(relayActiveLow) digitalWrite(motorRelayPin, HIGH);
  else digitalWrite(motorRelayPin, LOW);
}

void loop() {
  unsigned long now = millis();

  // --- RFID + Knock ---
  checkRFID();
  detectKnock();

  // --- IR sensor ---
  checkIR(now);

  // --- Access duration ---
  if(accessGranted && now - accessStartTime >= accessDuration){
    myServo.write(180);  // close
    digitalWrite(ledPin, LOW);
    accessGranted = false;
    rfidMatched = false;
    currentCardIndex = -1;
  }

  delay(10);
}

// =================== Functions ===================

void checkRFID(){
  if(!rfidMatched && !accessGranted){
    uint8_t success;
    uint8_t uid[7];
    uint8_t uidLength;
    success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

    if(success){
      bool allowed = false;
      for(int i=0; i<2; i++){
        allowed = true;
        for(int j=0; j<4; j++){
          if(uid[j] != allowedUIDs[i][j]){
            allowed = false;
            break;
          }
        }
        if(allowed){
          currentCardIndex = i;
          currentRequiredKnocks = knockPatterns[i];
          break;
        }
      }

      if(allowed){
        Serial.print("RFID correct! Card index: "); Serial.println(currentCardIndex);
        blinkLED(1);
        rfidMatched = true;
        knockCount = 0;
        lastKnockDetected = 0;
      } else {
        Serial.println("RFID wrong!");
        blinkLED(2);
      }
      delay(500);
    }
  }
}

void detectKnock(){
  unsigned long now = millis();
  int val = analogRead(knockSensor);
  if(val >= threshold && !knockActive && now - lastKnockTime > debounceTime){
    knockActive = true;
    lastKnockTime = now;
    knockCount++;
    lastKnockDetected = now;
    Serial.print("Knock detected! Count: "); Serial.println(knockCount);
    blinkLED(1);
  }
  if(val < threshold) knockActive = false;

  // Check knock pattern after user stops knocking
  if(rfidMatched && knockCount > 0 && (now - lastKnockDetected > interKnockTimeout)){
    if(knockCount == currentRequiredKnocks){
      Serial.println("Access granted!");
      digitalWrite(ledPin, HIGH);
      myServo.write(0); // open
      accessStartTime = now;
      accessGranted = true;
    } else {
      Serial.println("Wrong knock pattern. Scan RFID again.");
      blinkLED(2);
      rfidMatched = false;  // require new RFID
    }
    knockCount = 0;
    lastKnockDetected = 0;
    currentCardIndex = -1;
  }
}

void checkIR(unsigned long now){
  int stateA = digitalRead(irA);
  int stateB = digitalRead(irB);

  if(stateA == 0 && !aFirst && !bFirst) { aFirst = true; lastTriggerTime = now; }
  if(stateB == 0 && !bFirst && !aFirst) { bFirst = true; lastTriggerTime = now; }

  if(stateA == 0 && stateB == 0 && !counted){
    if(aFirst && !bFirst) totalPersons++;
    else if(bFirst && !aFirst) totalPersons--;
    if(totalPersons < 0) totalPersons = 0;
    Serial.print("Total persons: "); Serial.println(totalPersons);
    counted = true;
  }

  if(stateA == 1 && stateB == 1){
    aFirst = false; bFirst = false; counted = false; lastTriggerTime = 0;
  }

  if(lastTriggerTime > 0 && (now - lastTriggerTime > IR_TIMEOUT)){
    aFirst = false; bFirst = false; counted = false; lastTriggerTime = 0;
  }

  // Motor control
  if(totalPersons > 0){
    if(relayActiveLow) digitalWrite(motorRelayPin, LOW);
    else digitalWrite(motorRelayPin, HIGH);
  } else{
    if(relayActiveLow) digitalWrite(motorRelayPin, HIGH);
    else digitalWrite(motorRelayPin, LOW);
  }
}

void blinkLED(int times){
  for(int i=0; i<times; i++){
    digitalWrite(ledPin, HIGH);
    delay(150);
    digitalWrite(ledPin, LOW);
    delay(150);
  }
}
