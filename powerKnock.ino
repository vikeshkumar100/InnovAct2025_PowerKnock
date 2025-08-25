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
uint8_t allowedUIDs[4][4] = {
  {0xF3, 0x4C, 0x5E, 0xEC},  // Card 1
  {0x1A, 0x87, 0xCA, 0x62},  // Card 2
  {0x9A, 0x5E, 0xA1, 0x62},  // Card 3
  {0x5A, 0xA9, 0xCF, 0x62}   // Card 4
};
int knockPatterns[4] = {3, 7, 2, 5}; // corresponding knock counts
int currentRequiredKnocks = 0;
bool rfidMatched = false;
int currentCardIndex = -1;

// --- IR Sensors ---
const int irA = 22;
const int irB = 23;
int totalPersons = 0;
bool prevA = HIGH;
bool prevB = HIGH;
unsigned long lastATime = 0;
unsigned long lastBTime = 0;
const unsigned long MAX_DIFF = 1000; // 1 second max difference
bool counted = false; // for IR debounce

// --- Motor Relay ---
const int motorRelayPin = 26;
const bool relayActiveLow = true;

void setup() {
  Serial.begin(115200);

  // Servo setup
  myServo.attach(servoPin);
  myServo.write(180); // closed

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

  // IR sensor setup
  pinMode(irA, INPUT);
  pinMode(irB, INPUT);

  // Motor relay setup
  pinMode(motorRelayPin, OUTPUT);
  if(relayActiveLow) digitalWrite(motorRelayPin, HIGH); // OFF
  else digitalWrite(motorRelayPin, LOW); // OFF
}

void loop() {
  unsigned long now = millis();

  // RFID + Knock Logic
  checkRFID();
  detectKnock();

  // IR Sensor counting (independent of door)
  checkIR();

  // Access duration for servo
  if(accessGranted && now - accessStartTime >= accessDuration){
    myServo.write(180);  // close
    digitalWrite(ledPin, LOW);
    accessGranted = false;
    rfidMatched = false;
    currentCardIndex = -1;
  }

  // Motor relay control
  if(totalPersons > 0){
    if(relayActiveLow) digitalWrite(motorRelayPin, LOW);
    else digitalWrite(motorRelayPin, HIGH);
  } else{
    if(relayActiveLow) digitalWrite(motorRelayPin, HIGH);
    else digitalWrite(motorRelayPin, LOW);
  }

  delay(10); // small delay for loop stability
}

// ----------------- Functions -----------------

void checkRFID(){
  if(!rfidMatched && !accessGranted){
    uint8_t success;
    uint8_t uid[7];
    uint8_t uidLength;
    success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

    if(success){
      bool allowed = false;
      for(int i=0; i<4; i++){
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
      delay(500); // debounce
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

  // Evaluate knock pattern after pause
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
      rfidMatched = false;
    }

    knockCount = 0;
    lastKnockDetected = 0;
    currentCardIndex = -1;
  }
}

// ---------------- IR counting independent ----------------
void checkIR(){
  int stateA = digitalRead(irA);
  int stateB = digitalRead(irB);
  unsigned long now = millis();

  // Detect falling edges
  if(stateA == LOW && prevA == HIGH) lastATime = now;
  if(stateB == LOW && prevB == HIGH) lastBTime = now;

  // Both triggered within MAX_DIFF
  if(lastATime && lastBTime){
    long diff = (long)lastATime - (long)lastBTime;

    if(abs(diff) < MAX_DIFF && !counted){
      if(diff < 0) totalPersons++;  // A->B Entry
      else totalPersons--;           // B->A Exit
      if(totalPersons < 0) totalPersons = 0;

      Serial.print("Total persons: "); Serial.println(totalPersons);
      counted = true;  // mark counted
    }

    // Reset after counting
    if(abs(diff) >= MAX_DIFF || (stateA == HIGH && stateB == HIGH)){
      lastATime = 0;
      lastBTime = 0;
      counted = false;
    }
  }

  prevA = stateA;
  prevB = stateB;
}

// --- LED blink function ---
void blinkLED(int times){
  for(int i=0; i<times; i++){
    digitalWrite(ledPin, HIGH);
    delay(150);
    digitalWrite(ledPin, LOW);
    delay(150);
  }
}
