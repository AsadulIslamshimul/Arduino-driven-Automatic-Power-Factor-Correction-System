/* ============================================================
 * Project: Arduino driven Automatic Power Factor Correction System (APFC)
 * Controller: Arduino Uno
 * Method: Zero Crossing Detection (Voltage & Current)
 * ============================================================ */

#include <LiquidCrystal.h>
#include <math.h>

/* -------------------- PIN CONFIG -------------------- */
#define VOLT_ZC_PIN 2      // INT0
#define CURR_ZC_PIN 3      // INT1

#define RELAY1_PIN 8
#define RELAY2_PIN 9
#define RELAY3_PIN 10

/* LCD: RS, EN, D4, D5, D6, D7 */
LiquidCrystal lcd(12, 11, 7, 6, 5, 4);

/* -------------------- CONSTANTS -------------------- */
#define SUPPLY_FREQ 50.0
#define HALF_CYCLE_US (1000000.0 / (SUPPLY_FREQ * 2))
#define TARGET_PF 0.95

/* -------------------- GLOBALS -------------------- */
volatile unsigned long tVoltage = 0;
volatile unsigned long tCurrent = 0;
volatile bool voltageCaptured = false;
volatile bool currentCaptured = false;

float powerFactor = 1.0;
float phaseAngle = 0.0;

/* -------------------- ISR -------------------- */
void voltageZC_ISR() {
    tVoltage = micros();
    voltageCaptured = true;
}

void currentZC_ISR() {
    tCurrent = micros();
    currentCaptured = true;
}

/* -------------------- SETUP -------------------- */
void setup() {
    pinMode(VOLT_ZC_PIN, INPUT);
    pinMode(CURR_ZC_PIN, INPUT);

    pinMode(RELAY1_PIN, OUTPUT);
    pinMode(RELAY2_PIN, OUTPUT);
    pinMode(RELAY3_PIN, OUTPUT);

    digitalWrite(RELAY1_PIN, LOW);
    digitalWrite(RELAY2_PIN, LOW);
    digitalWrite(RELAY3_PIN, LOW);

    lcd.begin(16, 2);
    lcd.clear();
    lcd.print("APFC System");
    lcd.setCursor(0, 1);
    lcd.print("Initializing");

    attachInterrupt(digitalPinToInterrupt(VOLT_ZC_PIN), voltageZC_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(CURR_ZC_PIN), currentZC_ISR, RISING);

    delay(2000);
    lcd.clear();
}

/* -------------------- MAIN LOOP -------------------- */
void loop() {
    if (voltageCaptured && currentCaptured) {
        noInterrupts();

        long deltaT = (long)tCurrent - (long)tVoltage;
        voltageCaptured = false;
        currentCaptured = false;

        interrupts();

        if (deltaT < 0) deltaT += HALF_CYCLE_US;

        phaseAngle = (deltaT / HALF_CYCLE_US) * 180.0;
        powerFactor = cos(phaseAngle * DEG_TO_RAD);

        if (powerFactor < 0) powerFactor = -powerFactor;

        controlCapacitors(powerFactor);
        displayData(powerFactor, phaseAngle);
    }
}

/* -------------------- CAPACITOR CONTROL -------------------- */
void controlCapacitors(float pf) {
    if (pf >= TARGET_PF) {
        allRelaysOff();
    } 
    else if (pf >= 0.90) {
        relayStep(1);
    } 
    else if (pf >= 0.80) {
        relayStep(2);
    } 
    else {
        relayStep(3);
    }
}

void relayStep(int step) {
    digitalWrite(RELAY1_PIN, step >= 1 ? HIGH : LOW);
    digitalWrite(RELAY2_PIN, step >= 2 ? HIGH : LOW);
    digitalWrite(RELAY3_PIN, step >= 3 ? HIGH : LOW);
}

void allRelaysOff() {
    digitalWrite(RELAY1_PIN, LOW);
    digitalWrite(RELAY2_PIN, LOW);
    digitalWrite(RELAY3_PIN, LOW);
}

/* -------------------- LCD DISPLAY -------------------- */
void displayData(float pf, float angle) {
    lcd.setCursor(0, 0);
    lcd.print("PF:");
    lcd.print(pf, 2);
    lcd.print(" Ang:");
    lcd.print(angle, 0);

    lcd.setCursor(0, 1);
    lcd.print("CAP:");
    lcd.print(digitalRead(RELAY1_PIN));
    lcd.print(digitalRead(RELAY2_PIN));
    lcd.print(digitalRead(RELAY3_PIN));
    lcd.print("      ");
}
