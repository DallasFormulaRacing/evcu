#include <Arduino.h>

#define RELAY_BJT_BASE_PIN 13
#define IMD_STATUS_INPUT 26
#define IMD_PWM_INPUT 14
#define STATUS_LED_OUTPUT 33

void decodeMhsSignal();
void calculateAndPrintResistance(float);
/*
IMD pins:
  Pin 1: Chassis/Elec gnd
  Pin 2: 12V supply
  Pin 3: Chassis GND
  Pin 4: Chassis GND SEPERATE CONNECTION
  Pin 5: Data Out, PWM (high side)
  Pin 6: NC
  Pin 7: NC
  Pin 8: Status Output (high side) - 10V 
*/
/*
This code is designed to work with a resistor divider stepping down the
  Bender's high-voltage output to a 3.3V level safe for the ESP32.

  !!! LOGIC CHANGE !!!
  This version uses DIRECT logic. An output of HIGH from the Bender corresponds
  to a HIGH reading on the ESP32.
  - OKHS HIGH (System OK) -> ESP32 Reads HIGH
  - OKHS LOW (Fault)      -> ESP32 Reads LOW

  - ESP32 GPIO 14: Connect to the output of the resistor divider for the IMD STATUS Pin 8 (OK_HS).
  - ESP32 GPIO 26: Connect to the output of the resistor divider for the IMD PWM Pin 5 (M_HS).
  - ESP32 GPIO 13: Connect to the base resistor of the NPN transistor driving the relay.
*/

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_BJT_BASE_PIN, OUTPUT);
  pinMode(IMD_STATUS_INPUT, INPUT);
  pinMode(IMD_PWM_INPUT, INPUT);
  pinMode(STATUS_LED_OUTPUT, OUTPUT);

  digitalWrite(RELAY_BJT_BASE_PIN, LOW);

  delay(3000); // 3 second startup delay 
}

int IMD_status = 0;
int IMD_Fault_Count = 0;
long period;
float frequency;
float duty_cycle;

void loop() {
  //digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)

  // Check IMD
  IMD_status = digitalRead(IMD_STATUS_INPUT); // This status is ONLY the IMD DC output value, not dependant on PWM
  decodeMhsSignal();


  if(IMD_status == LOW) {
    IMD_Fault_Count++;
    Serial.println("Fault!!!");
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(RELAY_BJT_BASE_PIN, LOW);
  }

  if(IMD_status == HIGH) {
    Serial.println("IMD Good!");
    digitalWrite(RELAY_BJT_BASE_PIN, HIGH);
  }

  Serial.print("Fault count: ");
  Serial.println(IMD_Fault_Count);


  delay(750); // check for 500ms
}

void decodeMhsSignal(int PWM_Signal_Pin) {
  // Measure the duration of the HIGH and LOW parts of the PWM signal
  long PULSE_TIMEOUT = 250000;
  long high_duration = pulseIn(PWM_Signal_Pin, HIGH, PULSE_TIMEOUT);
  long low_duration = pulseIn(PWM_Signal_Pin, LOW, PULSE_TIMEOUT);

  // Check for a timeout (no signal)
  if (high_duration == 0 || low_duration == 0) {
    Serial.println("MHS Signal: No signal or timeout.");
    return;
  }

  // --- Calculate Frequency and Duty Cycle ---
  period = high_duration + low_duration;
  frequency = 1000000.0 / period;
  duty_cycle = (float)high_duration / period * 100.0;

  Serial.println("--- MHS Signal Analysis ---");
  Serial.print("Frequency: ");
  Serial.print(frequency, 2);
  Serial.print(" Hz, Duty Cycle: ");
  Serial.print(duty_cycle, 2);
  Serial.println(" %");

  // --- Decode Status based on Frequency (from Datasheet page 5) ---
  if (frequency > 8 && frequency < 12) {
    Serial.print("Status: Normal Condition (10 Hz)\n");
    calculateAndPrintResistance(duty_cycle);

  } else if (frequency > 18 && frequency < 22) {
    Serial.print("Status: Undervoltage Condition (20 Hz)\n");
    calculateAndPrintResistance(duty_cycle);

  } else if (frequency > 28 && frequency < 32) {
    Serial.print("Status: Speed Start Measurement (30 Hz)\n");
    if (duty_cycle >= 5 && duty_cycle <= 10) {
      Serial.println("Insulation: Good");
    } else if (duty_cycle >= 90 && duty_cycle <= 95) {
      Serial.println("Insulation: Bad");
    }

  } else if (frequency > 38 && frequency < 42) {
    Serial.println("Status: DEVICE ERROR (40 Hz)");

  } else if (frequency > 48 && frequency < 52) {
    Serial.println("Status: EARTH CONNECTION FAULT (50 Hz)");

  } else {
    Serial.println("Status: Unknown or Invalid Frequency");
  }
   Serial.println("----------------------------------------------------------------");
}

void calculateAndPrintResistance(float dc) {
  // This formula is from the Bender IR155-3204 datasheet, page 5.
  // Rf = (90% * 1200 kΩ) / (dc_meas - 5%) - 1200 kΩ
  if (dc <= 5.5) {
    Serial.println("Insulation Resistance: > 10 MOhms (Infinite)");
  } else if (dc >= 94.5) {
    Serial.println("Insulation Resistance: 0 kOhms (Short Circuit)");
  } else {
    float resistance_kOhm = (108000.0 / (dc - 5.0)) - 1200.0;
    Serial.print("Insulation Resistance: ");
    Serial.print(resistance_kOhm, 2);
    Serial.println(" kOhms");
  }
}