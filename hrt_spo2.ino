#include <Wire.h>
#include "MAX30105.h"  // SparkFun MAX3010x library
#include "heartRate.h"

MAX30105 particleSensor;

#define FINGER_ON 3000     // Finger-on threshold for IR signal
#define FSpO2 0.7          // Filter factor for estimated SpO₂ (smoothing)
#define TIMETOBOOT 3000    // Sensor stabilization time
#define RATE_SIZE 4        // Number of samples for BPM averaging

byte rates[RATE_SIZE];     // Array of heart rates for averaging
byte rateSpot = 0;
long lastBeat = 50;

float beatsPerMinute = 0;
float beatAvg = 0;
double ESpO2 = 95.0;       // Initial estimated SpO₂
double avgSpO2 = 95.0;     // Averaged SpO₂
double sumredrms = 0.0, sumirrms = 0.0;
double avered = 0.0, aveir = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);  // Set SDA and SCL pins
  
  Serial.println("Initializing...");

  // Initialize sensor
  
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 not found. Please check wiring/power.");
    while (1);
  }

  // Configure sensor
  byte ledBrightness = 50;  // Adjust dynamically based on touch detection
  byte sampleAverage = 4;
  byte ledMode = 2;
  int sampleRate = 400;
  int pulseWidth = 411;
  int adcRange = 16384;

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.setPulseAmplitudeRed(0x1F);  // Initial Red LED brightness
  particleSensor.setPulseAmplitudeGreen(0);   // Turn off Green LED

}

void loop() {
  uint32_t ir, red;
  double fred, fir;
  double SpO2 = 0.0;

  particleSensor.check();  // Check if new data is available

  while (particleSensor.available()) {
    red = particleSensor.getFIFORed(); // Get Red LED data
    ir = particleSensor.getFIFOIR();  // Get IR LED data

    fred = (double)red;
    fir = (double)ir;

    // Detect touch based on IR signal
    if (fir > FINGER_ON) {
      // Calculate SpO₂
      avered = avered * 0.95 + fred * 0.05;
        aveir = aveir * 0.95 + fir * 0.05;
        sumredrms += (fred - avered) * (fred - avered);
        sumirrms += (fir - aveir) * (fir - aveir);

        // Calculate SpO₂
        if (sumredrms > 0 && sumirrms > 0) {
          double redRMS = sqrt(sumredrms);
          double irRMS = sqrt(sumirrms);

          double R = (redRMS / avered) / (irRMS / aveir);
          R = constrain(R, 0.5, 1.0);  // Clamp R value to avoid outliers

          SpO2 = -23.3 * (R - 0.4) + 100;  // SpO₂ calculation formula
          SpO2 = constrain(SpO2, 70.0, 100.0);  // Clamp SpO₂ to realistic range

          ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;  // Low-pass filter for smooth SpO₂
          avgSpO2 = (avgSpO2 * 0.9) + (ESpO2 * 0.1);     // Weighted average for stability
        }}

    // Heart Rate Calculation
    if (checkForBeat(fir)) {
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);
      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;

        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) {
          beatAvg += rates[x];
        }
        beatAvg /= RATE_SIZE;
      }
    }

    // Print Heart Rate and SpO₂
    if (fir > FINGER_ON) {
      Serial.print("BPM=");
      Serial.print(beatsPerMinute, 1);
      Serial.print(", Avg BPM=");
      Serial.print(beatAvg, 1);
      Serial.print(", SpO₂=");
      Serial.print(ESpO2, 1);
      Serial.print(", Avg SpO₂=");
      Serial.println(avgSpO2, 1);

    // Reset RMS calculations after each sample batch
        sumredrms = 0.0;
        sumirrms = 0.0;
 } else {
      Serial.println("No reliable touch detected.");
       
    }

    particleSensor.nextSample(); // Move to the next sample
  }

}
