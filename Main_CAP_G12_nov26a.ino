#include "arduino_secrets.h"
#include "thingProperties.h" // Includes IoT Cloud properties

// Server and sensor pin definitions
WiFiServer server(80);  // Initialize a WiFi server on port 80
const int forceSensorPin = A0;  // Analog pin for force sensor
const int alcoholSensorPin = A1;  // Analog pin for alcohol sensor
const int buzzer = 13;  // Pin for buzzer output

// Variables
float timer = 1;
double EAR = 0;  // Eye Aspect Ratio received from client


// Setup function: runs once when the program starts
void setup() {
  Serial.begin(9600);  // Initialize serial communication at 9600 baud
  initProperties();  // Initialize IoT Cloud properties
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);  // Start IoT Cloud connection
  setDebugMessageLevel(2);  // Set debug message level
  ArduinoCloud.printDebugInfo();  // Print debug information
  pinMode(buzzer, OUTPUT);  // Set buzzer pin as output
  server.begin();  // Start the WiFi server
}

// Main loop: runs repeatedly after setup
void loop() {
  ArduinoCloud.update();  // Update IoT Cloud properties

  // Alcohol sensor calculations
  float rawAlcoholValue = analogRead(alcoholSensorPin) - 98;  // Read sensor value
  float alcoholVoltage = (rawAlcoholValue / 1023.0) * 5.0;  // Convert to voltage
  float alcoholPPM = (alcoholVoltage - 0.1005) * 200.0;  // Convert to ppm
  if (alcoholVoltage < 0.1005) alcoholPPM = 0;  // Avoid negative ppm values

  // Force sensor calculations (pressure)
  double rawForceValue = analogRead(forceSensorPin);  // Read force sensor value
  float a = 8852.182184, b = -64.50769273, c = 0.1396801832;  // Calibration coefficients
  double calculatedPressure = c * pow(rawForceValue, 2) + b * rawForceValue + a;

  // Read EAR value from Serial input
  if (Serial.available() > 0) {
    EAR = Serial.parseFloat();
    Serial.print("EAR value received: ");
    Serial.println(EAR, 6);
  }

  // Read EAR value from WiFi client
  WiFiClient client = server.available();
  if (client) {
    String request = client.readStringUntil('\r');
    if (request.indexOf("GET /?EAR=") >= 0) {
      String earValueStr = request.substring(request.indexOf("EAR=") + 4);
      EAR = earValueStr.toDouble();
      Serial.println("Received EAR: " + String(EAR, 6));
    }
    client.flush();
    client.stop();
  }

  // Pressure value handling
  if (rawForceValue < 10) {
    pressure = 0;
    intger_pressurepascal = 0;
    perssurepascal = 0;
  } else {
    pressure = rawForceValue;
    intger_pressurepascal = calculatedPressure / 1000;  // Convert to kPa
    perssurepascal = calculatedPressure / 1000;  // Convert to kPa
  }

  // Update IoT Cloud variables
  pPM_Alcohol = alcoholPPM;
  main_Alcohol = rawAlcoholValue;
  int_Alcohol_PPM = alcoholPPM;
  emailalcohol = (alcoholPPM >= 240) ? 1 : 0;
eYE_EAR=EAR;
eYE_EAR_Integer=EAR;
  // Eye state and buzzer control
  if (EAR < 0.18) {  // Eyes closed
    tone(buzzer, 300, 1000);  // Activate buzzer
    eye_state = 0;
  } else {  // Eyes open
    noTone(buzzer);
    eye_state = 1;
  }

  delay(50);  // Delay to stabilize readings
}

void onPressureChange() {}
void onEyeStateChange() {}
void onMainAlcoholChange() {}
void onPPMAlcoholChange() {}
void onEmailalcoholChange() {}
void onIntAlcoholPPMChange() {}
void onIntgerPressurepascalChange() {}
void onPerssurepascalChange() {}
void onEYEEARChange()  {}
void onEYEEARIntegerChange()  {}