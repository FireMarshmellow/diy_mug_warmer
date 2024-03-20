#include <DFRobot_MLX90614.h>
#include <Arduino.h>

// MLX90614 sensor setup
DFRobot_MLX90614_I2C sensor;

// Thermistor reading setup
const int adcPin = 5; // Thermistor connected to A5
const float referenceResistor = 10000; // 10k resistor in Ohms
const float referenceTemperature = 298.15; // Reference temperature (25°C in Kelvin)
const float B = 3950; // B coefficient
const float referenceResistance = 100000; // Thermistor resistance at reference temperature (25°C)

// Hysteresis thresholds
const float heaterOnThreshold = 44; // Temperature to turn the heater on
const float heaterOffThreshold = 56; // Temperature to turn the heater off
const float maxHeaterTempC = 100; // Maximum allowable temperature for the heater

bool objectPresent = false; // Global variable to track the presence of the object
static bool heaterState = false; // Tracks the current state of the heater

void setup() {
  Serial.begin(115200);
  while(NO_ERR != sensor.begin()) {
    Serial.println("Communication with MLX90614 failed, please check connection");
    delay(3000);
  }
  pinMode(4, INPUT); // Sensor to check if the object is on the heater
  pinMode(2, OUTPUT); // Relay for heater
  pinMode(3, OUTPUT); // Relay for fan
  analogReadResolution(10); // Adjust for your board, if necessary

  Serial.println("Setup complete!");
}

void loop() {
  // Current state of object presence
  bool currentObjectPresent = digitalRead(4) == LOW; // Assuming HIGH means object is present

  // Check if the object was just placed on the heater
  if (currentObjectPresent && !objectPresent) {
    // Object was just placed, reset the heater state to allow it to turn on again
    heaterState = false;
    Serial.println("Object placed on heater. Ready to monitor temperature.");
  }

  // Update the global objectPresent state for the next loop iteration
  objectPresent = currentObjectPresent;

  if (!objectPresent) { // If the object is not present, turn everything off
    digitalWrite(2, LOW); // Turn off heater
    digitalWrite(3, LOW); // Turn off fan
    Serial.println("Object not detected. Both heater and fan are off.");
    delay(500); // Short delay before checking again
    return; // Skip the rest of the loop
  }

  // MLX90614 temperature reading
  float objectTemp = sensor.getObjectTempCelsius();
  Serial.print("MLX90614 Object Celsius: ");
  Serial.print(objectTemp);
  Serial.println(" °C");

  // Control logic for heater with hysteresis
  if(objectTemp < heaterOnThreshold && !heaterState) {
    digitalWrite(2, HIGH); // Turn on heater
    heaterState = true;
    Serial.println("Heater on, raising temperature.");
  } else if(objectTemp > heaterOffThreshold && heaterState) {
    digitalWrite(2, LOW); // Turn off heater
    heaterState = false;
    Serial.println("Heater off, temperature within range.");
  }

  // Fan control logic
  if(objectTemp > 55) {
    digitalWrite(3, HIGH); // Turn on fan to cool down
    Serial.println("Fan on, lowering temperature.");
  } else {
    digitalWrite(3, LOW); // Fan off
  }

  // Thermistor temperature reading for heater safety
  int adcValue = analogRead(adcPin);
  float voltage = adcValue * 3.3 / 1023.0;
  float thermistorResistance = referenceResistor * (3.3 / voltage - 1.0);
  float temperatureK = B / (log(thermistorResistance / referenceResistance) + B / referenceTemperature);
  float heaterTempC = temperatureK - 273.15;
  Serial.print("Heater Temperature: ");
  Serial.print(heaterTempC);
  Serial.println(" C");

  // Safety logic: If heater temperature exceeds 100°C, turn off the heater
  if(heaterTempC > maxHeaterTempC) {
    digitalWrite(2, LOW);
    heaterState = false; // Ensure the state is updated to reflect the heater being off
    Serial.println("Heater turned off due to excessive temperature.");
  }

  delay(500); // Delay before next loop iteration
}
