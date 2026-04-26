#include <DHT.h>
#include <Servo.h>

#define DHTPIN 2
#define DHTTYPE DHT11      
#define SERVO_PIN 9
#define POT_PIN A0

DHT dht(DHTPIN, DHTTYPE);
Servo myServo;

// --- PID Parameters ---
// Adjust these to change how the system reacts
float Kp = 5.0;   // Proportional: How hard to push based on current error
float Ki = 0.1;   // Integral: How hard to push based on accumulated past error
float Kd = 1.0;   // Derivative: How much to "brake" to prevent overshoot

// --- PID Variables ---
float setPoint;
float temperature;
float error, lastError = 0;
float integral = 0;
float derivative;
float output;

// --- Timing ---
unsigned long lastTime = 0;
const unsigned long sampleTime = 1000; // 1 second update rate

void setup() {
  Serial.begin(9600);
  dht.begin();
  myServo.attach(SERVO_PIN);

  // Print Header for Serial Plotter
  Serial.println("Temperature,SetPoint,ServoOutput");
}

void loop() {
  unsigned long now = millis();

  // Run the PID loop exactly every 'sampleTime'
  if (now - lastTime >= sampleTime) {
    // Calculate time change in seconds (dt = 1.0)
    float dt = (now - lastTime) / 1000.0; 
    lastTime = now;

    // 1. Read Sensor Data
    temperature = dht.readTemperature();
    
    // Safety check: if sensor fails, stop the loop
    if (isnan(temperature)) {
      Serial.println("Error: DHT sensor not found!");
      return; 
    }

    // 2. Read Target (Setpoint) from Potentiometer
    int potValue = analogRead(POT_PIN);
    setPoint = map(potValue, 0, 1023, 20, 40); // Target range: 20°C to 40°C

    // 3. PID Calculations
    error = setPoint - temperature;

    // --- ANTI-WINDUP LOGIC ---
    // Only add to integral if the servo isn't already at its max/min limit.
    // This prevents the "climbing line" issue you saw earlier.
    if (output > 0 && output < 180) {
      integral += error * dt;
    }

    derivative = (error - lastError) / dt;

    // Compute total PID sum
    output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // 4. Physical Constraints
    // Keep the value between 0 and 180 for the servo
    output = constrain(output, 0, 180);
    myServo.write((int)output);

    // Save error for the next derivative calculation
    lastError = error;

    // 5. Output to Serial Plotter
    // Format: Value1,Value2,Value3
    Serial.print(temperature); 
    Serial.print(",");
    Serial.print(setPoint);
    Serial.print(",");
    Serial.println(output);
  }
}