#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSOX.h>
#include <Servo.h>
#include <string.h>


// Define pins for sensors and devices
#define OPENLOG_RX 0
#define OPENLOG_TX 1
#define OPENLOG_BAUD_RATE 9600 // Adjust the baud rate to match OpenLog's configuration
#define XBEE_BAUD_RATE 9600
#define XBEE_RX 8
#define XBEE_TX 9
#define SERVO_PIN1 16
#define SERVO_PIN2 17
#define adcPin 26
#define FEEDBACK_PIN 27
#define MOSFET_1 2
#define MOSFET_2 18
#define MOSFET_3 19

#define NUM_GROUND_SAMPLES 10
#define NUM_OVERSAMPLES 10
#define TELEMETRY_RATE_HZ 1
#define SENSOR_RATE_HZ 10

#define SEALEVELPRESSURE_HPA (1013.25)

// Define state variables
boolean LAUNCH_READY = true;
boolean ASCENT = false;
boolean SEPARATE = false;
boolean DESCENT = false;
boolean LANDED = false;

// Initialize sensors
Adafruit_BMP3XX bmp;
Adafruit_LSM6DSOX lsm;
Servo servo1;
Servo servo2;

// Variables for sensor data
float temperature = 0.0;
float pressure = 0.0;
float height = 0.0;
float previousHeight = 0.0;
float gyro_R = 0.0;
float gyro_P = 0.0;
float gyro_Y = 0.0;
float x = 0.0;
float y = 0.0;
float z = 0.0;
float heightAGL = 0.0;
float groundHeight1 = 0.0;
bool groundHeightSet = false;
float heightAGLOversampled = 0.0;
float heightOversampled = 0.0;
float previousHeightOversampled = 0.0;
unsigned long oversampleInterval = 10;
unsigned long previousOversampleMillis = 0;

static unsigned long servo1StartTime = 0;
static unsigned long servo2StartTime = 0;
const unsigned long servoUpdateInterval = 10; // Adjust for servo speed

// Servo control variables
const int servo1MinAngle = 0;
const int servo1MaxAngle = 150;
const int servo2MinAngle = 0;
const int servo2MaxAngle = 270;
int servo1Angle = servo1MinAngle;
int servo2Angle = servo2MinAngle;

//non-blocking timing
unsigned long previousMillis = 0;
unsigned long currentMillis4 = millis();
unsigned long previousTelemetryMillis = 0;
const unsigned long sensorInterval = 1000 / SENSOR_RATE_HZ;  // Update height every 10 milliseconds
const unsigned long telemetryInterval = 1000 / TELEMETRY_RATE_HZ; // 10 Hz (0.1 second interval)

// CSV data variables
unsigned int packetCount = 0; // Packet count
char swState[20] = "LAUNCH_READY"; // Software state
char plState = 'N'; // Payload state
float voltage = 0.0; // Voltage measurement
char csvBuffer[128]; // CSV data buffer

// Voltage
const float referenceVoltage = 3.3; // Reference voltage (V) of the ADC
const float r1 = 6800.0; // Resistance (ohms) of the 6.8 kilo Ohm resistor
const float r2 = 3300.0; // Resistance (ohms) of the 3.3 kilo Ohm resistor


void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize OpenLog communication
  Serial1.setTX(OPENLOG_RX);
  Serial1.setRX(OPENLOG_TX);
  Serial1.begin(OPENLOG_BAUD_RATE);

  // Initialize XBee communication
  Serial2.setTX(XBEE_RX);
  Serial2.setRX(XBEE_TX);
  Serial2.begin(XBEE_BAUD_RATE);

  // Initialize I2C communication
  Wire.begin();

  // Initialize sensors
  if (!bmp.begin_I2C(0x77)) { // Specify the BMP3XX sensor address (0x76 or 0x77)
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  if (!lsm.begin_I2C(0x6A)) { // Specify the LSM6DSOX sensor address (0x6A or 0x6B)
    Serial.println("Could not find a valid LSM6DSOX sensor, check wiring!");
    while (1);
  }

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  Serial.print("Accelerometer sample rate = ");
  Serial.print(lsm.accelerationSampleRate());
  Serial.println(" Hz");

  // Attach servos
  servo1.attach(SERVO_PIN1);
  servo2.attach(SERVO_PIN2);

  // Set up MOSFET pins as outputs
  pinMode(MOSFET_1, OUTPUT);
  pinMode(MOSFET_2, OUTPUT);
  pinMode(MOSFET_3, OUTPUT);

  // Set up ADC
  analogReadResolution(12); // 12-bit ADC resolution
}

void loop() {
  // Get the current time
  unsigned long currentMillis = millis();

  // Read data at 100Hz
  if (currentMillis - previousMillis >= sensorInterval) {
    previousMillis = currentMillis;

    // Read sensor data
    readSensorData();

    // Update height data
    // Calculate height based on pressure data (you may need to adjust this)
    height = bmp.readAltitude(SEALEVELPRESSURE_HPA); // Height Algorithm  
  
    // Calculate heightAGL using the ground point
    float heightAGL = height - groundHeight1;

    // Set groundHeight on the first sample
    if (!groundHeightSet) {
      groundHeight1 = height;
      groundHeightSet = true;
    }

  }

  // Oversample the height and heightAGL
  if (currentMillis - previousOversampleMillis >= oversampleInterval) {
    previousOversampleMillis = currentMillis;
    
    float heightSum = 0.0;
    float heightAGLSum = 0.0;

    for (int i = 0; i < NUM_OVERSAMPLES; i++) {
      // Read sensor data and update height as before
      height = bmp.readAltitude(SEALEVELPRESSURE_HPA); // Height Algorithm
      // Update heightAGL using the ground point
      heightAGL = height - groundHeight1;

      // Accumulate height and heightAGL
      heightSum += height;
      heightAGLSum += heightAGL;
    }

    // Calculate the average oversampled values
    heightOversampled = heightSum / NUM_OVERSAMPLES;
    heightAGLOversampled = heightAGLSum / NUM_OVERSAMPLES;
    previousHeightOversampled = heightOversampled;
  }


  
  // Get the current time
  unsigned long currentMillis3 = millis();

  // Check if it's time to log telemetry data
  if (currentMillis3 - previousTelemetryMillis >= telemetryInterval) {
    previousTelemetryMillis = currentMillis3;

    // Log telemetry data here
    logTelemetryData();

    // Get ground station data
    XBeeRecieve();
  }
  
  // Update state based on height and sensor data
  updateState();

  // Control MOSFETs based on the current state
  controlMOSFETs();
 
  // Control servo motors based on state
  controlServos(); 

  // Other code based on states can be added here
}

void readSensorData() {
  // Read BMP390 sensor data
  pressure = bmp.readPressure() / 100.0F;  // Convert to hPa
  temperature = bmp.readTemperature();

  previousHeight = height;

  // Read LSM6DSOX sensor data =- gyro
  if (lsm.gyroscopeAvailable()) {
    lsm.readGyroscope(gyro_R, gyro_P, gyro_Y);
  }

  // Read LSM6DSOX sensor data - acceleration
  if (lsm.accelerationAvailable()) {
    lsm.readAcceleration(x, y, z);
  }

  // Read the raw ADC value
  int rawValue = analogRead(adcPin);

  // Calculate the voltage across the 3.3 kilo Ohm resistor
  float voltageAcrossR2 = (referenceVoltage * rawValue) / 4095.0;

  // Calculate the actual battery voltage using the voltage divider formula
  voltage = voltageAcrossR2 * ((r1 + r2) / r2);

}

void updateState() {
    // Update state variables based on height and other conditions
  if (heightAGL > 5 && (height - previousHeight) > 2 && groundHeightSet) {
    LAUNCH_READY = false;
    ASCENT = true;
    strcpy(swState, "ASCENT");
  }

  if (heightAGL >= 300 && ASCENT) {
    ASCENT = false;
    SEPARATE = true;
    strcpy(swState, "SEPARATE");
  }

  if (SEPARATE && (height - previousHeight) < -5) {
    SEPARATE = false;
    DESCENT = true;
    strcpy(swState, "DESCENT");
  }

  if (DESCENT && heightAGL < 10 && abs(height - previousHeight) < 0.2) {
    DESCENT = false;
    LANDED = true;
    strcpy(swState, "LANDED");
  }
}

void controlMOSFETs() {
  static unsigned long previousMillis1 = 0;
  const unsigned long mosfetInterval = 1000; // Timing interval in milliseconds

  // Get the current time
  unsigned long currentMillis1 = millis();

  if (LAUNCH_READY) {
    // Pulse MOSFET connected to GPIO 2 every second
    if (currentMillis1 - previousMillis1 >= mosfetInterval) {
      previousMillis1 = currentMillis1;
      static bool mosfetState = false;

      if (mosfetState) {
        digitalWrite(MOSFET_1, LOW);
        mosfetState = false;
      } else {
        digitalWrite(MOSFET_1, HIGH);
        mosfetState = true;
      }
    }
  }

  if (ASCENT) {
    // Pulse MOSFET connected to GPIO 2 twice every second
    if (currentMillis1 - previousMillis1 >= mosfetInterval / 2) {
      previousMillis1 = currentMillis1;
      static int pulseCount = 0;

      if (pulseCount < 2) {
        digitalWrite(MOSFET_1, HIGH);
        pulseCount++;
      } else {
        digitalWrite(MOSFET_1, LOW);
        pulseCount = 0;
      }
    }
  }

  if (SEPARATE) {
    // Activate MOSFET connected to GPIO 2
    digitalWrite(MOSFET_1, HIGH);
  }

  if (DESCENT) {
    // Make two short but distinct pulses every two seconds
    if (currentMillis1 - previousMillis1 >= mosfetInterval / 2) {
      previousMillis1 = currentMillis1;
      static bool mosfetState = false;

      if (mosfetState) {
        digitalWrite(MOSFET_1, LOW);
        mosfetState = false;
      } else {
        digitalWrite(MOSFET_1, HIGH);
        mosfetState = true;
      }
    }
  }

  if (LANDED) {
    // Keep MOSFET connected to GPIO 2 fully on
    digitalWrite(MOSFET_1, HIGH);
  }
}


void controlServos() {


  if (SEPARATE) {
    // Give power to MOSFET_2 for servo1
    digitalWrite(MOSFET_2, HIGH);

    // Code to rotate servo1 to 150 degrees
    if (servo1Angle < servo1MaxAngle) {
      if (millis() - servo1StartTime >= servoUpdateInterval) {
        servo1Angle++;
        servo1.write(servo1Angle);
        servo1StartTime = millis();
      }
    }
  } else {
    // Cut off power to MOSFET_2
    digitalWrite(MOSFET_2, LOW);
  }

  if (DESCENT && height <= 250) {
    // Give power to MOSFET_3 for servo2
    digitalWrite(MOSFET_3, HIGH);

    // Code to rotate servo2 to 270 degrees
    if (servo2Angle < servo2MaxAngle) {
      if (millis() - servo2StartTime >= servoUpdateInterval) {
        servo2Angle++;
        servo2.write(servo2Angle);
        servo2StartTime = millis();
      }
    }
    
    if (servo2Angle >= servo2MaxAngle) {
      // Wait for 1 second after rotation
      static unsigned long rotationCompleteTime = 0;
      if (millis() - rotationCompleteTime >= 1000) {
        // Return servo2 to 0 degrees
        if (servo2Angle > servo2MinAngle) {
          if (millis() - servo2StartTime >= servoUpdateInterval) {
            servo2Angle--;
            servo2.write(servo2Angle);
            servo2StartTime = millis();
          }
        }
      } else {
        // Keep servo2 powered during the 1-second delay
        digitalWrite(MOSFET_3, HIGH);
      }
    } else {
      // Cut off power to MOSFET_3
      digitalWrite(MOSFET_3, LOW);
    }
  }
}

void logTelemetryData() {
  // Increment packet count
  packetCount++;

  currentMillis4 = millis();
  // Get current time in format hh:mm:ss.ss
  unsigned int seconds = currentMillis4 / 1000;
  unsigned int minutes = seconds / 60;
  unsigned int hours = minutes / 60;
  unsigned int milliseconds = currentMillis4 % 100;
  
  char missionTime[12];
  sprintf(missionTime, "%02d:%02d:%02d.%02d", hours % 100, minutes % 60, seconds % 60, milliseconds);

  // Format and log telemetry data
  sprintf(csvBuffer, "1008,%s,%u,%s,%c,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
          missionTime, packetCount, swState, plState, heightAGLOversampled, temperature, voltage,
          gyro_R, gyro_P, gyro_Y, x, y, z, heightOversampled);

  // Log to serial monitor
  Serial.println(csvBuffer);

  // Log to OpenLog
  Serial1.println(csvBuffer);

  // Log to XBee
  Serial2.println(csvBuffer);
}

void XBeeRecieve() {
  unsigned long currentMillis6 = millis();
  unsigned long previousMillis6 = 0;
  unsigned long interval5 = 1000; // Interval for checking XBee data (1000 milliseconds)
  bool packetReceived = false;
  char receivedChar;

  // Check for incoming data from the XBee every 1000 milliseconds
  if (currentMillis6 - previousMillis6 >= interval5) {
    previousMillis6 = currentMillis6;

    // Check if there's data available from the XBee
    if (Serial2.available() > 0) {
      // Read the incoming data
      receivedChar = Serial2.read();
      
  
      // Check if the received character is 'r'
      if (receivedChar == 'r') {
        // Set the state back SEPARATE
        SEPARATE = true;
      }
      if (receivedChar == 'z') {
        // Reset ground level
        groundHeightSet = false;
      }   
      if (receivedChar == 'l') {
        // Reset to launch ready
        LAUNCH_READY = true;
      }
      if (receivedChar == 'p') {
        // Release parachute manually

        // Give power to MOSFET_3 for servo2
        digitalWrite(MOSFET_3, HIGH);

        // Code to rotate servo2 to 270 degrees
        if (servo2Angle < servo2MaxAngle) {
          if (millis() - servo2StartTime >= servoUpdateInterval) {
            servo2Angle++;
            servo2.write(servo2Angle);
            servo2StartTime = millis();
          }
        }
        
        if (servo2Angle >= servo2MaxAngle) {
          // Wait for 1 second after rotation
          static unsigned long rotationCompleteTime = 0;
          if (millis() - rotationCompleteTime >= 1000) {
            // Return servo2 to 0 degrees
            if (servo2Angle > servo2MinAngle) {
              if (millis() - servo2StartTime >= servoUpdateInterval) {
                servo2Angle--;
                servo2.write(servo2Angle);
                servo2StartTime = millis();
              }
            }
          } else {
            // Keep servo2 powered during the 1-second delay
            digitalWrite(MOSFET_3, HIGH);
          }
        } else {
          // Cut off power to MOSFET_3
          digitalWrite(MOSFET_3, LOW);
        }
      }
      
      
      // Process the received data here
      // You can add your own code to handle the received data
      
      // Set the packetReceived flag to true
      packetReceived = true;
    } else {
      // No data received within the interval
      packetReceived = false;
    }

      // Print whether or not a packet has been received
    if (packetReceived) {
      Serial.println(receivedChar);
    }   else {
      Serial.println("No Packet Received");
    }
    
  }

}
