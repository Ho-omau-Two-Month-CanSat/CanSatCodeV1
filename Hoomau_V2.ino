#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LSM6DSOX.h>
#include <Servo.h>

// Define pins for sensors and devices
#define OPENLOG_RX 0
#define OPENLOG_TX 1
#define XBEE_RX 8
#define XBEE_TX 9
#define SERVO_PIN1 16
#define SERVO_PIN2 17
#define FEEDBACK_PIN 27
#define MOSFET_1 2
#define MOSFET_2 18
#define MOSFET_3 19

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

// Servo control variables
const int servo1MinAngle = 0;
const int servo1MaxAngle = 150;
const int servo2MinAngle = 0;
const int servo2MaxAngle = 270;
int servo1Angle = servo1MinAngle;
int servo2Angle = servo2MinAngle;

//non-blocking timing
unsigned long previousMillis = 0;
const unsigned long heightUpdateInterval = 100;  // Update height every 100 milliseconds

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

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

  Serial.print("Accelerometer sample rate = ");
  Serial.print(lsm.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");

  // Attach servos
  servo1.attach(SERVO_PIN1);
  servo2.attach(SERVO_PIN2);

  // Set up MOSFET pins as outputs
  pinMode(MOSFET_1, OUTPUT);
  pinMode(MOSFET_2, OUTPUT);
  pinMode(MOSFET_3, OUTPUT);
}

void loop() {
  // Get the current time
  unsigned long currentMillis = millis();

  // Read sensor data
  readSensorData();

  // Update height data at a higher rate
  if (currentMillis - previousMillis >= heightUpdateInterval) {
    previousMillis = currentMillis;

    // Update height data
    // Calculate height based on pressure data (you may need to adjust this)
    height = (1.0 - pow((pressure / 1013.25), 0.1903)) * 44330.0; // Height Algorithm  

    // Store current height for comparison
    previousHeight = height;  

    // Print the height
    Serial.print("Height: ");
    Serial.print(height);
    Serial.println(" meters");

    // Print the Orientation
    Serial.print("Orientation:   ");
    Serial.print(gyro_R);
    Serial.print('\t');
    Serial.print(gyro_P);
    Serial.print('\t');
    Serial.println(gyro_Y);

    //Print the acceleration
    Serial.print("Acceleration:   ");
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
  
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

  // Read LSM6DSOX sensor data =- gyro
  if (lsm.gyroscopeAvailable()) {
    lsm.readGyroscope(gyro_R, gyro_P, gyro_Y);
  }

  // Read LSM6DSOX sensor data - acceleration
  if (lsm.accelerationAvailable()) {
    lsm.readAcceleration(x, y, z);
  }
}

void updateState() {
    // Update state variables based on height and other conditions
  if (height > 0 && abs(height - previousHeight) > 0.5) {
    LAUNCH_READY = false;
    ASCENT = true;
  }

  if (height >= 500) {
    ASCENT = false;
    SEPARATE = true;
  }

  if (SEPARATE && abs(height - previousHeight) < 5) {
    SEPARATE = false;
    DESCENT = true;
  }

  if (DESCENT && height < 10 && abs(height - previousHeight) < 0.1) {
    DESCENT = false;
    LANDED = true;
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
  static unsigned long servo1StartTime = 0;
  static unsigned long servo2StartTime = 0;
  const unsigned long servoUpdateInterval = 10; // Adjust for servo speed

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
