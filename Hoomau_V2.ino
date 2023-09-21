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

// Servo control variables
const int servo1MinAngle = 0;
const int servo1MaxAngle = 150;
const int servo2MinAngle = 0;
const int servo2MaxAngle = 270;
int servo1Angle = servo1MinAngle;
int servo2Angle = servo2MinAngle;

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

  //lsm6dsox.setAccelRange(LSM6DSOX_ACCEL_RANGE_2_G);
  //lsm6dsox.setGyroRange(LSM6DSOX_GYRO_RANGE_250_DPS);

  // Attach servos
  servo1.attach(SERVO_PIN1);
  servo2.attach(SERVO_PIN2);

  // Set up MOSFET pins as outputs
  pinMode(MOSFET_1, OUTPUT);
  pinMode(MOSFET_2, OUTPUT);
  pinMode(MOSFET_3, OUTPUT);
}

void loop() {
  // Read sensor data
  readSensorData();

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

  // Read LSM6DSOX sensor data
  sensors_event_t accel, gyro;
  //lsm.getEvent(&accel);
  //lsm.getEvent(&gyro);

  //gyro_R = gyro.gyro.x;
  //gyro_P = gyro.gyro.y;
  //gyro_Y = gyro.gyro.z;
}

void updateState() {
  // Calculate height based on pressure data (you may need to adjust this)
  height = (1.0 - pow((pressure / 1013.25), 0.1903)) * 44330.0; // Height Algorithm

  // Print the height
  Serial.print("Height: ");
  Serial.print(height);
  Serial.println(" meters");  

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

  // Store current height for comparison
  previousHeight = height;
}

void controlMOSFETs() {
  if (LAUNCH_READY) {
    // Code to pulse MOSFET connected to GPIO 2 every second
    digitalWrite(MOSFET_1, HIGH);
    delay(500);
    digitalWrite(MOSFET_1, LOW);
    delay(1500);
  }

  if (ASCENT) {
    // Code to pulse MOSFET connected to GPIO 2 twice every second
    digitalWrite(MOSFET_1, HIGH);
    delay(250);
    digitalWrite(MOSFET_1, LOW);
    delay(250);
    digitalWrite(MOSFET_1, HIGH);
    delay(250);
    digitalWrite(MOSFET_1, LOW);
    delay(250);
  }

  if (SEPARATE) {
    // Code to activate MOSFET connected to GPIO 2
    digitalWrite(MOSFET_1, HIGH);
  }

  if (DESCENT) {
    // Code to make two short but distinct pulses every two seconds on MOSFET connected to GPIO 2
    digitalWrite(MOSFET_1, HIGH);
    delay(100);
    digitalWrite(MOSFET_1, LOW);
    delay(100);
    digitalWrite(MOSFET_1, HIGH);
    delay(100);
    digitalWrite(MOSFET_1, LOW);
    delay(1700);
  }

  if (LANDED) {
    // Code to keep MOSFET connected to GPIO 2 fully on
    digitalWrite(MOSFET_1, HIGH);
  }
}

void controlServos() {
  if (SEPARATE) {
    // Give power to MOSFET_2 for servo1
    digitalWrite(MOSFET_2, HIGH);

    // Code to rotate servo1 to 150 degrees
    if (servo1Angle < servo1MaxAngle) {
      servo1Angle++;
      servo1.write(servo1Angle);
      delay(10); // Adjust delay for servo speed
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
      servo2Angle++;
      servo2.write(servo2Angle);
      delay(10); // Adjust delay for servo speed
    }
    delay(1000); // Wait after rotation

    // Return servo2 to 0 degrees
    while (servo2Angle > servo2MinAngle) {
      servo2Angle--;
      servo2.write(servo2Angle);
      delay(10); // Adjust delay for servo speed
    }

    // Cut off power to MOSFET_3
    digitalWrite(MOSFET_3, LOW);
  }
}