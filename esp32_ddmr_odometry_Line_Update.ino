/*
 * Made by: @mochshultan
 * ESP32 DDMR Robot with FreeRTOS
 * Core 0: Odometry and Navigation System with TB6612FNG Motor Driver
 * Core 1: LED Blink
 * 
 * Motor Specs:
 * - Encoder: 2 phase (A & B)
 * - Wheel diameter: 68 mm
 * - Gearbox: JGA25
 * - Driver: TB6612FNG
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <EEPROM.h>

// ========== PIN DEFINITIONS ==========

// Line Sensor Pins
#define LINE_SENSOR_1 15  // D15
#define LINE_SENSOR_2 2   // D2
#define LINE_SENSOR_3 4   // D4

// ====== Stepper =======
#define IN1 14
#define IN2 23
#define IN3 19
#define IN4 18

int stepSpeed = 1000;
const int stepSequence[8][4] = {
  {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
  {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
};

// TB6612FNG Motor Driver Pins
#define MOTOR_LEFT_PWM 12   // PWMA - Left motor PWM B
#define MOTOR_LEFT_IN1 26   // AIN1 - Left motor direction 1
#define MOTOR_LEFT_IN2 27   // AIN2 - Left motor direction 2
#define MOTOR_RIGHT_PWM 16  // PWMB - Right motor PWM A
#define MOTOR_RIGHT_IN1 33  // BIN1 - Right motor direction 1
#define MOTOR_RIGHT_IN2 32  // BIN2 - Right motor direction 2
#define MOTOR_STBY 25       // STBY - Standby pin (HIGH to enable)

// Encoder Pins (Interrupt capable pins)
#define ENCODER_LEFT_A 35   // Left encoder phase A
#define ENCODER_LEFT_B 13   // Left encoder phase B
#define ENCODER_RIGHT_A 5   // Right encoder phase A
#define ENCODER_RIGHT_B 17  // Right encoder phase B

// LED Pin
#define LED_PIN 2

// ========== ROBOT PHYSICAL PARAMETERS ==========
#define WHEEL_DIAMETER 0.068  // 67mm in meters
#define WHEEL_CIRCUMFERENCE (PI * WHEEL_DIAMETER)
#define WHEELBASE 0.225  // Distance between wheels (adjust to your robot)

// JGA25 Gearbox Parameters
#define ENCODER_PPR 11    // Raw encoder PPR (before gearbox) - typical for JGA25
#define R_GEAR_RATIO 171  // JGA25-370 gear ratio (common variants: 120, 298, 495, 1000)
#define L_GEAR_RATIO 171  // JGA25-370 gear ratio (common variants: 120, 298, 495, 1000)

// Note: Different JGA25 variants have different gear ratios:
// JGA25-370: 120:1, 298:1, 495:1, 1000:1
// Check your motor specifications for exact ratio

#define EFFECTIVE_PPR_R (ENCODER_PPR * R_GEAR_RATIO)  // Total pulses per wheel revolution
#define EFFECTIVE_PPR_L (ENCODER_PPR * L_GEAR_RATIO)  // Total pulses per wheel revolution

// ========== GLOBAL VARIABLES ==========
// Encoder counters (volatile for interrupt access)
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Robot pose (position and orientation)
struct RobotPose {
  double x;      // X position in meters
  double y;      // Y position in meters
  double theta;  // Orientation in radians
};

RobotPose robotPose = { 0.0, 0.0, 0.0 };

// Previous encoder counts for odometry calculation
long prevLeftCount = 0;
long prevRightCount = 0;

// Mutex for protecting shared data
SemaphoreHandle_t poseMutex;

// Task handles
TaskHandle_t odometryTaskHandle = NULL;
TaskHandle_t ledTaskHandle = NULL;

// Navigation control variables
volatile bool navigationActive = false;
volatile double targetDistance = 0.0;
volatile double targetStop = 0.0;
volatile double targetAngle = 0.0;
volatile bool moveForward = false;
volatile bool turnRobot = false;

// PID parameters untuk gojek
double kp_pos = 2.0;    // Proportional gain untuk posisi
double ki_pos = 0.001;  // Integral gain untuk posisi
double kd_pos = 20.0;   // Derivative gain untuk posisi
double prev_error_pos = 0.0;
double integral_pos = 0.0;

// PID parameters untuk ramping down
double kp_dist = 60.0;    // Proportional gain untuk jarak
double ki_dist = 0.7;    // Integral gain untuk jarak
double kd_dist = 20.0;  // Derivative gain untuk jarak
double prev_error_dist = 0.0;
double integral_dist = 0.0;

double kp_angle = 2.0;    // Proportional gain untuk sudut
double ki_angle = 0.001;  // Integral gain untuk sudut
double kd_angle = 30.0;   // Derivative gain untuk sudut
double prev_error_angle = 0.0;
double integral_angle = 0.0;

// PID parameters untuk koreksi gyro yang smooth
double kp_gyro = 5.0;    // Proportional gain untuk koreksi gyro
double ki_gyro = 0.5;    // Integral gain untuk koreksi gyro
double kd_gyro = 100.0;  // Derivative gain untuk koreksi gyro
double prev_error_gyro = 0.0;
double integral_gyro = 0.0;

// Sensor variables
Adafruit_MPU6050 mpu;
volatile float gyroHeading = 0.0;
volatile unsigned long lastDriftCorrection = 0;
float homeHeading = 0.0;  // Heading home reference

volatile int lineSensorRaw[3] = { 0, 0, 0 };
volatile bool lineSensorDigital[3] = { 0, 0, 0 };
SemaphoreHandle_t sensorMutex;
int lineThreshold[3] = { 3500, 1500, 3500 };  // Nilai threshold untuk setiap sensor, di atas threshold == hitam == 1

// BLE variables
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
volatile bool deviceConnected = false;
String receivedMessage = "";
volatile bool newMessageReceived = false;

#define SERVICE_UUID "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-cba987654321"

// ========== BLE CALLBACKS ==========
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue().c_str();
    if (value.length() > 0) {
      receivedMessage = value;
      newMessageReceived = true;
    }
  }
};

// ========== INTERRUPT SERVICE ROUTINES ==========
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(ENCODER_LEFT_A) == digitalRead(ENCODER_LEFT_B)) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(ENCODER_RIGHT_A) == digitalRead(ENCODER_RIGHT_B)) {
    rightEncoderCount--;  // Reverse for right motor
  } else {
    rightEncoderCount++;
  }
}

// ========== MOTOR CONTROL FUNCTIONS ==========
void setupMotorDriver() {
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);

  // Setup LEDC for PWM (12-bit, 20kHz)
  ledcAttach(MOTOR_LEFT_PWM, 20000, 8);   // pin, freq, resolution
  ledcAttach(MOTOR_RIGHT_PWM, 20000, 8);

  digitalWrite(MOTOR_STBY, HIGH);  // Enable motor driver
}

void setMotorSpeed(int leftPWM, int rightPWM) {
  // Constrain speeds to -255 to 255
  leftPWM = constrain(leftPWM, -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

  // Left motor
  if (leftPWM >= 0) {
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
  } else {
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, HIGH);
    leftPWM = -leftPWM;
  }
  ledcWrite(MOTOR_LEFT_PWM, leftPWM);

  // Right motor
  if (rightPWM >= 0) {
    digitalWrite(MOTOR_RIGHT_IN1, HIGH);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
  } else {
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, HIGH);
    rightPWM = -rightPWM;
  }
  ledcWrite(MOTOR_RIGHT_PWM, rightPWM);
}

void stopMotors() {
  digitalWrite(MOTOR_STBY, LOW);
  setMotorSpeed(0, 0);
  digitalWrite(MOTOR_STBY, HIGH);
}

// ========== ODOMETRY FUNCTIONS ==========
void updateOdometry() {
  long currentLeftCount, currentRightCount;

  // Read encoder counts atomically
  noInterrupts();
  currentLeftCount = leftEncoderCount;
  currentRightCount = rightEncoderCount;
  interrupts();

  // Calculate distance traveled by each wheel
  long deltaLeft = currentLeftCount - prevLeftCount;
  long deltaRight = currentRightCount - prevRightCount;

  // Convert encoder counts to wheel distance considering gearbox ratio
  // Formula: distance = (encoder_pulses * wheel_circumference) / (encoder_ppr * gear_ratio)
  double leftDistance = (deltaLeft * WHEEL_CIRCUMFERENCE) / EFFECTIVE_PPR_L;
  double rightDistance = (deltaRight * WHEEL_CIRCUMFERENCE) / EFFECTIVE_PPR_R;

  // Calculate robot motion
  double deltaDistance = (leftDistance + rightDistance) / 2.0;
  double deltaTheta = (rightDistance - leftDistance) / WHEELBASE;

  // Update robot pose using differential drive kinematics
  if (xSemaphoreTake(poseMutex, portMAX_DELAY) == pdTRUE) {
    double newTheta = robotPose.theta + deltaTheta;

    // Normalize angle to (-PI, PI]
    while (newTheta > PI) newTheta -= 2 * PI;
    while (newTheta <= -PI) newTheta += 2 * PI;

    // Update position using average heading
    double avgTheta = (robotPose.theta + newTheta) / 2.0;
    robotPose.x += deltaDistance * cos(avgTheta);
    robotPose.y += deltaDistance * sin(avgTheta);
    robotPose.theta = newTheta;

    xSemaphoreGive(poseMutex);
  }

  // Update previous counts
  prevLeftCount = currentLeftCount;
  prevRightCount = currentRightCount;
}

void setHome() {
  // Set home position
  if (xSemaphoreTake(poseMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    robotPose.x = 0.0;
    robotPose.y = 0.0;
    robotPose.theta = 0.0;
    xSemaphoreGive(poseMutex);
  }

  // Set home heading dari gyro
  if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    homeHeading = gyroHeading;
    xSemaphoreGive(sensorMutex);
  }

  Serial.print("Home set - Position: (0,0,0°), Heading: ");
  Serial.print(homeHeading, 1);
  Serial.println("°");
}

// ========== NAVIGATION FUNCTIONS ==========
void maju(double jarak, bool rasis=false, bool follow=false) {
  vTaskDelay(pdMS_TO_TICKS(20));
  if (navigationActive) return;  // Mencegah navigasi bersamaan

  navigationActive = true;
  targetDistance = abs(jarak) * 0.95;  // Gunakan nilai absolut untuk kalkulasi jarak
  targetStop = (follow) ? targetDistance * 0.9 : targetDistance;
  bool isForward = (jarak >= 0);       // Tentukan arah
  moveForward = true;
  turnRobot = false;

  // Reset PID untuk ramping down
  prev_error_dist = 0.0;
  integral_dist = 0.0;

  double startX = robotPose.x;
  double startY = robotPose.y;

  // Baca heading awal dari gyro
  float startHeading;
  if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    startHeading = gyroHeading;
    xSemaphoreGive(sensorMutex);
  }

  Serial.print(isForward ? "Maju " : "Mundur ");
  Serial.print(targetDistance);
  Serial.print(" meter, heading awal: ");
  Serial.println(startHeading);

  bool allSensorsBlack = false;
  int tunggu = 0;
  float currentHeading = startHeading;  // Default ke startHeading
  while (moveForward && navigationActive) {
    double currentDistance = sqrt(pow(robotPose.x - startX, 2) + pow(robotPose.y - startY, 2));

    double remainingDistance = targetStop - currentDistance;

    if (rasis && allSensorsBlack) {
      remainingDistance=0;
      tunggu = 50;
    }

    if (remainingDistance <= 0.005) {  // Toleransi 5mm
      vTaskDelay(pdMS_TO_TICKS(tunggu));  // 100 Hz navigation loop
      stopMotors();
      moveForward = false;
      navigationActive = false;
      break;
    }

    // PID control untuk ramping down yang smooth
    double error_dist = remainingDistance;
    integral_dist += error_dist;
    integral_dist = constrain(integral_dist, -500, ceil(abs(jarak))*75);
    double derivative_dist = error_dist - prev_error_dist;
    prev_error_dist = error_dist;

    double speed_output = kp_dist * error_dist + ki_dist * integral_dist + kd_dist * derivative_dist;
    int speed = constrain((int)speed_output, 25, 225);

    // Koreksi heading menggunakan gyro
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      currentHeading = gyroHeading;
      allSensorsBlack = (lineSensorDigital[0] && lineSensorDigital[1] && lineSensorDigital[2]);
      xSemaphoreGive(sensorMutex);
    }

    float headingError = startHeading - currentHeading;
    // Normalisasi ke rentang -180 s.d. 180
    if (headingError > 180.0f) headingError -= 360.0f;
    if (headingError < -180.0f) headingError += 360.0f;
    Serial.println(headingError);
    int correction = (int)(headingError * 15);

    // Terapkan arah dan koreksi heading
    if (isForward) {
      setMotorSpeed(speed + correction, speed - correction);  // Maju dengan koreksi
    } else {
      setMotorSpeed(-speed + correction, -speed - correction);  // Mundur dengan koreksi terbalik
    }

    vTaskDelay(pdMS_TO_TICKS(5));  // 100 Hz navigation loop
  }

  int maxCorrections = 500;
  int consecutiveSmallErrors = 0;

  for (int i = 0; i < maxCorrections; i++) {
    float currentHeading;
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      currentHeading = gyroHeading;
      xSemaphoreGive(sensorMutex);
    }

    float errorHeading = startHeading - currentHeading;

    // Normalisasi error
    if (errorHeading > 180) {
      errorHeading -= 360;
    } else if (errorHeading <= -180) {
      errorHeading += 360;
    }

    // Anti-jiggle
    if (abs(errorHeading) < 0.15) {
      consecutiveSmallErrors++;
      if (consecutiveSmallErrors >= 3) {
        Serial.print("✓ Gyro correction selesai | Final error: ");
        Serial.print(errorHeading, 2);
        Serial.println("°");
        break;
      }
    } else {
      consecutiveSmallErrors = 0;
    }

    // PID
    double error_gyro = errorHeading;
    integral_gyro += error_gyro;
    integral_gyro = constrain(integral_gyro, -20, 20);

    double derivative_gyro = error_gyro - prev_error_gyro;
    prev_error_gyro = error_gyro;

    double correction_output = kp_gyro * error_gyro + ki_gyro * integral_gyro + kd_gyro * derivative_gyro;
    int correctionSpeed = constrain(abs((int)correction_output), 40, 60);

    // GYRO: errorHeading < 0 → heading perlu naik → CCW
    if (errorHeading < 0) {
      setMotorSpeed(-correctionSpeed, correctionSpeed);
    } else {
      setMotorSpeed(correctionSpeed, -correctionSpeed);
    }

    vTaskDelay(pdMS_TO_TICKS(15));
    stopMotors();
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  if (!follow){
    stopMotors();
    navigationActive = false;
  } else {
    // Line following mechanism
    Serial.println("Starting line following...");
    
    // Phase 1: Find black line with sweeping
    bool lineFound = false;
    int sweepCount = 0;
    bool sweepRight = true;
    
    while (!lineFound && sweepCount < 2) {
      if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (lineSensorDigital[0] || lineSensorDigital[1] || lineSensorDigital[2]) {
          lineFound = true;
        }
        xSemaphoreGive(sensorMutex);
      }
      
      if (!lineFound) {
        const int sweepSpeed = 90;   // Kecepatan motor saat sweep
        const int sweepDuration = 1500; // Durasi tiap gerakan (ms), sesuaikan dengan robot
        
        // Sweep kanan
        setMotorSpeed(sweepSpeed, 10);
        for (int i = 0; i < sweepDuration/50 && !lineFound; i++) {
          vTaskDelay(pdMS_TO_TICKS(50));
          if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (lineSensorDigital[0] || lineSensorDigital[1] || lineSensorDigital[2]) {
              lineFound = true;
            }
            xSemaphoreGive(sensorMutex);
          }
        }
        if (lineFound) break;
        
        // Balik kanan
        setMotorSpeed(-sweepSpeed, -10);
        for (int i = 0; i < sweepDuration/50 && !lineFound; i++) {
          vTaskDelay(pdMS_TO_TICKS(50));
          if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (lineSensorDigital[0] || lineSensorDigital[1] || lineSensorDigital[2]) {
              lineFound = true;
            }
            xSemaphoreGive(sensorMutex);
          }
        }
        if (lineFound) break;

        // Sweep kiri
        setMotorSpeed(10, sweepSpeed);
        for (int i = 0; i < sweepDuration/50 && !lineFound; i++) {
          vTaskDelay(pdMS_TO_TICKS(50));
          if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (lineSensorDigital[0] || lineSensorDigital[1] || lineSensorDigital[2]) {
              lineFound = true;
            }
            xSemaphoreGive(sensorMutex);
          }
        }
        if (lineFound) break;
        
        // Balik kiri
        setMotorSpeed(-10, -sweepSpeed);
        for (int i = 0; i < sweepDuration/50 && !lineFound; i++) {
          vTaskDelay(pdMS_TO_TICKS(50));
          if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (lineSensorDigital[0] || lineSensorDigital[1] || lineSensorDigital[2]) {
              lineFound = true;
            }
            xSemaphoreGive(sensorMutex);
          }
        }
        if (lineFound) break;
        
        sweepCount++;
      }
    }
    
    if (!lineFound) {
      Serial.println("Line not found after sweeping, force follow.");
      stopMotors();
    }

    // Phase 2: Center on line and follow while maintaining heading
    Serial.println("Line found, centering...");
    int consecutiveCenter = 0;
    unsigned long followStart = millis();
    
    while (consecutiveCenter < 20 && (millis() - followStart < 10000)) {
      bool sensor[3];
      float currentHeading;
      
      if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        sensor[0] = lineSensorDigital[0];
        sensor[1] = lineSensorDigital[1];
        sensor[2] = lineSensorDigital[2];
        currentHeading = gyroHeading;
        xSemaphoreGive(sensorMutex);
      }
      
      // Line following logic
      int baseSpeed = 30;
      int leftSpeed = baseSpeed;
      int rightSpeed = baseSpeed;
      
      // Line position correction
      if (sensor[1]) { // Center sensor on line
        consecutiveCenter++;
      } else {
        consecutiveCenter = 0;
        if (sensor[0] && !sensor[2]) { // Line on left
          leftSpeed = 120;
          rightSpeed = 0;
        } else if (sensor[2] && !sensor[0]) { // Line on right
          leftSpeed = 10;
          rightSpeed = 120;
        } else { // No line detected
          leftSpeed = baseSpeed;
          rightSpeed = baseSpeed;
        }
      }
      
      setMotorSpeed(leftSpeed, rightSpeed);
      vTaskDelay(pdMS_TO_TICKS(5));
    }
    
    stopMotors();

    // Correct heading after line follow
    for (int i = 0; i < 100; i++) {
      float currentHeading;
      if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        currentHeading = gyroHeading;
        xSemaphoreGive(sensorMutex);
      }

      float errorHeading = homeHeading - currentHeading;

      // Normalisasi error
    if (errorHeading > 90) {
      errorHeading -= 180;
    } else if (errorHeading < -90) {
      errorHeading += 180;
    }

      // Anti-jiggle
      if (abs(errorHeading) < 0.15) {
        consecutiveSmallErrors++;
        if (consecutiveSmallErrors >= 3) {
          Serial.print("✓ Gyro correction selesai | Final error: ");
          Serial.print(errorHeading, 2);
          Serial.println("°");
          break;
        }
      } else {
        consecutiveSmallErrors = 0;
      }

      // PID
      double error_gyro = errorHeading;
      integral_gyro += error_gyro;
      integral_gyro = constrain(integral_gyro, -25, 25);

      double derivative_gyro = error_gyro - prev_error_gyro;
      prev_error_gyro = error_gyro;

      double correction_output = kp_gyro * error_gyro + ki_gyro * integral_gyro + kd_gyro * derivative_gyro;
      int correctionSpeed = constrain(abs((int)correction_output), 30, 50);

      // GYRO: errorHeading < 0 → heading perlu naik → CCW
      if (errorHeading < 0) {
        setMotorSpeed(-correctionSpeed, correctionSpeed);
      } else {
        setMotorSpeed(correctionSpeed, -correctionSpeed);
      }

      vTaskDelay(pdMS_TO_TICKS(20));
      stopMotors();
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Maju sisa
    navigationActive = true;
    moveForward = true;
    while (moveForward && navigationActive) {
    double currentDistance = sqrt(pow(robotPose.x - startX, 2) + pow(robotPose.y - startY, 2));

    double remainingDistance = targetDistance - currentDistance;

    if (rasis && allSensorsBlack) {
      remainingDistance=0;
      tunggu = 50;
    }

    if (remainingDistance <= 0.005) {  // Toleransi 5mm
      vTaskDelay(pdMS_TO_TICKS(tunggu));  // 100 Hz navigation loop
      stopMotors();
      moveForward = false;
      navigationActive = false;
      break;
    }

    // PID control untuk ramping down yang smooth
    double error_dist = remainingDistance;
    integral_dist += error_dist;
    integral_dist = constrain(integral_dist, -500, ceil(abs(jarak))*75);
    double derivative_dist = error_dist - prev_error_dist;
    prev_error_dist = error_dist;

    double speed_output = kp_dist * error_dist + ki_dist * integral_dist + kd_dist * derivative_dist;
    int speed = constrain((int)speed_output, 25, 225);

    // Koreksi heading menggunakan gyro
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      currentHeading = gyroHeading;
      allSensorsBlack = (lineSensorDigital[0] && lineSensorDigital[1] && lineSensorDigital[2]);
      xSemaphoreGive(sensorMutex);
    }

    float headingError = startHeading - currentHeading;
    // Normalisasi ke rentang -180 s.d. 180
    if (headingError > 180.0f) headingError -= 360.0f;
    if (headingError < -180.0f) headingError += 360.0f;
    Serial.println(headingError);
    int correction = (int)(headingError * 15);

    // Terapkan arah dan koreksi heading
    if (isForward) {
      setMotorSpeed(speed + correction, speed - correction);  // Maju dengan koreksi
    } else {
      setMotorSpeed(-speed + correction, -speed - correction);  // Mundur dengan koreksi terbalik
    }

    vTaskDelay(pdMS_TO_TICKS(5));  // 100 Hz navigation loop
  }

  stopMotors();
  vTaskDelay(pdMS_TO_TICKS(10));

  int maxCorrections = 100;
  int consecutiveSmallErrors = 0;

  for (int i = 0; i < maxCorrections; i++) {
    float currentHeading;
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      currentHeading = gyroHeading;
      xSemaphoreGive(sensorMutex);
    }

    float errorHeading = startHeading - currentHeading;

    // Normalisasi error
    if (errorHeading > 180) {
      errorHeading -= 360;
    } else if (errorHeading <= -180) {
      errorHeading += 360;
    }

    // Anti-jiggle
    if (abs(errorHeading) < 0.15) {
      consecutiveSmallErrors++;
      if (consecutiveSmallErrors >= 3) {
        Serial.print("✓ Gyro correction selesai | Final error: ");
        Serial.print(errorHeading, 2);
        Serial.println("°");
        break;
      }
    } else {
      consecutiveSmallErrors = 0;
    }

    // PID
    double error_gyro = errorHeading;
    integral_gyro += error_gyro;
    integral_gyro = constrain(integral_gyro, -20, 20);

    double derivative_gyro = error_gyro - prev_error_gyro;
    prev_error_gyro = error_gyro;

    double correction_output = kp_gyro * error_gyro + ki_gyro * integral_gyro + kd_gyro * derivative_gyro;
    int correctionSpeed = constrain(abs((int)correction_output), 30, 50);

    // GYRO: errorHeading < 0 → heading perlu naik → CCW
    if (errorHeading < 0) {
      setMotorSpeed(-correctionSpeed, correctionSpeed);
    } else {
      setMotorSpeed(correctionSpeed, -correctionSpeed);
    }

    vTaskDelay(pdMS_TO_TICKS(15));
    stopMotors();
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  stopMotors();
  navigationActive = false;
  Serial.println("Line following completed");
  }
}

void belok(double derajat) {
  vTaskDelay(pdMS_TO_TICKS(10));
  if (navigationActive) return;

  navigationActive = true;
  turnRobot = true;
  moveForward = false;

  prev_error_angle = 0.0;
  integral_angle = 0.0;
  integral_gyro = 0.0;
  prev_error_gyro = 0.0;

  double startTheta = robotPose.theta * 180.0 / PI;  // Konversi ke DERAJAT
  double targetTheta = startTheta + derajat;         // LANGSUNG pakai derajat!
  float startHeading;

  if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    startHeading = gyroHeading;
    xSemaphoreGive(sensorMutex);
  }

  // Normalisasi target angle ke (-180, 180]
  while (targetTheta > 180) targetTheta -= 360;
  while (targetTheta <= -180) targetTheta += 360;

  Serial.print("=== Belok ");
  Serial.print(derajat);
  Serial.println(" derajat ===");
  Serial.print("Start theta: ");
  Serial.print(startTheta, 2);
  Serial.print("° → Target theta: ");
  Serial.print(targetTheta, 2);
  Serial.println("°");
  Serial.print("Start heading: ");
  Serial.print(startHeading, 2);
  Serial.println("°");

  // FASE 1: Belok dengan odometry
  unsigned long lastPrint = millis();
  while (turnRobot && navigationActive) {

    double currentTheta = robotPose.theta * 180.0 / PI;  // DERAJAT
    double remainingAngle = targetTheta - currentTheta;  // DERAJAT

    // Normalisasi remaining angle
    if (remainingAngle > 180) remainingAngle -= 360;
    if (remainingAngle <= -180) remainingAngle += 360;

    // Debug setiap 200ms
    if (millis() - lastPrint > 200) {
      Serial.print("Current theta: ");
      Serial.print(currentTheta, 2);
      Serial.print("° | Remaining: ");
      Serial.print(remainingAngle, 2);
      Serial.println("°");
      lastPrint = millis();
    }

    if (abs(remainingAngle) < 0.5) {  // 2 derajat toleransi
      stopMotors();
      turnRobot = false;
      Serial.println("✓ Fase odometry selesai");
      break;
    }

    double error_angle = remainingAngle;
    integral_angle += error_angle;
    integral_angle = constrain(integral_angle, -20, 15);
    double derivative_angle = error_angle - prev_error_angle;
    prev_error_angle = error_angle;

    double turn_output = kp_angle * abs(error_angle) + ki_angle * integral_angle + kd_angle * abs(derivative_angle);
    int turnSpeed = constrain((int)turn_output, 30, 120);

    // remainingAngle > 0 → theta perlu naik → CCW (kiri)
    if (remainingAngle > 0) {
      setMotorSpeed(-turnSpeed, turnSpeed);  // CCW
    } else {
      setMotorSpeed(turnSpeed, -turnSpeed);  // CW
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
  stopMotors();

  // FASE 2: Koreksi dengan gyro
  vTaskDelay(pdMS_TO_TICKS(150));

  // Gyro berlawanan: CW → heading turun
  // belok(+45) CW → theta +45° → heading -45°
  float targetHeading = startHeading - derajat;  // INVERSI

  // Normalisasi target heading
  while (targetHeading > 180) targetHeading -= 360;
  while (targetHeading <= -180) targetHeading += 360;

  float currentHeadingNow;
  if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    currentHeadingNow = gyroHeading;
    xSemaphoreGive(sensorMutex);
  }

  Serial.print("Gyro correction | Target: ");
  Serial.print(targetHeading, 2);
  Serial.print("° | Current: ");
  Serial.print(currentHeadingNow, 2);
  Serial.println("°");

  int maxCorrections = 1000;
  int consecutiveSmallErrors = 0;

  for (int i = 0; i < maxCorrections; i++) {
    float currentHeading;
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      currentHeading = gyroHeading;
      xSemaphoreGive(sensorMutex);
    }

    float errorHeading = targetHeading - currentHeading;

    // Normalisasi error
    if (errorHeading > 180) {
      errorHeading -= 360;
    } else if (errorHeading <= -180) {
      errorHeading += 360;
    }

    // Anti-jiggle
    if (abs(errorHeading) < 0.15) {
      consecutiveSmallErrors++;
      if (consecutiveSmallErrors >= 3) {
        Serial.print("✓ Gyro correction selesai | Final error: ");
        Serial.print(errorHeading, 2);
        Serial.println("°");
        break;
      }
    } else {
      consecutiveSmallErrors = 0;
    }

    // PID
    double error_gyro = errorHeading;
    integral_gyro += error_gyro;
    integral_gyro = constrain(integral_gyro, -20, 20);

    double derivative_gyro = error_gyro - prev_error_gyro;

    double correction_output = kp_gyro * error_gyro + ki_gyro * integral_gyro + kd_gyro * derivative_gyro;
    int correctionSpeed = constrain(abs((int)correction_output),30, 60);

    prev_error_gyro = error_gyro;

    Serial.print("#");
    Serial.print(i + 1);
    Serial.print(" Heading: ");
    Serial.print(currentHeading, 2);
    Serial.print("° | Error: ");
    Serial.print(errorHeading, 2);
    Serial.print("° | Speed: ");
    Serial.print(correctionSpeed);
    Serial.print(" | ");

    // GYRO: errorHeading < 0 → heading perlu naik → CCW
    if (errorHeading < 0) {
      Serial.println("CCW ↺");
      setMotorSpeed(-correctionSpeed, correctionSpeed);
    } else {
      Serial.println("CW ↻");
      setMotorSpeed(correctionSpeed, -correctionSpeed);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
    stopMotors();
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  stopMotors();
  navigationActive = false;

  float finalHeading;
  double finalTheta = robotPose.theta * 180.0 / PI;  // DERAJAT
  if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    finalHeading = gyroHeading;
    xSemaphoreGive(sensorMutex);
  }

  Serial.println("==================");
  Serial.print("Final theta: ");
  Serial.print(finalTheta, 2);
  Serial.print("° (target ");
  Serial.print(targetTheta, 2);
  Serial.print("°, error ");
  Serial.print(targetTheta - finalTheta, 2);
  Serial.println("°)");
  Serial.print("Final heading: ");
  Serial.print(finalHeading, 2);
  Serial.print("° (target ");
  Serial.print(targetHeading, 2);
  Serial.print("°, error ");
  Serial.print(targetHeading - finalHeading, 2);
  Serial.println("°)");
  Serial.println("==================");
  vTaskDelay(pdMS_TO_TICKS(500));
}
void pivot(double derajat) {
  vTaskDelay(pdMS_TO_TICKS(10));
  if (navigationActive) return;

  navigationActive = true;
  turnRobot = true;
  moveForward = false;

  prev_error_angle = 0.0;
  integral_angle = 0.0;
  integral_gyro = 0.0;
  prev_error_gyro = 0.0;

  double startTheta = robotPose.theta * 180.0 / PI;  // Konversi ke DERAJAT
  double targetTheta = startTheta + derajat;         // LANGSUNG pakai derajat!
  float startHeading;

  if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    startHeading = gyroHeading;
    xSemaphoreGive(sensorMutex);
  }

  // Normalisasi target angle ke (-180, 180]
  while (targetTheta > 180) targetTheta -= 360;
  while (targetTheta <= -180) targetTheta += 360;

  // Serial.print("=== Belok ");
  // Serial.print(derajat);
  // Serial.println(" derajat ===");
  // Serial.print("Start theta: ");
  // Serial.print(startTheta, 2);
  // Serial.print("° → Target theta: ");
  // Serial.print(targetTheta, 2);
  // Serial.println("°");
  // Serial.print("Start heading: ");
  // Serial.print(startHeading, 2);
  // Serial.println("°");

  // FASE 1: Belok dengan odometry
  unsigned long lastPrint = millis();
  while (turnRobot && navigationActive) {

    double currentTheta = robotPose.theta * 180.0 / PI;  // DERAJAT
    double remainingAngle = targetTheta - currentTheta;  // DERAJAT

    // Normalisasi remaining angle
    if (remainingAngle > 180) remainingAngle -= 360;
    if (remainingAngle <= -180) remainingAngle += 360;

    // Debug setiap 200ms
    if (millis() - lastPrint > 200) {
      Serial.print("Current theta: ");
      Serial.print(currentTheta, 2);
      Serial.print("° | Remaining: ");
      Serial.print(remainingAngle, 2);
      Serial.println("°");
      lastPrint = millis();
    }

    if (abs(remainingAngle) < 0.5) {  // 2 derajat toleransi
      stopMotors();
      turnRobot = false;
      Serial.println("✓ Fase odometry selesai");
      break;
    }

    double error_angle = remainingAngle;
    integral_angle += error_angle;
    integral_angle = constrain(integral_angle, -20, 15);
    double derivative_angle = error_angle - prev_error_angle;
    prev_error_angle = error_angle;

    double turn_output = kp_angle * abs(error_angle) + ki_angle * integral_angle + kd_angle * abs(derivative_angle);
    int turnSpeed = constrain((int)turn_output, 30, 120);

    // remainingAngle > 0 → theta perlu naik → CCW (kiri)
    if (remainingAngle > 0) {
      setMotorSpeed(0, turnSpeed);  // CCW
    } else {
      setMotorSpeed(turnSpeed, 0);  // CW
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
  stopMotors();

  // FASE 2: Koreksi dengan gyro
  vTaskDelay(pdMS_TO_TICKS(150));

  // Gyro berlawanan: CW → heading turun
  // belok(+45) CW → theta +45° → heading -45°
  float targetHeading = startHeading - derajat;  // INVERSI

  // Normalisasi target heading
  while (targetHeading > 180) targetHeading -= 360;
  while (targetHeading <= -180) targetHeading += 360;

  float currentHeadingNow;
  if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    currentHeadingNow = gyroHeading;
    xSemaphoreGive(sensorMutex);
  }

  Serial.print("Gyro correction | Target: ");
  Serial.print(targetHeading, 2);
  Serial.print("° | Current: ");
  Serial.print(currentHeadingNow, 2);
  Serial.println("°");

  int maxCorrections = 1000;
  int consecutiveSmallErrors = 0;

  for (int i = 0; i < maxCorrections; i++) {
    float currentHeading;
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      currentHeading = gyroHeading;
      xSemaphoreGive(sensorMutex);
    }

    float errorHeading = targetHeading - currentHeading;

    // Normalisasi error
    if (errorHeading > 180) {
      errorHeading -= 360;
    } else if (errorHeading <= -180) {
      errorHeading += 360;
    }

    // Anti-jiggle
    if (abs(errorHeading) < 0.15) {
      consecutiveSmallErrors++;
      if (consecutiveSmallErrors >= 3) {
        Serial.print("✓ Gyro correction selesai | Final error: ");
        Serial.print(errorHeading, 2);
        Serial.println("°");
        break;
      }
    } else {
      consecutiveSmallErrors = 0;
    }

    // PID
    double error_gyro = errorHeading;
    integral_gyro += error_gyro;
    integral_gyro = constrain(integral_gyro, -20, 20);

    double derivative_gyro = error_gyro - prev_error_gyro;

    double correction_output = kp_gyro * error_gyro + ki_gyro * integral_gyro + kd_gyro * derivative_gyro;
    int correctionSpeed = constrain(abs((int)correction_output),30, 60);

    prev_error_gyro = error_gyro;

    Serial.print("#");
    Serial.print(i + 1);
    Serial.print(" Heading: ");
    Serial.print(currentHeading, 2);
    Serial.print("° | Error: ");
    Serial.print(errorHeading, 2);
    Serial.print("° | Speed: ");
    Serial.print(correctionSpeed);
    Serial.print(" | ");

    // GYRO: errorHeading < 0 → heading perlu naik → CCW
    if (errorHeading < 0) {
      Serial.println("CCW ↺");
      setMotorSpeed(-correctionSpeed, correctionSpeed);
    } else {
      Serial.println("CW ↻");
      setMotorSpeed(correctionSpeed, -correctionSpeed);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
    stopMotors();
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  stopMotors();
  navigationActive = false;

  float finalHeading;
  double finalTheta = robotPose.theta * 180.0 / PI;  // DERAJAT
  if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    finalHeading = gyroHeading;
    xSemaphoreGive(sensorMutex);
  }

  Serial.println("==================");
  Serial.print("Final theta: ");
  Serial.print(finalTheta, 2);
  Serial.print("° (target ");
  Serial.print(targetTheta, 2);
  Serial.print("°, error ");
  Serial.print(targetTheta - finalTheta, 2);
  Serial.println("°)");
  Serial.print("Final heading: ");
  Serial.print(finalHeading, 2);
  Serial.print("° (target ");
  Serial.print(targetHeading, 2);
  Serial.print("°, error ");
  Serial.print(targetHeading - finalHeading, 2);
  Serial.println("°)");
  Serial.println("==================");
  vTaskDelay(pdMS_TO_TICKS(500));
}
// Fungsi gojek - navigasi ke pose target dengan 3 tahap
void gojek(double targetX, double targetY, double targetTheta) {
  if (navigationActive) return;  // Mencegah navigasi bersamaan

  // Ambil posisi saat ini dari robotPose
  double currentX, currentY, currentTheta;
  if (xSemaphoreTake(poseMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    currentX = robotPose.x;
    currentY = robotPose.y;
    currentTheta = robotPose.theta;
    xSemaphoreGive(poseMutex);
  }

  // Simpan heading awal saat gojek dipanggil
  float initialHeading;
  if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    initialHeading = gyroHeading;
    xSemaphoreGive(sensorMutex);
  }

  Serial.print("Gojek dari pose: (");
  Serial.print(currentX, 2);
  Serial.print(", ");
  Serial.print(currentY, 2);
  Serial.print(", ");
  Serial.print(currentTheta * 180.0 / PI, 1);
  Serial.print("°) ke pose: (");
  Serial.print(targetX, 2);
  Serial.print(", ");
  Serial.print(targetY, 2);
  Serial.print(", ");
  Serial.print(targetTheta, 1);
  Serial.println("°)");

  // TAHAP 1: Hitung delta posisi dan sudut ke target berdasarkan robotPose
  double deltaX = targetX - currentX;
  double deltaY = targetY - currentY;
  double magnitude = sqrt(deltaX * deltaX + deltaY * deltaY);
  double angleToTarget = atan2(-deltaY, -deltaX) * 180.0 / PI;  // Konversi ke derajat, frame BLD

  Serial.print("Magnitude: ");
  Serial.print(magnitude, 2);
  Serial.print("m, Sudut: ");
  Serial.print(angleToTarget, 1);
  Serial.println("°");

  // TAHAP 2: Belok ke arah target
  double currentThetaDeg = initialHeading;
  double angleDiff = angleToTarget - currentThetaDeg;

  // Normalisasi ke (-180, 180]
  while (angleDiff > 180) angleDiff -= 360;
  while (angleDiff <= -180) angleDiff += 360;

  if (abs(angleDiff) > 0.1) {  // Skip jika sudah hampir tepat
    Serial.print("Belok relatif: ");
    Serial.print(angleDiff, 1);
    Serial.print("° (dari ");
    Serial.print(currentThetaDeg, 1);
    Serial.print("° ke ");
    Serial.print(angleToTarget, 1);
    Serial.println("°)");
    belok(angleDiff);
    vTaskDelay(pdMS_TO_TICKS(500));  // Delay sebentar
  } else {
    Serial.println("Skip belok, sudah menghadap target");
  }

  // TAHAP 3: Maju sejauh magnitude
  if (magnitude > 0.001) {  // Toleransi 0.1 cm
    Serial.print("Maju sejauh: ");
    Serial.print(magnitude, 2);
    Serial.println("m");
    maju(magnitude);
    vTaskDelay(pdMS_TO_TICKS(500));  // Delay sebentar
  }

  // TAHAP 4: Koreksi heading final dengan PID
  double finalTargetHeading = homeHeading + targetTheta;
  while (finalTargetHeading > 180) finalTargetHeading -= 360;
  while (finalTargetHeading <= -180) finalTargetHeading += 360;

  Serial.print("Koreksi heading final ke: ");
  Serial.print(finalTargetHeading, 1);
  Serial.println("°");

  integral_gyro = 0.0;
  prev_error_gyro = 0.0;
  int consecutiveSmallErrors = 0;

  while (true) {
    float currentHeading;
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      currentHeading = gyroHeading;
      xSemaphoreGive(sensorMutex);
    }

    float errorHeading = finalTargetHeading - currentHeading;
    if (errorHeading > 180) errorHeading -= 360;
    else if (errorHeading <= -180) errorHeading += 360;

    if (abs(errorHeading) < 0.15) {
      consecutiveSmallErrors++;
      if (consecutiveSmallErrors >= 3) {
        Serial.print("✓ Heading final OK | Error: ");
        Serial.print(errorHeading, 2);
        Serial.println("°");
        break;
      }
    } else {
      consecutiveSmallErrors = 0;
    }

    // PID
    double error_gyro = errorHeading;
    integral_gyro += error_gyro;
    integral_gyro = constrain(integral_gyro, -20, 20);

    double derivative_gyro = error_gyro - prev_error_gyro;
    prev_error_gyro = error_gyro;

    double correction_output = kp_gyro * error_gyro + ki_gyro * integral_gyro + kd_gyro * derivative_gyro;
    int correctionSpeed = constrain(abs((int)correction_output), 40, 70);

    if (errorHeading < 0) {
      setMotorSpeed(-correctionSpeed, correctionSpeed);
    } else {
      setMotorSpeed(correctionSpeed, -correctionSpeed);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
    stopMotors();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  stopMotors();

  Serial.println("Gojek selesai!");
}

// Fungsi Stepper Motor
void putarStepper(int jumlahPutaran, int arah) {
  int totalStep = 4096 * jumlahPutaran;  // 1 putaran = 4096 step

  if (arah > 0) {  // searah jarum jam
    for (int step = 0; step < totalStep; step++) {
      stepMotor(step % 8);
    vTaskDelay(pdMS_TO_TICKS(stepSpeed/1000)); // Sesuaikan kecepatan: 1ms ≈ 300–500 RPM tergantung motor
    }
  } else {  // berlawanan arah jarum jam
    for (int step = totalStep; step > 0; step--) {
      stepMotor(step % 8);
    vTaskDelay(pdMS_TO_TICKS(stepSpeed/1000)); // Sesuaikan kecepatan: 1ms ≈ 300–500 RPM tergantung motor
    }
  }
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  return;
}

void stepMotor(int stepIndex) {
  digitalWrite(IN1, stepSequence[stepIndex][0]);
  digitalWrite(IN2, stepSequence[stepIndex][1]);
  digitalWrite(IN3, stepSequence[stepIndex][2]);
  digitalWrite(IN4, stepSequence[stepIndex][3]);
}

void saveThresholdToEEPROM() {
  for (int i = 0; i < 3; i++) {
    EEPROM.writeInt(i * 4, lineThreshold[i]);
  }
  EEPROM.commit();
  Serial.println("Threshold disimpan ke EEPROM");
}

void loadThresholdFromEEPROM() {
  for (int i = 0; i < 3; i++) {
    lineThreshold[i] = EEPROM.readInt(i * 4);
  }
  Serial.println("Threshold dimuat dari EEPROM:");
  Serial.print("Sensor 1: ");
  Serial.println(lineThreshold[0]);
  Serial.print("Sensor 2: ");
  Serial.println(lineThreshold[1]);
  Serial.print("Sensor 3: ");
  Serial.println(lineThreshold[2]);
}

// bool checkLineFound() {
//   if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
//     bool found = (lineSensorDigital[0] || lineSensorDigital[1] || lineSensorDigital[2]);
//     xSemaphoreGive(sensorMutex);
//     return found;
//   }
//   return false;
// }

// void sweepForLine() {
//   Serial.println("Mencari garis...");
//   bool lineFound = false;
//   int sweepCount = 0;
//   const int sweepSpeed = 70;
//   const int sweepSteps = 60; // 60 x 50ms = 3 detik

//   while (!lineFound && sweepCount < 2) {
//     // Sweep kiri
//     setMotorSpeed(0, sweepSpeed);
//     for (int i = 0; i < sweepSteps && !lineFound; i++) {
//       vTaskDelay(pdMS_TO_TICKS(50));
//       lineFound = checkLineFound();
//     }
//     if (lineFound) break;

//     // Balik kiri
//     setMotorSpeed(0, -sweepSpeed);
//     for (int i = 0; i < sweepSteps && !lineFound; i++) {
//       vTaskDelay(pdMS_TO_TICKS(50));
//       lineFound = checkLineFound();
//     }
//     if (lineFound) break;

//     // Sweep kanan
//     setMotorSpeed(sweepSpeed, 0);
//     for (int i = 0; i < sweepSteps && !lineFound; i++) {
//       vTaskDelay(pdMS_TO_TICKS(50));
//       lineFound = checkLineFound();
//     }
//     if (lineFound) break;

//     // Balik kanan
//     setMotorSpeed(-sweepSpeed, 0);
//     for (int i = 0; i < sweepSteps && !lineFound; i++) {
//       vTaskDelay(pdMS_TO_TICKS(50));
//       lineFound = checkLineFound();
//     }
    
//     sweepCount++;
//   }

//   stopMotors();
//   if (lineFound) {
//     Serial.println("Garis ditemukan!");
//   } else {
//     Serial.println("Garis tidak ditemukan setelah sweep");
//   }
// }

void kalibrasiLineSensor(){
  Serial.println("Kalibrasi line sensor dimulai...");
  Serial.println("Letakkan robot di atas ungu selama 5 detik");
  
  int kalibrasiDurasi = 2000;  // 1 detik
  unsigned long startTime = millis();

  int maxValues[3] = {0, 0, 0};

  while (millis() - startTime < kalibrasiDurasi) {
    setMotorSpeed(45, 48);
    int sensor1 = analogRead(LINE_SENSOR_1);
    int sensor2 = analogRead(LINE_SENSOR_2);
    int sensor3 = analogRead(LINE_SENSOR_3);

    if (sensor1 > maxValues[0]) maxValues[0] = sensor1;
    if (sensor2 > maxValues[1]) maxValues[1] = sensor2;
    if (sensor3 > maxValues[2]) maxValues[2] = sensor3;

    vTaskDelay(pdMS_TO_TICKS(50));
  }
  setMotorSpeed(0, 0);
  startTime = millis();

  while (millis() - startTime < kalibrasiDurasi) {
    setMotorSpeed(-45, -48);
    int sensor1 = analogRead(LINE_SENSOR_1);
    int sensor2 = analogRead(LINE_SENSOR_2);
    int sensor3 = analogRead(LINE_SENSOR_3);

    if (sensor1 > maxValues[0]) maxValues[0] = sensor1;
    if (sensor2 > maxValues[1]) maxValues[1] = sensor2;
    if (sensor3 > maxValues[2]) maxValues[2] = sensor3;

    vTaskDelay(pdMS_TO_TICKS(50));
  }

  setMotorSpeed(0, 0);

  // Hitung threshold = nilai ungu tertinggi + 100
  for (int i = 0; i < 3; i++) {
    lineThreshold[i] = maxValues[i] - 200;
  }

  saveThresholdToEEPROM();

  Serial.println("Kalibrasi selesai. Threshold line sensor:");
  Serial.print("Sensor 1: ");
  Serial.println(lineThreshold[0]);
  Serial.print("Sensor 2: ");
  Serial.println(lineThreshold[1]);
  Serial.print("Sensor 3: ");
  Serial.println(lineThreshold[2]);
}

// ========== FREERTOS TASKS ==========
void odometryTask(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);  // 100Hz update rate

  Serial.println("Odometry task started on Core 0");

  while (true) {
    updateOdometry();

    // Print robot pose every 1 second
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
      if (xSemaphoreTake(poseMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        Serial.print("Pose - X: ");
        Serial.print(robotPose.x, 3);
        Serial.print("m, Y: ");
        Serial.print(robotPose.y, 3);
        Serial.print("m, Theta: ");
        Serial.print(robotPose.theta * 180.0 / PI, 1);
        Serial.println("°");
        xSemaphoreGive(poseMutex);
      }
      lastPrint = millis();
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void ledTask(void *parameter) {
  Serial.println("Sensor IMU &  task dimulai di Core 1");

  // Inisialisasi MPU6050, cek scl sda apa sdh nyambung
  if (!mpu.begin()) {
    Serial.println("Gagal menginisialisasi MPU6050!");
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }

  // Konfigurasi MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);  // ini dipakai
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 berhasil diinisialisasi");
  // Inisialisasi
  float gyroZ_offset = 0;
  float heading = 0;
  unsigned long lastTime = millis();

  // Kalibrasi gyro (ambil offset)
  Serial.println("Kalibrasi gyro, jangan gerakkan robot...");
  for (int i = 0; i < 200; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroZ_offset += g.gyro.z;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  gyroZ_offset /= 200;
  Serial.print("Offset gyro Z: ");
  Serial.println(gyroZ_offset);

  while (true) {  // memulai task utama
    // Baca line sensor analog
    int line1_raw = analogRead(LINE_SENSOR_1);
    int line2_raw = analogRead(LINE_SENSOR_2);
    int line3_raw = analogRead(LINE_SENSOR_3);

    // Konversi ke digital berdasarkan threshold
    bool line1_digital = (line1_raw > lineThreshold[0]) ? 1 : 0;
    bool line2_digital = (line2_raw > lineThreshold[1]) ? 1 : 0;
    bool line3_digital = (line3_raw > lineThreshold[2]) ? 1 : 0;

    // Baca gyro dan hitung heading
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;  // detik
    lastTime = currentTime;

    // Integrasikan gyro Z untuk mendapat heading dengan dead zone
    float gyroZ_corrected = g.gyro.z - gyroZ_offset;

    // Dead zone untuk mengurangi drift saat diam
    if (abs(gyroZ_corrected) < 0.02) {  // Dead zone 0.02 rad/s
      gyroZ_corrected = 0.0;
    }
    heading += gyroZ_corrected * dt * 180.0 / PI;  // konversi ke derajat

    // Normalisasi heading ke range (-180, 180]
    while (heading > 180) heading -= 360;
    while (heading <= -180) heading += 360;

    // Update variabel global dengan mutex
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      lineSensorRaw[0] = line1_raw;
      lineSensorRaw[1] = line2_raw;
      lineSensorRaw[2] = line3_raw;
      lineSensorDigital[0] = line1_digital;
      lineSensorDigital[1] = line2_digital;
      lineSensorDigital[2] = line3_digital;
      gyroHeading = heading;
      xSemaphoreGive(sensorMutex);
    }

    // Tampilkan data sensor setiap 500ms
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
      Serial.print("Line Raw: ");
      Serial.print(line1_raw);
      Serial.print("-");
      Serial.print(line2_raw);
      Serial.print("-");
      Serial.print(line3_raw);
      Serial.print(" | Digital: ");
      Serial.print(line1_digital);
      Serial.print("-");
      Serial.print(line2_digital);
      Serial.print("-");
      Serial.print(line3_digital);
      Serial.print(" | Heading: ");
      Serial.print(heading, 1);
      Serial.println("°");
      lastPrint = millis();
    }

    // LED blink
    static bool ledState = false;
    static unsigned long lastLedToggle = 0;
    if (millis() - lastLedToggle > 250) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastLedToggle = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(20));  // 50Hz update rate
  }
}

void misiKanan() {
  maju(0.18,0,0);
  belok(-90);
  maju(0.28,1,0);
  belok(90);

  for (int i = 0; i < 5; i++) {
    maju(1.7,0,1);
    putarStepper(3, -1);
    belok(90);
    belok(90);    
    maju(1.7,0,1);
    vTaskDelay(pdMS_TO_TICKS(50));
    putarStepper(3, 1);
    maju(-0.15,0,0);
    if (i == 4) break;
    belok(90);
    maju(0.15,0,0);
    belok(90);
    maju(-0.15,0,0);
  }

  belok(90);
  belok(90);
  //ke kotak finish
  maju(1.0,0,0);
  belok(90);
  maju(1.0,0,0);
  belok(-90);
  maju(0.8,0,0);
  belok(180);
}

void misiKiri() {
  maju(0.18,0,0);
  belok(90);
  maju(0.28,1,0);
  belok(-90);

  for (int i = 0; i < 5; i++) {
    maju(1.7,0,1);
    putarStepper(3, -1);
    belok(-90);
    belok(-90);
    maju(1.7,0,1);
    vTaskDelay(pdMS_TO_TICKS(50));
    putarStepper(3, 1);
    maju(-0.15,0,0);
    if (i == 4) break;
    pivot(90);
    pivot(90);
    pivot(90);
  }

  belok(90);
  belok(90);
  //ke kotak finish
  maju(0.8,0,0);
  belok(-90);
  maju(0.9,0,0);
  belok(90);
  maju(0.6,0,0);
  belok(180);
}

// ========== SETUP AND LOOP ==========
void setup() {
  Serial.begin(115200);
  vTaskDelay(pdMS_TO_TICKS(1000));

  Serial.println("ESP32 DDMR Robot with FreeRTOS Starting...");

  // Initialize EEPROM
  EEPROM.begin(512);
  loadThresholdFromEEPROM();

  // Initialize hardware
  setupMotorDriver();
  pinMode(LED_PIN, OUTPUT);

  // Setup encoders
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

  // Setup line sensor
  pinMode(LINE_SENSOR_1, INPUT);
  pinMode(LINE_SENSOR_2, INPUT);
  pinMode(LINE_SENSOR_3, INPUT);

  // Setup stepper
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Inisialisasi I2C untuk MPU6050
  Wire.begin();

  // Initialize BLE
  BLEDevice::init("ESP32_Robot");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("BLE Server started, waiting for connections...");

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR, CHANGE);

  // Create mutex
  poseMutex = xSemaphoreCreateMutex();
  sensorMutex = xSemaphoreCreateMutex();

  // Create tasks
  xTaskCreatePinnedToCore(
    odometryTask,         // Task function
    "OdometryTask",       // Task name
    4096,                 // Stack size
    NULL,                 // Parameter
    2,                    // Priority
    &odometryTaskHandle,  // Task handle
    0                     // Core 0
  );

  xTaskCreatePinnedToCore(
    ledTask,         // Task function
    "SensorTask",    // Task name
    4096,            // Stack size (lebih besar untuk sensor)
    NULL,            // Parameter
    1,               // Priority
    &ledTaskHandle,  // Task handle
    1                // Core 1
  );

  Serial.println("Setup completed. Robot ready!");
  Serial.println("Motor Configuration:");
  Serial.print("- Wheel diameter: ");
  Serial.print(WHEEL_DIAMETER * 1000);
  Serial.println("mm");
  // Serial.print("- Gear ratio: ");
  // Serial.print(GEAR_RATIO);
  Serial.println(":1");
  // Serial.print("- Effective PPR: ");
  // Serial.println(EFFECTIVE_PPR);
  Serial.println("\nAvailable commands:");
  Serial.println("- 'maju <jarak_meter>' : Move forward");
  Serial.println("- 'belok <derajat>' : Turn (positive = left, negative = right)");
  Serial.println("- 'gojek <x> <y> <theta>' : Go to pose (x,y in meters, theta in degrees)");
  Serial.println("- 'stop' : Stop motors");
  Serial.println("- 'reset' : Reset pose to origin");
  Serial.println("- 'info' : Show robot configuration");
  vTaskDelay(pdMS_TO_TICKS(2000));

  // Set home position dan heading
  setHome();
}

void loop() {
  // Handle BLE messages
  // if (newMessageReceived) {
  //   String command = String(receivedMessage.c_str());
  //   command.trim();
  //   newMessageReceived = false;

  //   Serial.print("BLE Command received: ");
  //   Serial.println(command);

  //   if (command == "R" || command == "r") {
  //     Serial.println("Starting misiKanan...");
  //     misiKanan();
  //   } else if (command == "L" || command == "l") {
  //     Serial.println("Starting misiKiri...");
      misiKiri();
  //   } else if (command == "K" || command == "k") {
  //     Serial.println("Starting kalibrasi line sensor...");
  //     kalibrasiLineSensor();
  //   } else if (command.toFloat() != 0.0) {
  //     double jarak = command.toFloat();
  //     Serial.print("Maju jarak: ");
  //     Serial.println(jarak);
  //     maju(jarak);
  //   } else if (command == "up") {
  //     putarStepper(1, -1);
  //   } else if (command == "down") {
  //     putarStepper(1, 1);
  //   } else if (command == "tes") {
  //     maju(0.1, false, true);
  //     maju(0.1,0,0);
  //   } else {
  //     Serial.println("Unknown BLE command");
  //   }
  // }

  // // Handle disconnection
  // if (!deviceConnected) {
  //   vTaskDelay(pdMS_TO_TICKS(500));
  //   pServer->startAdvertising();
  //   Serial.println("Start advertising");
  // }

  vTaskDelay(pdMS_TO_TICKS(100));
}
