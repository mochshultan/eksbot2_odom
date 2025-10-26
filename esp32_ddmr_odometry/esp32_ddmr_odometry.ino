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

// ========== PIN DEFINITIONS ==========

// Line Sensor Pins
#define LINE_SENSOR_1     15    // D15
#define LINE_SENSOR_2     2     // D2 
#define LINE_SENSOR_3     4     // D4

// ====== Stepper =======
#define IN1 14
#define IN2 23
#define IN3 19
#define IN4 18

int stepSpeed = 900;
int stepSequence[8][4] = {
  {1,0,0,0}, {1,1,0,0},
  {0,1,0,0}, {0,1,1,0},
  {0,0,1,0}, {0,0,1,1},
  {0,0,0,1}, {1,0,0,1}
};

// TB6612FNG Motor Driver Pins
#define MOTOR_LEFT_PWM    12    // PWMA - Left motor PWM B
#define MOTOR_LEFT_IN1    26    // AIN1 - Left motor direction 1
#define MOTOR_LEFT_IN2    27    // AIN2 - Left motor direction 2
#define MOTOR_RIGHT_PWM   16    // PWMB - Right motor PWM A
#define MOTOR_RIGHT_IN1   33    // BIN1 - Right motor direction 1
#define MOTOR_RIGHT_IN2   32    // BIN2 - Right motor direction 2
#define MOTOR_STBY        25    // STBY - Standby pin (HIGH to enable)

// Encoder Pins (Interrupt capable pins)
#define ENCODER_LEFT_A    35    // Left encoder phase A
#define ENCODER_LEFT_B    13    // Left encoder phase B
#define ENCODER_RIGHT_A   5    // Right encoder phase A
#define ENCODER_RIGHT_B   17    // Right encoder phase B

// LED Pin
#define LED_PIN           2

// ========== ROBOT PHYSICAL PARAMETERS ==========
#define WHEEL_DIAMETER    0.068   // 67mm in meters
#define WHEEL_CIRCUMFERENCE (PI * WHEEL_DIAMETER)
#define WHEELBASE         0.225    // Distance between wheels (adjust to your robot)

// JGA25 Gearbox Parameters
#define ENCODER_PPR       11      // Raw encoder PPR (before gearbox) - typical for JGA25
#define R_GEAR_RATIO        171     // JGA25-370 gear ratio (common variants: 120, 298, 495, 1000)
#define L_GEAR_RATIO        171     // JGA25-370 gear ratio (common variants: 120, 298, 495, 1000)

// Note: Different JGA25 variants have different gear ratios:
// JGA25-370: 120:1, 298:1, 495:1, 1000:1
// Check your motor specifications for exact ratio

#define EFFECTIVE_PPR_R     (ENCODER_PPR * R_GEAR_RATIO)  // Total pulses per wheel revolution
#define EFFECTIVE_PPR_L     (ENCODER_PPR * L_GEAR_RATIO)  // Total pulses per wheel revolution

// ========== GLOBAL VARIABLES ==========
// Encoder counters (volatile for interrupt access)
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Robot pose (position and orientation)
struct RobotPose {
  double x;        // X position in meters
  double y;        // Y position in meters
  double theta;    // Orientation in radians
};

RobotPose robotPose = {0.0, 0.0, 0.0};

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
volatile double targetAngle = 0.0;
volatile bool moveForward = false;
volatile bool turnRobot = false;

// PID parameters untuk gojek
double kp_pos = 2.0;  // Proportional gain untuk posisi
double ki_pos = 0.001;    // Integral gain untuk posisi
double kd_pos = 20.0;   // Derivative gain untuk posisi
double prev_error_pos = 0.0;
double integral_pos = 0.0;

// PID parameters untuk ramping down
double kp_dist = 75.0; // Proportional gain untuk jarak
double ki_dist = 0.75;  // Integral gain untuk jarak
double kd_dist = 1200.0;  // Derivative gain untuk jarak
double prev_error_dist = 0.0;
double integral_dist = 0.0;

double kp_angle = 2.0; // Proportional gain untuk sudut
double ki_angle = 0.001;   // Integral gain untuk sudut
double kd_angle = 30.0;  // Derivative gain untuk sudut
double prev_error_angle = 0.0;
double integral_angle = 0.0;

// PID parameters untuk koreksi gyro yang smooth
double kp_gyro = 5.0;   // Proportional gain untuk koreksi gyro
double ki_gyro = 0.5;    // Integral gain untuk koreksi gyro
double kd_gyro = 100.0;    // Derivative gain untuk koreksi gyro
double prev_error_gyro = 0.0;
double integral_gyro = 0.0;

// Sensor variables
Adafruit_MPU6050 mpu;
volatile float gyroHeading = 0.0;
volatile unsigned long lastDriftCorrection = 0;
float homeHeading = 0.0; // Heading home reference

volatile int lineSensorRaw[3] = {0, 0, 0};
volatile int lineSensorDigital[3] = {0, 0, 0};
SemaphoreHandle_t sensorMutex;
int lineThreshold[3] = {850, 750, 1650}; // Nilai threshold untuk setiap sensor, di atas threshold == hitam == 1

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
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  
  digitalWrite(MOTOR_STBY, HIGH); // Enable motor driver
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Constrain speeds to -255 to 255
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  // Left motor control
  if (leftSpeed >= 0) {
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
  } else {
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(MOTOR_LEFT_PWM, leftSpeed);
  
  // Right motor control
  if (rightSpeed >= 0) {
    digitalWrite(MOTOR_RIGHT_IN1, HIGH);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
  } else {
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(MOTOR_RIGHT_PWM, rightSpeed);
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
void maju(double jarak) {
  vTaskDelay(pdMS_TO_TICKS(250));
  if (navigationActive) return; // Mencegah navigasi bersamaan
  
  navigationActive = true;
  targetDistance = abs(jarak)*0.98; // Gunakan nilai absolut untuk kalkulasi jarak
  bool isForward = (jarak >= 0); // Tentukan arah
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
    startHeading = gyroHeading ;
    xSemaphoreGive(sensorMutex);
  }
  
  Serial.print(isForward ? "Maju " : "Mundur ");
  Serial.print(targetDistance);
  Serial.print(" meter, heading awal: ");
  Serial.println(startHeading);

  bool allSensorsBlack = false;  
  float currentHeading = startHeading; // Default ke startHeading
  while (moveForward && navigationActive) {
    double currentDistance = sqrt(pow(robotPose.x - startX, 2) + pow(robotPose.y - startY, 2));
    
    double remainingDistance = targetDistance - currentDistance;
    
    if (remainingDistance <= 0.005 || allSensorsBlack) { // Toleransi 5mm
      stopMotors();
      moveForward = false;
      navigationActive = false;
      break;
    }
    
    // PID control untuk ramping down yang smooth
    double error_dist = remainingDistance;
    integral_dist += error_dist;
    integral_dist = constrain(integral_dist, -200, 200);
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
    if (headingError > 180.0f)  headingError -= 360.0f;
    if (headingError < -180.0f) headingError += 360.0f;
    Serial.println(headingError);
    int correction = (int)(headingError * 15);

    // Terapkan arah dan koreksi heading
    if (isForward) {
      setMotorSpeed(speed + correction, speed - correction);   // Maju dengan koreksi
    } else {
      setMotorSpeed(-speed + correction, -speed - correction); // Mundur dengan koreksi terbalik
    }
    
    vTaskDelay(pdMS_TO_TICKS(20)); // 200Hz navigation loop
  }
  
  stopMotors();

  vTaskDelay(pdMS_TO_TICKS(50));

  int maxCorrections = 1000;
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
    if (abs(errorHeading) < 0.2) {
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
    
    vTaskDelay(pdMS_TO_TICKS(10));
    stopMotors();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  stopMotors();
  navigationActive = false;
}

void belok(double derajat) {
  vTaskDelay(pdMS_TO_TICKS(250));
  if (navigationActive) return;
  
  navigationActive = true;
  turnRobot = true;
  moveForward = false;
  
  prev_error_angle = 0.0;
  integral_angle = 0.0;
  integral_gyro = 0.0;
  prev_error_gyro = 0.0;
  
  double startTheta = robotPose.theta * 180.0 / PI; // Konversi ke DERAJAT
  double targetTheta = startTheta + derajat; // LANGSUNG pakai derajat!
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
    
    double currentTheta = robotPose.theta * 180.0 / PI; // DERAJAT
    double remainingAngle = targetTheta - currentTheta; // DERAJAT
    
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
    
    if (abs(remainingAngle) < 1) { // 2 derajat toleransi
      stopMotors();
      turnRobot = false;
      Serial.println("✓ Fase odometry selesai");
      break;
    }
    
    double error_angle = remainingAngle;
    integral_angle += error_angle;
    integral_angle = constrain(integral_angle, -10, 10);
    double derivative_angle = error_angle - prev_error_angle;
    prev_error_angle = error_angle;
    
    double turn_output = kp_angle * abs(error_angle) + ki_angle * integral_angle + kd_angle * abs(derivative_angle);
    int turnSpeed = constrain((int)turn_output, 20, 120);
        
    // remainingAngle > 0 → theta perlu naik → CCW (kiri)
    if (remainingAngle > 0) {
      setMotorSpeed(-turnSpeed, turnSpeed); // CCW
    } else {
      setMotorSpeed(turnSpeed, -turnSpeed); // CW
    }
    
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  stopMotors();

  // FASE 2: Koreksi dengan gyro
  vTaskDelay(pdMS_TO_TICKS(150));
  
  // Gyro berlawanan: CW → heading turun
  // belok(+45) CW → theta +45° → heading -45°
  float targetHeading = startHeading - derajat; // INVERSI
  
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
    if (abs(errorHeading) < 0.21) {
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
    int correctionSpeed = constrain(abs((int)correction_output), 40, 50);
    
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
  double finalTheta = robotPose.theta * 180.0 / PI; // DERAJAT
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
  if (navigationActive) return; // Mencegah navigasi bersamaan
  
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
  double angleToTarget = atan2(-deltaY, -deltaX) * 180.0 / PI; // Konversi ke derajat, frame BLD
  
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
  
  if (abs(angleDiff) > 0.1) { // Skip jika sudah hampir tepat
    Serial.print("Belok relatif: ");
    Serial.print(angleDiff, 1);
    Serial.print("° (dari ");
    Serial.print(currentThetaDeg, 1);
    Serial.print("° ke ");
    Serial.print(angleToTarget, 1);
    Serial.println("°)");
    belok(angleDiff);
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay sebentar
  } else {
    Serial.println("Skip belok, sudah menghadap target");
  }
  
  // TAHAP 3: Maju sejauh magnitude
  if (magnitude > 0.001) { // Toleransi 0.1 cm
    Serial.print("Maju sejauh: ");
    Serial.print(magnitude, 2);
    Serial.println("m");
    maju(magnitude);
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay sebentar
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
    
    if (abs(errorHeading) < 0.2) {
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
  int totalStep = 4096 * jumlahPutaran; // 1 putaran = 4096 step

  if (arah > 0) { // searah jarum jam
    for (int step = 0; step < totalStep; step++) {
      stepMotor(step % 8);
      delayMicroseconds(stepSpeed);
    }
  } else { // berlawanan arah jarum jam
    for (int step = totalStep; step > 0; step--) {
      stepMotor(step % 8);
      delayMicroseconds(stepSpeed);
    }
  }
}
void stepMotor(int stepIndex) {
  digitalWrite(IN1, stepSequence[stepIndex][0]);
  digitalWrite(IN2, stepSequence[stepIndex][1]);
  digitalWrite(IN3, stepSequence[stepIndex][2]);
  digitalWrite(IN4, stepSequence[stepIndex][3]);
}
// ========== FREERTOS TASKS ==========
void odometryTask(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz update rate
  
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
  mpu.setGyroRange(MPU6050_RANGE_250_DEG); // ini dipakai
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  
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
  
  while (true) { // memulai task utama
    // Baca line sensor analog
    int line1_raw = analogRead(LINE_SENSOR_1);
    int line2_raw = analogRead(LINE_SENSOR_2);
    int line3_raw = analogRead(LINE_SENSOR_3);
    
    // Konversi ke digital berdasarkan threshold
    int line1_digital = (line1_raw > lineThreshold[0]) ? 1 : 0;
    int line2_digital = (line2_raw > lineThreshold[1]) ? 1 : 0;
    int line3_digital = (line3_raw > lineThreshold[2]) ? 1 : 0;
    
    // Baca gyro dan hitung heading
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; // detik
    lastTime = currentTime;
    
    // Integrasikan gyro Z untuk mendapat heading dengan dead zone
    float gyroZ_corrected = g.gyro.z - gyroZ_offset;
    
    // Dead zone untuk mengurangi drift saat diam
    if (abs(gyroZ_corrected) < 0.01) { // Dead zone 0.01 rad/s
      gyroZ_corrected = 0.0;
    }
    heading += gyroZ_corrected * dt * 180.0 / PI; // konversi ke derajat
    
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
    
    vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz update rate
  }
}
void taskstorage(){
  maju(1.65);
  putarStepper(3,-1);
  maju(-0.15);
  belok(90);
  belok(90);
  maju(1.5);
  vTaskDelay(pdMS_TO_TICKS(50));
  putarStepper(2,1);
  maju(-0.15);
  belok(90);
  maju(0.125);
  belok(90);
  maju(-0.15);
}
// ========== SETUP AND LOOP ==========
void setup() {
  Serial.begin(115200);
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  Serial.println("ESP32 DDMR Robot with FreeRTOS Starting...");
  
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
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR, CHANGE);
  
  // Create mutex
  poseMutex = xSemaphoreCreateMutex();
  sensorMutex = xSemaphoreCreateMutex();
  
  // Create tasks
  xTaskCreatePinnedToCore(
    odometryTask,       // Task function
    "OdometryTask",     // Task name
    4096,               // Stack size
    NULL,               // Parameter
    2,                  // Priority
    &odometryTaskHandle,// Task handle
    0                   // Core 0
  );
  
  xTaskCreatePinnedToCore(
    ledTask,            // Task function
    "SensorTask",       // Task name
    4096,               // Stack size (lebih besar untuk sensor)
    NULL,               // Parameter
    1,                  // Priority
    &ledTaskHandle,     // Task handle
    1                   // Core 1
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
  // Handle serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "info") {
      Serial.println("\n=== Robot Configuration ===");
      Serial.print("Wheel diameter: ");
      Serial.print(WHEEL_DIAMETER * 1000);
      Serial.println(" mm");
      Serial.print("Wheel circumference: ");
      Serial.print(WHEEL_CIRCUMFERENCE * 1000, 1);
      Serial.println(" mm");
      Serial.print("Wheelbase: ");
      Serial.print(WHEELBASE * 1000);
      Serial.println(" mm");
      Serial.print("Encoder PPR (raw): ");
      Serial.println(ENCODER_PPR);
      // Serial.print("Gear ratio: ");
      // Serial.print(GEAR_RATIO);
      Serial.println(":1");
      // Serial.print("Effective PPR: ");
      // Serial.println(EFFECTIVE_PPR);
      // Serial.print("Distance per pulse: ");
      // Serial.print((WHEEL_CIRCUMFERENCE / EFFECTIVE_PPR) * 1000000, 2);
      // Serial.println(" micrometers");
      
      if (xSemaphoreTake(poseMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        Serial.println("\n=== Current State ===");
        Serial.print("Position X: ");
        Serial.print(robotPose.x, 4);
        Serial.println(" m");
        Serial.print("Position Y: ");
        Serial.print(robotPose.y, 4);
        Serial.println(" m");
        Serial.print("Orientation: ");
        Serial.print(robotPose.theta * 180.0 / PI, 2);
        Serial.println(" degrees");
        
        noInterrupts();
        long leftCount = leftEncoderCount;
        long rightCount = rightEncoderCount;
        interrupts();
        
        Serial.print("Left encoder: ");
        Serial.println(leftCount);
        Serial.print("Right encoder: ");
        Serial.println(rightCount);
        
        xSemaphoreGive(poseMutex);
      }
      Serial.println("========================\n");
    }
    else if (command.length() > 0) {
      Serial.println("Unknown command");
    }
  } 
  else {
    vTaskDelay(pdMS_TO_TICKS(1500));
    while(true){
    // tulis misi di sini reizo
    // putarStepper(4, 1); //cw turun
  
    maju(0.18);
    belok(-90);
    maju(0.27);
    belok(90);
    taskstorage();
    taskstorage();
    taskstorage();
    while(true);
    }
  }
  vTaskDelay(pdMS_TO_TICKS(100));
}
