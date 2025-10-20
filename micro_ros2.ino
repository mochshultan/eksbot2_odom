/*
 * Made by: @mochshultan
 * ESP32 DDMR Robot with FreeRTOS + micro-ROS
 * Core 0: Odometry and Navigation System with TB6612FNG Motor Driver
 * Core 1: Sensor Task + micro-ROS Publisher
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

// ===== micro-ROS includes =====
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <nav_msgs/msg/odometry.h>
#include <nav_msgs/msg/path.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>


// ========== PIN DEFINITIONS ==========
#define LINE_SENSOR_1     15
#define LINE_SENSOR_2     2 
#define LINE_SENSOR_3     4

#define IN1 14
#define IN2 23
#define IN3 19
#define IN4 18

int stepSpeed = 1000;
int stepSequence[8][4] = {
  {1,0,0,0}, {1,1,0,0},
  {0,1,0,0}, {0,1,1,0},
  {0,0,1,0}, {0,0,1,1},
  {0,0,0,1}, {1,0,0,1}
};

#define MOTOR_LEFT_PWM    12
#define MOTOR_LEFT_IN1    26
#define MOTOR_LEFT_IN2    27
#define MOTOR_RIGHT_PWM   16
#define MOTOR_RIGHT_IN1   33
#define MOTOR_RIGHT_IN2   32
#define MOTOR_STBY        25

#define ENCODER_LEFT_A    35
#define ENCODER_LEFT_B    13
#define ENCODER_RIGHT_A   5
#define ENCODER_RIGHT_B   17

#define LED_PIN           2

// ========== ROBOT PHYSICAL PARAMETERS ==========
#define WHEEL_DIAMETER    0.068
#define WHEEL_CIRCUMFERENCE (PI * WHEEL_DIAMETER)
#define WHEELBASE         0.225

#define ENCODER_PPR       11
#define R_GEAR_RATIO        171
#define L_GEAR_RATIO        171

#define EFFECTIVE_PPR_R     (ENCODER_PPR * R_GEAR_RATIO)
#define EFFECTIVE_PPR_L     (ENCODER_PPR * L_GEAR_RATIO)

// ========== GLOBAL VARIABLES ==========
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

struct RobotPose {
  double x;
  double y;
  double theta;
};

RobotPose robotPose = {0.0, 0.0, 0.0};

long prevLeftCount = 0;
long prevRightCount = 0;

SemaphoreHandle_t poseMutex;
SemaphoreHandle_t sensorMutex;
SemaphoreHandle_t pathMutex;

TaskHandle_t odometryTaskHandle = NULL;
TaskHandle_t ledTaskHandle = NULL;
TaskHandle_t microRosTaskHandle = NULL;

volatile bool navigationActive = false;
volatile double targetDistance = 0.0;
volatile double targetAngle = 0.0;
volatile bool moveForward = false;
volatile bool turnRobot = false;

// PID parameters
double kp_pos = 2.0;
double ki_pos = 0.001;
double kd_pos = 20.0;
double prev_error_pos = 0.0;
double integral_pos = 0.0;

double kp_dist = 300.0;
double ki_dist = 0.5;
double kd_dist = 6000.0;
double prev_error_dist = 0.0;
double integral_dist = 0.0;

double kp_angle = 2.0;
double ki_angle = 0.001;
double kd_angle = 20.0;
double prev_error_angle = 0.0;
double integral_angle = 0.0;

double kp_gyro = 3.0;
double ki_gyro = 0.3;
double kd_gyro = 60.0;
double prev_error_gyro = 0.0;
double integral_gyro = 0.0;

// Sensor variables
Adafruit_MPU6050 mpu;
volatile float gyroHeading = 0.0;
volatile unsigned long lastDriftCorrection = 0;

volatile int lineSensorRaw[3] = {0, 0, 0};
volatile int lineSensorDigital[3] = {0, 0, 0};
int lineThreshold[3] = {300, 300, 300};

// IMU data for micro-ROS
volatile float imuAccelX = 0.0;
volatile float imuAccelY = 0.0;
volatile float imuAccelZ = 0.0;
volatile float imuAngVelZ = 0.0;
volatile float imuHeadingDeg = 0.0;

// ===== micro-ROS variables =====
const char* ssid = "Vortex10";
const char* password = "mauuuuuu";
#define AGENT_IP "192.168.0.101"
#define AGENT_PORT 8888

rcl_publisher_t imu_pub;
rcl_publisher_t odom_pub;
rcl_publisher_t tf_pub;
rcl_publisher_t path_pub;
sensor_msgs__msg__Imu imu_msg;
nav_msgs__msg__Odometry odom_msg;
nav_msgs__msg__Path path_msg;
tf2_msgs__msg__TFMessage tf_msg;
geometry_msgs__msg__TransformStamped transformStamped;

// Buffer untuk menyimpan trajectory (max 1000 pose)
#define MAX_PATH_POINTS 100
geometry_msgs__msg__PoseStamped pathBuffer[MAX_PATH_POINTS];
volatile int pathIndex = 0;
volatile bool pathBufferFull = false;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  Serial.println("micro-ROS error");
  while(1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

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
    rightEncoderCount--;
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
  
  digitalWrite(MOTOR_STBY, HIGH);
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  if (leftSpeed >= 0) {
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
  } else {
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(MOTOR_LEFT_PWM, leftSpeed);
  
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
  
  noInterrupts();
  currentLeftCount = leftEncoderCount;
  currentRightCount = rightEncoderCount;
  interrupts();
  
  long deltaLeft = currentLeftCount - prevLeftCount;
  long deltaRight = currentRightCount - prevRightCount;
  
  double leftDistance = (deltaLeft * WHEEL_CIRCUMFERENCE) / EFFECTIVE_PPR_L;
  double rightDistance = (deltaRight * WHEEL_CIRCUMFERENCE) / EFFECTIVE_PPR_R;
  
  double deltaDistance = (leftDistance + rightDistance) / 2.0;
  double deltaTheta = (rightDistance - leftDistance) / WHEELBASE;
  
  if (xSemaphoreTake(poseMutex, portMAX_DELAY) == pdTRUE) {
    double newTheta = robotPose.theta + deltaTheta;
    
    while (newTheta > PI) newTheta -= 2 * PI;
    while (newTheta <= -PI) newTheta += 2 * PI;
    
    double avgTheta = (robotPose.theta + newTheta) / 2.0;
    robotPose.x -= deltaDistance * cos(avgTheta);
    robotPose.y -= deltaDistance * sin(avgTheta);
    robotPose.theta = newTheta;
    
    xSemaphoreGive(poseMutex);
  }
  
  prevLeftCount = currentLeftCount;
  prevRightCount = currentRightCount;
}

// ========== NAVIGATION FUNCTIONS ==========
void maju(double jarak) {
  if (navigationActive) return;
  
  navigationActive = true;
  targetDistance = abs(jarak)*0.99;
  bool isForward = (jarak >= 0);
  moveForward = true;
  turnRobot = false;
  
  prev_error_dist = 0.0;
  integral_dist = 0.0;
  
  double startX = robotPose.x;
  double startY = robotPose.y;
  
  float startHeading;
  if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    startHeading = gyroHeading;
    xSemaphoreGive(sensorMutex);
  }
  
  Serial.print(isForward ? "Maju " : "Mundur ");
  Serial.print(targetDistance);
  Serial.print(" meter, heading awal: ");
  Serial.println(startHeading);
  
  while (moveForward && navigationActive) {
    double currentDistance = sqrt(pow(robotPose.x - startX, 2) + pow(robotPose.y - startY, 2));
    
    double remainingDistance = targetDistance - currentDistance;
    
    if (remainingDistance <= 0.007) {
      stopMotors();
      moveForward = false;
      navigationActive = false;
      break;
    }
    
    double error_dist = remainingDistance;
    integral_dist += error_dist;
    integral_dist = constrain(integral_dist, -200, 200);
    double derivative_dist = error_dist - prev_error_dist;
    prev_error_dist = error_dist;
    
    double speed_output = kp_dist * error_dist + ki_dist * integral_dist + kd_dist * derivative_dist;
    int speed = constrain((int)speed_output, 20, 220);
        
    float currentHeading = startHeading;
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      currentHeading = gyroHeading;
      xSemaphoreGive(sensorMutex);
    }
    
    float headingError = startHeading - currentHeading;
    if (headingError > 180.0f)  headingError -= 360.0f;
    if (headingError < -180.0f) headingError += 360.0f;
    int correction = (int)(headingError * 25);

    if (isForward) {
      setMotorSpeed(speed + correction, speed - correction);
    } else {
      setMotorSpeed(-speed - correction, -speed + correction);
    }
    
    vTaskDelay(pdMS_TO_TICKS(20));
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
    int correctionSpeed = constrain(abs((int)correction_output), 40, 70);
        
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
  if (navigationActive) return;
  
  navigationActive = true;
  turnRobot = true;
  moveForward = false;
  
  prev_error_angle = 0.0;
  integral_angle = 0.0;
  integral_gyro = 0.0;
  prev_error_gyro = 0.0;
  
  double startTheta = robotPose.theta * 180.0 / PI;
  double targetTheta = startTheta + derajat;
  float startHeading;
  
  if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    startHeading = gyroHeading;
    xSemaphoreGive(sensorMutex);
  }
  
  while (targetTheta > 180) targetTheta -= 360;
  while (targetTheta <= -180) targetTheta += 360;
  
  Serial.print("=== Belok ");
  Serial.print(derajat);
  Serial.println(" derajat ===");
  
  while (turnRobot && navigationActive) {
    double currentTheta = robotPose.theta * 180.0 / PI;
    double remainingAngle = targetTheta - currentTheta;
    
    if (remainingAngle > 180) remainingAngle -= 360;
    if (remainingAngle <= -180) remainingAngle += 360;
    
    if (abs(remainingAngle) < 2) {
      stopMotors();
      turnRobot = false;
      break;
    }
    
    double error_angle = remainingAngle;
    integral_angle += error_angle;
    integral_angle = constrain(integral_angle, -10, 10);
    double derivative_angle = error_angle - prev_error_angle;
    prev_error_angle = error_angle;
    
    double turn_output = kp_angle * abs(error_angle) + ki_angle * integral_angle + kd_angle * abs(derivative_angle);
    int turnSpeed = constrain((int)turn_output, 40, 175);
        
    if (remainingAngle > 0) {
      setMotorSpeed(-turnSpeed, turnSpeed);
    } else {
      setMotorSpeed(turnSpeed, -turnSpeed);
    }
    
    vTaskDelay(pdMS_TO_TICKS(20));
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
    int correctionSpeed = constrain(abs((int)correction_output), 40, 70);
        
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

void stepMotor(int stepIndex) {
  digitalWrite(IN1, stepSequence[stepIndex][0]);
  digitalWrite(IN2, stepSequence[stepIndex][1]);
  digitalWrite(IN3, stepSequence[stepIndex][2]);
  digitalWrite(IN4, stepSequence[stepIndex][3]);
}

void putarStepper(int jumlahPutaran, int arah) {
  int totalStep = 4096 * jumlahPutaran;

  if (arah > 0) {
    for (int step = 0; step < totalStep; step++) {
      stepMotor(step % 8);
      delayMicroseconds(stepSpeed);
    }
  } else {
    for (int step = totalStep; step > 0; step--) {
      stepMotor(step % 8);
      delayMicroseconds(stepSpeed);
    }
  }
}

// ========== micro-ROS FUNCTIONS ==========
static void init_microros_publishers() {
  sensor_msgs__msg__Imu__init(&imu_msg);
  nav_msgs__msg__Odometry__init(&odom_msg);
  tf2_msgs__msg__TFMessage__init(&tf_msg);
  geometry_msgs__msg__TransformStamped__init(&transformStamped);
  
  // Proper path message initialization
  nav_msgs__msg__Path__init(&path_msg);
  path_msg.poses.data = (geometry_msgs__msg__PoseStamped*)malloc(sizeof(geometry_msgs__msg__PoseStamped) * MAX_PATH_POINTS);
  path_msg.poses.capacity = MAX_PATH_POINTS;
  path_msg.poses.size = 0;
  
  // Initialize each pose in the buffer
  for (int i = 0; i < MAX_PATH_POINTS; i++) {
    geometry_msgs__msg__PoseStamped__init(&pathBuffer[i]);
    geometry_msgs__msg__PoseStamped__init(&path_msg.poses.data[i]);
  }

  RCCHECK(rclc_publisher_init_default(
    &imu_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data"
  ));

  RCCHECK(rclc_publisher_init_default(
    &odom_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"
  ));

  RCCHECK(rclc_publisher_init_default(
    &tf_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "tf"
  ));

  RCCHECK(rclc_publisher_init_default(
    &path_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Path),
    "robot_path"  // Topic name untuk RViz
  ));
}


// ===== TAMBAHKAN FUNGSI UNTUK UPDATE PATH =====
void updateTrajectory(double x, double y, double theta) {
  if (xSemaphoreTake(pathMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (pathIndex < MAX_PATH_POINTS) {
      uint32_t ms = millis();
      
      pathBuffer[pathIndex].header.stamp.sec = ms / 1000;
      pathBuffer[pathIndex].header.stamp.nanosec = (ms % 1000) * 1000000;
      pathBuffer[pathIndex].header.frame_id.data = (char*)"odom";
      pathBuffer[pathIndex].header.frame_id.size = strlen("odom");
      pathBuffer[pathIndex].header.frame_id.capacity = strlen("odom") + 1;
      
      // Gunakan transformed position (seperti di publish odometry)
      pathBuffer[pathIndex].pose.position.x = -x;  // Sesuai dengan transformasi axis
      pathBuffer[pathIndex].pose.position.y = -y;  // Sesuai dengan transformasi axis
      pathBuffer[pathIndex].pose.position.z = 0.0;
      
      double theta_transformed = theta + PI;
      double qz = sin(theta_transformed / 2.0);
      double qw = cos(theta_transformed / 2.0);
      
      pathBuffer[pathIndex].pose.orientation.x = 0.0;
      pathBuffer[pathIndex].pose.orientation.y = 0.0;
      pathBuffer[pathIndex].pose.orientation.z = qz;
      pathBuffer[pathIndex].pose.orientation.w = qw;
      
      pathIndex++;
    } else {
      pathBufferFull = true;
      Serial.println("⚠️ Path buffer penuh! Max 10000 points tercapai");
    }
    xSemaphoreGive(pathMutex);
  }
}

void microRosTask(void *parameter) {
  Serial.println("micro-ROS task starting...");
  set_microros_wifi_transports("Vortex10", "mauuuuuu", "192.168.0.101", 8888);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_robot", "", &support));

  init_microros_publishers();

  const TickType_t publishDelay = pdMS_TO_TICKS(50);
  while (true) {
    double px, py, pth;
    float ax, ay, az, gz, heading_deg;
    
    if (xSemaphoreTake(poseMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      px = robotPose.x;
      py = robotPose.y;
      pth = robotPose.theta;
      xSemaphoreGive(poseMutex);
    } else {
      px = py = pth = 0.0;
    }

    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      ax = imuAccelX;
      ay = imuAccelY;
      az = imuAccelZ;
      gz = imuAngVelZ;
      heading_deg = imuHeadingDeg;
      xSemaphoreGive(sensorMutex);
    } else {
      ax = ay = az = gz = heading_deg = 0.0;
    }

    uint32_t ms = millis();
    builtin_interfaces__msg__Time stamp;
    stamp.sec = ms / 1000;
    stamp.nanosec = (ms % 1000) * 1000000;

    // ===== PUBLISH TRAJECTORY =====
    if (xSemaphoreTake(pathMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      path_msg.header.stamp = stamp;
      path_msg.header.frame_id.data = (char*)"odom";
      path_msg.header.frame_id.size = strlen("odom");
      path_msg.header.frame_id.capacity = strlen("odom") + 1;
      
      // Update size to current path index
      path_msg.poses.size = pathIndex;
      
      // Copy buffer ke message (sudah teralokasi di init)
      for (int i = 0; i < pathIndex && i < MAX_PATH_POINTS; i++) {
        path_msg.poses.data[i] = pathBuffer[i];
      }
      
      RCSOFTCHECK(rcl_publish(&path_pub, &path_msg, NULL));
      xSemaphoreGive(pathMutex);
    }

    // Fill IMU message
    imu_msg.header.stamp = stamp;
    imu_msg.header.frame_id.data = (char*)"base_link";
    imu_msg.header.frame_id.size = strlen("base_link");
    imu_msg.header.frame_id.capacity = strlen("base_link") + 1;
    
    double yaw = heading_deg * M_PI / 180.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = sin(yaw / 2.0);
    double qw = cos(yaw / 2.0);
    imu_msg.orientation.x = qx;
    imu_msg.orientation.y = qy;
    imu_msg.orientation.z = qz;
    imu_msg.orientation.w = qw;
    imu_msg.orientation_covariance[0] = -1.0;

    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = gz;

    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;

    RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));

    // Fill Odometry message - konsisten dengan path coordinate
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id.data = (char*)"odom";
    odom_msg.header.frame_id.size = strlen("odom");
    odom_msg.header.frame_id.capacity = strlen("odom") + 1;
    odom_msg.child_frame_id.data = (char*)"base_link";
    odom_msg.child_frame_id.size = strlen("base_link");
    odom_msg.child_frame_id.capacity = strlen("base_link") + 1;

    // Konsisten dengan transformasi di path
    odom_msg.pose.pose.position.x = -px;
    odom_msg.pose.pose.position.y = -py;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = qx;
    odom_msg.pose.pose.orientation.y = qy;
    odom_msg.pose.pose.orientation.z = qz;
    odom_msg.pose.pose.orientation.w = qw;

    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = gz;

    RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, NULL));

    // Publish TF
    transformStamped.header.stamp = stamp;
    transformStamped.header.frame_id.data = (char*)"odom";
    transformStamped.header.frame_id.size = strlen("odom");
    transformStamped.header.frame_id.capacity = strlen("odom") + 1;
    transformStamped.child_frame_id.data = (char*)"base_link";
    transformStamped.child_frame_id.size = strlen("base_link");
    transformStamped.child_frame_id.capacity = strlen("base_link") + 1;
    transformStamped.transform.translation.x = -px;
    transformStamped.transform.translation.y = -py;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = qx;
    transformStamped.transform.rotation.y = qy;
    transformStamped.transform.rotation.z = qz;
    transformStamped.transform.rotation.w = qw;

    if (tf_msg.transforms.data == NULL) {
      geometry_msgs__msg__TransformStamped__Sequence__init(&tf_msg.transforms, 1);
    } else if (tf_msg.transforms.size != 1) {
      geometry_msgs__msg__TransformStamped__Sequence__fini(&tf_msg.transforms);
      geometry_msgs__msg__TransformStamped__Sequence__init(&tf_msg.transforms, 1);
    }
    tf_msg.transforms.data[0] = transformStamped;

    RCSOFTCHECK(rcl_publish(&tf_pub, &tf_msg, NULL));

    vTaskDelay(publishDelay);
  }
}

// ========== FREERTOS TASKS ==========
void odometryTask(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);
  
  Serial.println("Odometry task started on Core 0");

  static unsigned long lastTrajectoryUpdate = 0;
  
  while (true) {
    updateOdometry();

    // Update trajectory setiap 100ms untuk mengurangi data points
    if (millis() - lastTrajectoryUpdate > 100) {
      if (xSemaphoreTake(poseMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        updateTrajectory(robotPose.x, robotPose.y, robotPose.theta);
        xSemaphoreGive(poseMutex);
      }
      lastTrajectoryUpdate = millis();
    }
    
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
  Serial.println("Sensor IMU task dimulai di Core 1");
  
  if (!mpu.begin()) {
    Serial.println("Gagal menginisialisasi MPU6050!");
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  Serial.println("MPU6050 berhasil diinisialisasi");
  
  float gyroZ_offset = 0;
  float heading = 0;
  unsigned long lastTime = millis();
  
  Serial.println("Kalibrasi gyro, jangan gerakkan robot...");
  for (int i = 0; i < 250; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroZ_offset += g.gyro.z;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  gyroZ_offset /= 250;
  Serial.print("Offset gyro Z: ");
  Serial.println(gyroZ_offset);
  
  while (true) {
    int line1_raw = analogRead(LINE_SENSOR_1);
    int line2_raw = analogRead(LINE_SENSOR_2);
    int line3_raw = analogRead(LINE_SENSOR_3);
    
    int line1_digital = (line1_raw > lineThreshold[0]) ? 1 : 0;
    int line2_digital = (line2_raw > lineThreshold[1]) ? 1 : 0;
    int line3_digital = (line3_raw > lineThreshold[2]) ? 1 : 0;
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;
    
    float gyroZ_corrected = g.gyro.z - gyroZ_offset;
    
    if (abs(gyroZ_corrected) < 0.01) {
      gyroZ_corrected = 0.0;
    }
    heading += gyroZ_corrected * dt * 180.0 / PI;
    
    while (heading > 180) heading -= 360;
    while (heading <= -180) heading += 360;
    
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      lineSensorRaw[0] = line1_raw;
      lineSensorRaw[1] = line2_raw;
      lineSensorRaw[2] = line3_raw;
      lineSensorDigital[0] = line1_digital;
      lineSensorDigital[1] = line2_digital;
      lineSensorDigital[2] = line3_digital;
      gyroHeading = heading;
      imuAccelX = a.acceleration.x;
      imuAccelY = a.acceleration.y;
      imuAccelZ = a.acceleration.z;
      imuAngVelZ = gyroZ_corrected;
      imuHeadingDeg = heading;
      xSemaphoreGive(sensorMutex);
    }
    
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
    
    static bool ledState = false;
    static unsigned long lastLedToggle = 0;
    if (millis() - lastLedToggle > 250) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastLedToggle = millis();
    }
    
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ========== SETUP AND LOOP ==========
void setup() {
  Serial.begin(115200);
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  Serial.println("ESP32 DDMR Robot with FreeRTOS + micro-ROS Starting...");
  
  setupMotorDriver();
  pinMode(LED_PIN, OUTPUT);
  
  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
  
  pinMode(LINE_SENSOR_1, INPUT);
  pinMode(LINE_SENSOR_2, INPUT);
  pinMode(LINE_SENSOR_3, INPUT);
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  Wire.begin();
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR, CHANGE);
  
  poseMutex = xSemaphoreCreateMutex();
  sensorMutex = xSemaphoreCreateMutex();
  pathMutex = xSemaphoreCreateMutex();
  
  xTaskCreatePinnedToCore(
    odometryTask,
    "OdometryTask",
    4096,
    NULL,
    2,
    &odometryTaskHandle,
    0
  );
  
  xTaskCreatePinnedToCore(
    ledTask,
    "SensorTask",
    4096,
    NULL,
    1,
    &ledTaskHandle,
    1
  );

  xTaskCreatePinnedToCore(
    microRosTask,
    "MicroROSTask",
    8192,
    NULL,
    1,
    &microRosTaskHandle,
    1
  );
  
  Serial.println("Setup completed. Robot ready!");
  vTaskDelay(pdMS_TO_TICKS(2000));
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "info") {
      Serial.println("\n=== Robot Configuration ===");
      Serial.print("Wheel diameter: ");
      Serial.print(WHEEL_DIAMETER * 1000);
      Serial.println(" mm");
      Serial.println("========================\n");
    }
    else if (command.length() > 0) {
      Serial.println("Unknown command");
    }
  } 
  else {
    vTaskDelay(pdMS_TO_TICKS(1500));
    while(true){
      maju(0.6);
      belok(90);
      maju(0.6);
      belok(90);
      maju(0.6);
      belok(90);
      maju(0.6);
      belok(90);
      // while(true);
    }
  }
  vTaskDelay(pdMS_TO_TICKS(100));
}
