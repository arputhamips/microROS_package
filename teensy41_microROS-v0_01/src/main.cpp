#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ROS Message Types
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <nav_msgs/msg/odometry.h>

// Hardware Libraries
#include <Encoder.h>
#include <Adafruit_BNO08x.h>

// ==========================================
// 1. FLEET CONFIGURATION
// ==========================================
#define ROBOT_NAMESPACE "" 

// ==========================================
// 2. PIN DEFINITIONS (Teensy 4.1)
// ==========================================
// Motor Driver (Cytron MDD10A)
#define PIN_M_RIGHT_PWM 0
#define PIN_M_RIGHT_DIR 1
#define PIN_M_LEFT_PWM  2
#define PIN_M_LEFT_DIR  3

// Encoders (REV-41-1301)
#define PIN_ENC_RIGHT_A 36
#define PIN_ENC_RIGHT_B 35
#define PIN_ENC_LEFT_A  33
#define PIN_ENC_LEFT_B  34

// IMU (Adafruit BNO085) - Wire1 (Pins 24/25)
#define PIN_IMU_SDA 25
#define PIN_IMU_SCL 24

// ==========================================
// 3. ROBOT CONSTANTS
// ==========================================
#define TICKS_PER_REV 1120.0 
#define WHEEL_DIA 0.045      
#define WHEEL_BASE 0.30      
#define MAX_PWM 255

const double DIST_PER_TICK = (PI * WHEEL_DIA) / TICKS_PER_REV;

// ==========================================
// 4. GLOBALS & OBJECTS
// ==========================================
Encoder encLeft(PIN_ENC_LEFT_A, PIN_ENC_LEFT_B);
Encoder encRight(PIN_ENC_RIGHT_A, PIN_ENC_RIGHT_B);
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// Micro-ROS Entities
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_timer_t timer;

// Publishers & Subscribers
rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t joint_pub;
sensor_msgs__msg__JointState joint_msg;

rcl_publisher_t odom_pub;
nav_msgs__msg__Odometry odom_msg;

rcl_subscription_t twist_sub;
geometry_msgs__msg__Twist twist_msg;

// Joint State Storage
double joint_pos[2];
double joint_vel[2];
double joint_eff[2];
rosidl_runtime_c__String joint_names[2];
char name0[] = "left_wheel_joint";
char name1[] = "right_wheel_joint";

// Odometry State
double odom_x = 0.0;
double odom_y = 0.0;
double odom_theta = 0.0;

// State Variables
long prev_ticks_left = 0;
long prev_ticks_right = 0;
unsigned long prev_time = 0;

// ==========================================
// 5. HELPER FUNCTIONS
// ==========================================

void set_motor_speeds(float linear_x, float angular_z) {
  float left_cmd = (linear_x + angular_z * (WHEEL_BASE / 2.0)) * 255.0 * 2.0;  
  float right_cmd = (linear_x - angular_z * (WHEEL_BASE / 2.0)) * 255.0 * 2.0; 

  int left_pwm = constrain((int)left_cmd, -MAX_PWM, MAX_PWM);
  int right_pwm = constrain((int)right_cmd, -MAX_PWM, MAX_PWM);

  // Drive Left
  if (left_pwm >= 0) {
    digitalWrite(PIN_M_LEFT_DIR, HIGH); 
    analogWrite(PIN_M_LEFT_PWM, left_pwm);
  } else {
    digitalWrite(PIN_M_LEFT_DIR, LOW); 
    analogWrite(PIN_M_LEFT_PWM, -left_pwm);
  }

  // Drive Right
  if (right_pwm >= 0) {
    digitalWrite(PIN_M_RIGHT_DIR, LOW); 
    analogWrite(PIN_M_RIGHT_PWM, right_pwm);
  } else {
    digitalWrite(PIN_M_RIGHT_DIR, HIGH); 
    analogWrite(PIN_M_RIGHT_PWM, -right_pwm);
  }
}

void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  set_motor_speeds(msg->linear.x, msg->angular.z);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  
  unsigned long now = millis();
  double dt = (now - prev_time) / 1000.0;
  prev_time = now;

  // --- 1. READ ENCODERS ---
  long curr_ticks_left = encLeft.read();
  long curr_ticks_right = encRight.read();

  double d_ticks_l = curr_ticks_left - prev_ticks_left;
  double d_ticks_r = curr_ticks_right - prev_ticks_right;
  prev_ticks_left = curr_ticks_left;
  prev_ticks_right = curr_ticks_right;

  // Joint States
  double rads_per_tick = (2.0 * PI) / TICKS_PER_REV;
  joint_msg.position.data[0] = curr_ticks_left * rads_per_tick;
  joint_msg.position.data[1] = curr_ticks_right * rads_per_tick;
  
  if (dt > 0) {
    joint_msg.velocity.data[0] = (d_ticks_l * rads_per_tick) / dt;
    joint_msg.velocity.data[1] = (d_ticks_r * rads_per_tick) / dt;
  }

  // Sync Time
  int64_t time_ns = rmw_uros_epoch_nanos();
  joint_msg.header.stamp.sec = time_ns / 1000000000;
  joint_msg.header.stamp.nanosec = time_ns % 1000000000;

  rcl_publish(&joint_pub, &joint_msg, NULL);

  // --- 2. CALCULATE ODOMETRY ---
  double d_left = d_ticks_l * DIST_PER_TICK;
  double d_right = d_ticks_r * DIST_PER_TICK;

  double d_center = (d_left + d_right) / 2.0;
  double phi = (d_right - d_left) / WHEEL_BASE;

  odom_x += d_center * cos(odom_theta);
  odom_y += d_center * sin(odom_theta);
  odom_theta += phi;

  // [FIX] Normalize Theta (-PI to PI)
  while (odom_theta > PI) odom_theta -= 2.0 * PI;
  while (odom_theta < -PI) odom_theta += 2.0 * PI;

  // Fill Message
  odom_msg.header.stamp = joint_msg.header.stamp;

  // Pose
  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation.z = sin(odom_theta / 2.0);
  odom_msg.pose.pose.orientation.w = cos(odom_theta / 2.0);

  // Twist
  if (dt > 0) {
    odom_msg.twist.twist.linear.x = d_center / dt;
    odom_msg.twist.twist.angular.z = phi / dt;
  }

  // [FIX] Populate Covariances (Diagonal only)
  // Low value (0.01) = High confidence. High value = Low confidence.
  // 6x6 Matrix (0, 7, 14, 21, 28, 35 are diagonals)
  
  // Pose Covariance: Trust X, Y, Yaw. Don't trust Z, Roll, Pitch.
  odom_msg.pose.covariance[0] = 0.01;  // X
  odom_msg.pose.covariance[7] = 0.01;  // Y
  odom_msg.pose.covariance[14] = 999.0; // Z
  odom_msg.pose.covariance[21] = 999.0; // Roll
  odom_msg.pose.covariance[28] = 999.0; // Pitch
  odom_msg.pose.covariance[35] = 0.01;  // Yaw

  // Twist Covariance
  odom_msg.twist.covariance[0] = 0.01;  // Vx
  odom_msg.twist.covariance[7] = 0.01;  // Vy
  odom_msg.twist.covariance[14] = 999.0;
  odom_msg.twist.covariance[21] = 999.0;
  odom_msg.twist.covariance[28] = 999.0;
  odom_msg.twist.covariance[35] = 0.01; // Vyaw

  rcl_publish(&odom_pub, &odom_msg, NULL);

  // --- 3. READ IMU ---
  if (bno08x.getSensorEvent(&sensorValue)) {
    imu_msg.header.stamp = joint_msg.header.stamp; 
    imu_msg.header.frame_id.data = "imu_link";
    imu_msg.header.frame_id.size = strlen("imu_link");

    // [FIX] Use RAW mapping. Handle rotation in XACRO.
    switch (sensorValue.sensorId) {
      case SH2_ROTATION_VECTOR:
        imu_msg.orientation.x = sensorValue.un.rotationVector.i;
        imu_msg.orientation.y = sensorValue.un.rotationVector.j;
        imu_msg.orientation.z = sensorValue.un.rotationVector.k;
        imu_msg.orientation.w = sensorValue.un.rotationVector.real;
        // Covariance
        imu_msg.orientation_covariance[0] = 0.01;
        imu_msg.orientation_covariance[4] = 0.01;
        imu_msg.orientation_covariance[8] = 0.01;
        break;
      
      case SH2_ACCELEROMETER:
        imu_msg.linear_acceleration.x = sensorValue.un.accelerometer.x; 
        imu_msg.linear_acceleration.y = sensorValue.un.accelerometer.y; 
        imu_msg.linear_acceleration.z = sensorValue.un.accelerometer.z;
        // Covariance
        imu_msg.linear_acceleration_covariance[0] = 0.01;
        imu_msg.linear_acceleration_covariance[4] = 0.01;
        imu_msg.linear_acceleration_covariance[8] = 0.01;
        break;
        
      case SH2_GYROSCOPE_CALIBRATED:
        imu_msg.angular_velocity.x = sensorValue.un.gyroscope.x;
        imu_msg.angular_velocity.y = sensorValue.un.gyroscope.y; 
        imu_msg.angular_velocity.z = sensorValue.un.gyroscope.z;
        // Covariance
        imu_msg.angular_velocity_covariance[0] = 0.01;
        imu_msg.angular_velocity_covariance[4] = 0.01;
        imu_msg.angular_velocity_covariance[8] = 0.01;
        break;
    }
    rcl_publish(&imu_pub, &imu_msg, NULL);
  }
}

void setup() {
  set_microros_serial_transports(Serial);
  
  pinMode(PIN_M_RIGHT_PWM, OUTPUT);
  pinMode(PIN_M_RIGHT_DIR, OUTPUT);
  pinMode(PIN_M_LEFT_PWM, OUTPUT);
  pinMode(PIN_M_LEFT_DIR, OUTPUT);

  if (bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire2)) {
    bno08x.enableReport(SH2_ROTATION_VECTOR);
    bno08x.enableReport(SH2_ACCELEROMETER);
    bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED);
  }

  allocator = rcl_get_default_allocator();

  while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
      delay(500);
  }

  rmw_uros_sync_session(1000);

  rclc_node_init_default(&node, "base_mcu_node", ROBOT_NAMESPACE, &support);

  // 1. Joint States
  rclc_publisher_init_default(
    &joint_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "joint_states"
  );
  
  static char frame_id[] = "base_link";
  joint_msg.header.frame_id.data = frame_id;
  joint_msg.header.frame_id.size = strlen(frame_id);
  joint_msg.header.frame_id.capacity = sizeof(frame_id);
  
  joint_msg.name.data = joint_names;
  joint_msg.name.size = 2; joint_msg.name.capacity = 2;
  joint_names[0].data = name0; joint_names[0].size = strlen(name0); joint_names[0].capacity = sizeof(name0);
  joint_names[1].data = name1; joint_names[1].size = strlen(name1); joint_names[1].capacity = sizeof(name1);
  joint_msg.position.data = joint_pos; joint_msg.position.size = 2; joint_msg.position.capacity = 2;
  joint_msg.velocity.data = joint_vel; joint_msg.velocity.size = 2; joint_msg.velocity.capacity = 2;
  joint_msg.effort.data = joint_eff; joint_msg.effort.size = 2; joint_msg.effort.capacity = 2;

  delay(100);

  // 2. Odometry Publisher (Using standard topic name for EKF)
  rclc_publisher_init_default(
    &odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom/unfiltered"
  );

  static char odom_frame[] = "odom";
  static char child_frame[] = "base_link";
  
  odom_msg.header.frame_id.data = odom_frame;
  odom_msg.header.frame_id.size = strlen(odom_frame);
  odom_msg.header.frame_id.capacity = sizeof(odom_frame);
  
  odom_msg.child_frame_id.data = child_frame;
  odom_msg.child_frame_id.size = strlen(child_frame);
  odom_msg.child_frame_id.capacity = sizeof(child_frame);

  delay(100);

  // 3. IMU
  rclc_publisher_init_default(
    &imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data"
  );
  
  delay(100);

  // 4. Cmd Vel Subscriber
  rclc_subscription_init_default(
    &twist_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  );
  
  delay(100);

  // Timer
  rclc_timer_init_default(
    &timer, &support, RCL_MS_TO_NS(50), timer_callback
  );

  // [FIX] Increased executor size to 5 to be safe
  rclc_executor_init(&executor, &support.context, 5, &allocator); 
  rclc_executor_add_timer(&executor, &timer);
  rclc_executor_add_subscription(&executor, &twist_sub, &twist_msg, &cmd_vel_callback, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}