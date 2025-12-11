# Teensy 4.1 ROS 2 Differential Drive Controller

This project provides firmware for a Teensy 4.1 microcontroller to act as a low-level hardware bridge for a differential drive robot using ROS 2 (Jazzy/Humble) and micro-ROS.

It handles motor control (PWM/Direction), reads quadrature encoders for odometry, interfaces with a BNO085 IMU, and communicates directly with a ROS 2 host (e.g., Raspberry Pi 5) over USB Serial.

---

## 1. Hardware Requirements

| Component | Specification |
|-----------|---------------|
| **Microcontroller** | Teensy 4.1 |
| **Motor Driver** | Cytron MDD10A (or similar PWM/DIR driver) |
| **Motors** | DC Motors with Quadrature Encoders (Code configured for REV HD Hex Motors) |
| **IMU** | Adafruit BNO085 (connected via I2C Wire2) |
| **Host Computer** | Raspberry Pi 5 or Laptop running Ubuntu 24.04 (ROS 2 Jazzy) |

---

## 2. Development Setup (PlatformIO)

This project is built using PlatformIO within VS Code.

### Step 1: Prerequisites

- Install [VS Code](https://code.visualstudio.com/)
- Install the **PlatformIO IDE** extension from the VS Code Marketplace

### Step 2: Project Initialization

Create a new project in PlatformIO:
- **Board:** Teensy 4.1
- **Framework:** Arduino

### Step 3: Configuration (`platformio.ini`)

Replace the contents of your `platformio.ini` with the following configuration to ensure all dependencies and the micro-ROS library are correctly linked:

```ini
[env:teensy41]
platform = teensy
board = teensy41
framework = arduino
monitor_speed = 115200

# micro-ROS configuration
board_microros_transport = serial
board_microros_distro = jazzy  ; or humble

lib_deps =
    https://github.com/micro-ROS/micro_ros_platformio
    paulstoffregen/Encoder
    adafruit/Adafruit BNO08x
```

### Step 4: Build & Upload

1. Connect the Teensy 4.1 via USB
2. Click the **Alien Face** icon (PlatformIO) on the left sidebar
3. Click **Build** to compile
4. Click **Upload** to flash the firmware

---

## 3. Code Overview & Configuration

The main logic resides in `src/main.cpp`. The code uses a **Timer Callback** approach to ensure consistent loop timing (50ms / 20Hz) for odometry calculation, rather than relying on the main `loop()`.

### Key Parameters to Modify

If you are adapting this code for a different robot, look for **Section 3: ROBOT CONSTANTS** in the code:

| Constant | Description | Default Value |
|----------|-------------|---------------|
| `TICKS_PER_REV` | Encoder ticks per one full wheel rotation | `1120.0` (REV HD Hex) |
| `WHEEL_DIA` | Diameter of the drive wheels (meters) | `0.045` |
| `WHEEL_BASE` | Distance between the center of the left and right wheels/tracks (meters) | `0.30` |
| `PIN_M_...` | GPIO pins for Motor PWM and Direction | `0, 1, 2, 3` |
| `PIN_ENC_...` | GPIO pins for Encoder Phase A/B | `33, 34, 35, 36` |

### Core Functions

#### `set_motor_speeds(linear_x, angular_z)`
- Converts ROS 2 Twist commands (m/s, rad/s) into PWM values (0-255)
- Handles the differential drive kinematics
- **Note:** If your robot spins the wrong way, check the sign (+/−) logic here

#### `timer_callback(...)`
Runs every 50ms:
- Reads Encoders & Calculates Odometry (x, y, θ)
- Reads IMU data (Quaternion, Accel, Gyro)
- Publishes `odom`, `imu`, and `joint_states` to ROS 2

#### `cmd_vel_callback(...)`
- Listens for velocity commands from the navigation stack or joystick
- Updates motor speeds immediately

---

## 4. Setting up the Micro-ROS Agent

For the Teensy to talk to ROS 2, you must run the **Micro-ROS Agent** on your Host Computer (Raspberry Pi/Laptop). The agent acts as a bridge, forwarding messages from the USB serial port to the DDS network.

### Method A: Using Docker (Recommended)

This is the easiest way to run the agent without building it from source.

```bash
# Run the agent for ROS 2 Jazzy
sudo docker run -it --rm -v /dev:/dev --privileged \
    microros/micro-ros-agent:jazzy serial --dev /dev/ttyACM0 -b 6000000
```

- Replace `jazzy` with `humble` if needed
- `/dev/ttyACM0` is usually the Teensy port. Check with `ls /dev/ttyACM*`

### Method B: Building from Source

If you are building a full workspace:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 6000000
```

---

## 5. ROS 2 Interface Guide

Once the Agent is running and the Teensy is connected, the following ROS 2 Graph is established.

**Node Name:** `/base_mcu_node`

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands to drive the robot |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/odom/unfiltered` | `nav_msgs/Odometry` | Raw wheel odometry (X, Y, Theta, Linear Vel, Angular Vel). Used for sensor fusion |
| `/imu/data` | `sensor_msgs/Imu` | Absolute orientation (Quaternion) and acceleration from BNO085 |
| `/joint_states` | `sensor_msgs/JointState` | Position and Velocity of the wheels (in radians), useful for `robot_state_publisher` |

### Message Frame Structure

The code assumes the following TF frame tree:

```plaintext
odom (Fixed Map Frame)
 └── base_link (Robot Center)
      └── imu_link (IMU Sensor location)
```

---

## 6. TODO: Planned Improvements

### 🔧 Timer Callback Optimization
**Goal:** Test different polling rates for optimal ROS 2 integration with the Raspberry Pi

- [ ] **Benchmark Current Performance**
  - Measure current callback execution time using `micros()`
  - Log timestamp deltas to identify timing jitter
  - Monitor CPU usage on both Teensy and Pi

- [ ] **Test Different Timer Frequencies**
  ```cpp
  // Current: 50ms (20Hz)
  // Test these frequencies:
  - [ ] 100Hz (10ms) - High-frequency control
  - [ ] 50Hz (20ms)  - Current baseline
  - [ ] 30Hz (33ms)  - Moderate frequency
  - [ ] 20Hz (50ms)  - Lower frequency
  - [ ] 10Hz (100ms) - Minimal overhead
  ```

- [ ] **Create Configurable Timer Parameter**
  - Add `#define CALLBACK_PERIOD_MS 50` at the top of `main.cpp`
  - Allow runtime configuration via ROS 2 parameter (stretch goal)
  - Test impact on odometry drift at different rates

- [ ] **Profile Specific Functions**
  - Measure encoder reading time
  - Measure IMU I2C transaction time
  - Measure ROS 2 publish overhead
  - Identify bottlenecks for optimization

---

### 💡 LED Visual Diagnostics System

**Goal:** Use Teensy's onboard LED and an external LED to provide real-time status feedback

#### LED Hardware Setup
```cpp
// Pin definitions (add to main.cpp)
#define LED_STATUS    13  // Onboard LED (always available)
#define LED_ERROR     2   // External LED (optional, choose unused pin)
```

#### Status LED Patterns (LED_STATUS - Pin 13)

| Pattern | Meaning | Implementation |
|---------|---------|----------------|
| **Solid ON** | System fully operational, ROS 2 connected | Normal operation |
| **Slow Blink (1Hz)** | Waiting for namespace configuration | Blink every 1000ms |
| **Fast Blink (5Hz)** | Namespace received, initializing topics | Blink every 200ms |
| **Heartbeat (Fade)** | All systems nominal, active communication | PWM fade in/out |
| **OFF** | Critical failure or not powered | N/A |

#### Error LED Codes (LED_ERROR - Pin 2)

| Blink Code | Error Type | Description |
|------------|------------|-------------|
| **1 blink** | IMU Failure | BNO085 not responding on I2C Wire2 |
| **2 blinks** | Encoder Error | No encoder pulses detected for 5 seconds |
| **3 blinks** | Motor Driver Fault | PWM signal issue or driver not responding |
| **4 blinks** | ROS 2 Agent Lost | micro-ROS agent disconnected |
| **5 blinks** | Memory Error | Heap/Stack overflow detected |
| **Rapid Flash** | Emergency Stop | Safety stop triggered (future feature) |

#### Implementation Checklist

- [ ] **Basic LED Setup**
  ```cpp
  void setup_leds() {
    pinMode(LED_STATUS, OUTPUT);
    pinMode(LED_ERROR, OUTPUT);
    digitalWrite(LED_STATUS, LOW);
    digitalWrite(LED_ERROR, LOW);
  }
  ```

- [ ] **LED State Machine**
  - [ ] Create `enum class SystemState`
  - [ ] Implement `update_status_led()` function
  - [ ] Add LED update to main loop (non-blocking)

- [ ] **Error Code Blinker**
  - [ ] Create `blink_error_code(uint8_t code)` function
  - [ ] Use `millis()` for non-blocking blinks
  - [ ] Clear error when issue is resolved

- [ ] **Health Monitoring**
  - [ ] IMU connection watchdog (timeout = 2s)
  - [ ] Encoder activity monitor (timeout = 5s)
  - [ ] ROS 2 agent connection check (use `rmw_uros_ping_agent()`)
  - [ ] Trigger appropriate LED error codes

---

### 🏷️ Dynamic Namespace Management

**Goal:** Request robot namespace from Pi on startup; delay topic initialization until namespace is received

#### Architecture

```plaintext
Teensy Startup Sequence:
  1. Power On → LED: Slow Blink (waiting for namespace)
  2. Request namespace via service call → /get_robot_namespace
  3. Receive response (e.g., "robot_01") → LED: Fast Blink
  4. Initialize all publishers/subscribers with namespace
  5. Begin normal operation → LED: Heartbeat
```

#### Implementation Options

**Option A: Service Client (Recommended)**
```cpp
// Teensy calls this service on startup
Service: /teensy/get_namespace
Type: std_srvs/srv/SetString (request: empty, response: namespace)

// Example:
Request:  (empty)
Response: {data: "robot_01"}
```

**Option B: Subscriber (Alternative)**
```cpp
// Pi publishes namespace, Teensy subscribes
Topic: /teensy/set_namespace
Type: std_msgs/String

// Pi publishes once on detection of new Teensy
```

#### Namespace Reset/Clear

**Method 1: Reboot (Simple)**
- Send empty namespace → Teensy reboots and requests again

**Method 2: Reset Service (Advanced)**
```cpp
Service: /teensy/reset_namespace
Type: std_srvs/srv/Trigger

// Calling this service:
// 1. Destroys all current publishers/subscribers
// 2. Resets to "waiting for namespace" state
// 3. Requests new namespace from Pi
```

#### Implementation Checklist

- [ ] **Namespace Storage**
  ```cpp
  char robot_namespace[32] = "";  // Global namespace buffer
  bool namespace_received = false;
  ```

- [ ] **Service Client Setup**
  - [ ] Add service client library to `platformio.ini`
  - [ ] Create service client: `/teensy/get_namespace`
  - [ ] Implement blocking call with timeout (5 seconds)
  - [ ] Retry logic: 3 attempts, then fallback to default "robot_00"

- [ ] **Conditional Topic Initialization**
  ```cpp
  void init_ros_topics() {
    if (!namespace_received) {
      // Don't initialize yet
      return;
    }
    
    // Build topic names with namespace
    sprintf(odom_topic, "/%s/odom/unfiltered", robot_namespace);
    sprintf(cmd_vel_topic, "/%s/cmd_vel", robot_namespace);
    // ... etc
  }
  ```

- [ ] **Pi-Side Service Server**
  - [ ] Create ROS 2 node: `teensy_namespace_server`
  - [ ] Implement service: `/teensy/get_namespace`
  - [ ] Logic: Assign next available namespace or read from config file
  - [ ] Log namespace assignments to file for tracking

- [ ] **Namespace Reset Feature**
  - [ ] Add reset service to Teensy: `/teensy/reset_namespace`
  - [ ] Implement `destroy_ros_topics()` function
  - [ ] Clear `robot_namespace` buffer
  - [ ] Re-request namespace from Pi

- [ ] **Testing Strategy**
  - [ ] Test with Pi offline (fallback to default namespace)
  - [ ] Test with multiple Teensys requesting namespaces simultaneously
  - [ ] Test namespace reset without power cycle
  - [ ] Verify topics appear correctly in `ros2 topic list`

---

### 📋 Additional TODO Items

- [ ] Add configuration file support (store namespace on Teensy EEPROM)
- [ ] Implement emergency stop via ROS 2 service
- [ ] Add battery voltage monitoring and low-battery LED warning
- [ ] Create calibration mode for encoder direction testing
- [ ] Add runtime parameter updates (wheel diameter, base width, etc.)

---

## 7. How to Contribute / Debug

### Common Issues

#### Robot moves backward when commanded forward
- In `set_motor_speeds`, swap `HIGH` and `LOW` for the direction pins
- Then, check if encoder counts go negative. If so, swap `PIN_ENC_A` and `PIN_ENC_B`

#### Robot spins opposite to command
- In `set_motor_speeds`, swap the `+` and `-` signs for the `angular_z` calculation

#### IMU not working
- Ensure you are using `Wire2` (Pins 24/25) as defined in `bno08x.begin_I2C`

### Testing

To verify everything is working, open a terminal on your ROS 2 host:

#### 1. Check connection

```bash
ros2 node list
# Should show /base_mcu_node
```

#### 2. Watch Odometry

```bash
ros2 topic echo /odom/unfiltered
```

#### 3. Teleoperation (Drive manually)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Quick Reference

| Task | Command |
|------|---------|
| Build firmware | Click **Build** in PlatformIO |
| Upload to Teensy | Click **Upload** in PlatformIO |
| Run micro-ROS agent (Docker) | `sudo docker run -it --rm -v /dev:/dev --privileged microros/micro-ros-agent:jazzy serial --dev /dev/ttyACM0 -b 6000000` |
| Check Teensy connection | `ls /dev/ttyACM*` |
| List ROS 2 nodes | `ros2 node list` |
| Test teleoperation | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` |

---

## Architecture Diagram

```plaintext
┌────────────────────────────────────────────────────────────┐
│                    ROS 2 Host (Raspberry Pi)               │
│                                                            │
│  ┌──────────────┐    ┌─────────────┐    ┌────────────────┐ │
│  │ Navigation   │───▶│ /cmd_vel    │───▶│  micro-ROS     │ │
│  │   Stack      │    │             │    │    Agent       │ │
│  └──────────────┘    └─────────────┘    └────────┬───────┘ │
│                                                  │         │
│  ┌──────────────┐    ┌─────────────┐             │         │
│  │  Sensor      │◀───│ /odom       │◀────────────┤         │
│  │  Fusion      │◀───│ /imu        │             │         │
│  └──────────────┘    └─────────────┘             │         │
└──────────────────────────────────────────────────┼─────────┘
                                                   │
                                          USB Serial (6 Mbps)
                                                   │
┌──────────────────────────────────────────────────┼─────────┐
│                    Teensy 4.1 Firmware           │         │
│                                                  │         │
│  ┌──────────────┐    ┌─────────────┐    ┌───────▼───────┐  │
│  │ Motor Driver │◀───│ PWM/DIR     │◀───│  main.cpp     │  │
│  │ (Cytron)     │    │             │    │               │  │
│  └──────────────┘    └─────────────┘    │  - Odometry   │  │
│                                         │  - IMU Reader │  │
│  ┌──────────────┐    ┌─────────────┐    │  - Motor Ctrl │  │
│  │  Encoders    │───▶│ Quadrature  │───▶│               │  │
│  │  (Left/Right)│    │   Reading   │    └──────┬────────┘  │
│  └──────────────┘    └─────────────┘           │           │
│                                                │           │
│  ┌──────────────┐    ┌─────────────┐           │           │
│  │  BNO085 IMU  │───▶│   I2C Wire2 │───────────┘           │
│  └──────────────┘    └─────────────┘                       │
└────────────────────────────────────────────────────────────┘
```