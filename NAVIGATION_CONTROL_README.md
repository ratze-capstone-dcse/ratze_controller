# Navigation Control Implementation

This document describes the wall-centering and discrete turning features added to the RATZE micromouse controller.

## Overview

Two major navigation features have been implemented:

1. **Proportional (Kp) Wall Centering Control** - Keeps the robot centered between maze walls while moving forward
2. **Discrete Turning with IMU+Encoder Fusion** - Precise angular turns using both encoder odometry and IMU heading

---

## 1. Wall Centering Control

### How It Works

The wall centering system uses Time-of-Flight (ToF) distance sensors on the left and right sides of the robot to maintain equal distance from both walls while moving forward.

### Algorithm

```
error = left_distance - right_distance
correction = Kp * error
```

- **Positive error**: Robot is too close to left wall → turns right
- **Negative error**: Robot is too close to right wall → turns left

### Key Features

- **Dual-wall centering**: Centers between two walls when both are detected
- **Single-wall following**: Maintains constant distance when only one wall is visible
- **Automatic activation**: Only applies when moving forward (cmd_vel.x > 0.1 m/s)
- **Tunable gain**: Adjust `KP_CENTERING` in `navigation_control.h` for responsiveness

### Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `KP_CENTERING` | 0.5 | Proportional gain (increase for stronger correction) |
| `CENTERING_MAX_CORRECTION` | 50 | Maximum PWM correction (prevents aggressive turns) |
| `WALL_DETECTION_THRESHOLD` | 300mm | Maximum distance to consider wall valid |
| `SENSOR_LEFT_SIDE` | 0 | Left sensor index |
| `SENSOR_RIGHT_SIDE` | 6 | Right sensor index |

### Tuning Guide

1. **Too sluggish?** Increase `KP_CENTERING` (try 0.7-1.0)
2. **Too oscillatory?** Decrease `KP_CENTERING` (try 0.3-0.4)
3. **Aggressive corrections?** Reduce `CENTERING_MAX_CORRECTION` to 30-40

---

## 2. Discrete Turning System

### Supported Turns

The system supports discrete angular turns with high precision:

- **±90°** - Quarter turns (primary maze navigation)
- **±45°** - Diagonal adjustments
- **±5°** - Fine heading corrections

### How It Works

The turning system fuses two independent measurements:

1. **IMU Heading** (`rad_yaw` from BNO055) - Absolute orientation
2. **Encoder Odometry** - Relative rotation calculated from wheel movement

#### Fusion Algorithm

```
imu_progress = 1.0 - |current_error| / |target_angle|
encoder_progress = avg_encoder_ticks / expected_ticks
fused_progress = 0.7 * imu_progress + 0.3 * encoder_progress
```

- 70% weight on IMU (better absolute accuracy)
- 30% weight on encoders (smooth progress tracking)

### Turn Execution Flow

1. **Initialize**
   - Record initial yaw and encoder positions
   - Calculate target heading and expected encoder ticks
   - Stop any current motion

2. **Execute Turn**
   - Set angular velocity (`cmd_vel.w`)
   - Continuously update IMU and encoders
   - Monitor fused progress

3. **Approach Phase** (progress > 80%)
   - Reduce speed to 40% for precision
   - Prevents overshooting

4. **Completion Check**
   - Turn stops when error < 2° (configurable)
   - Timeout after 5 seconds (safety)

### Key Functions

```cpp
// Generic turn by any angle
bool discrete_turn(float target_angle_deg);

// Convenience functions
bool turn_right_90();  // 90° clockwise
bool turn_left_90();   // 90° counter-clockwise
bool turn_right_45();  // 45° clockwise
bool turn_left_45();   // 45° counter-clockwise
bool turn_right_5();   // 5° clockwise
bool turn_left_5();    // 5° counter-clockwise
```

### Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `TURN_TOLERANCE_DEG` | 2.0° | Acceptable heading error |
| `TURN_TIMEOUT_MS` | 5000ms | Maximum turn duration |
| `TURN_SPEED` | 0.5 rad/s | Angular velocity during turn |

### Tuning Guide

1. **Overshooting?** 
   - Decrease `TURN_SPEED` (try 0.3-0.4 rad/s)
   - Increase approach phase threshold (change `0.8` to `0.7`)

2. **Too slow?**
   - Increase `TURN_SPEED` (try 0.7-0.8 rad/s)
   - Increase `TURN_TOLERANCE_DEG` (try 3.0°)

3. **Encoder drift?**
   - Adjust fusion weights (try 0.8 IMU, 0.2 encoder)
   - Verify `WHEEL_TRACK` dimension is accurate

---

## Serial Command Interface

### New Commands

#### Turn Command
```
T:angle
```
- **Example**: `T:90` (turn left 90°)
- **Example**: `T:-45` (turn right 45°)
- **Response**: `ACK:T:SUCCESS` or `ACK:T:FAILED`

#### Enable/Disable Centering
```
N:value
```
- **Example**: `N:1` (enable centering)
- **Example**: `N:0` (disable centering)
- **Response**: `ACK:N`

#### Set Centering Kp
```
K:value
```
- **Example**: `K:0.7` (set Kp to 0.7)
- **Response**: `ACK:K`
- **Note**: Currently requires code modification to make `KP_CENTERING` mutable

---

## Helper Functions

### Angle Utilities

```cpp
// Normalize angle to [-π, π]
float normalize_angle(float angle);

// Convert between degrees and radians
float deg_to_rad(float degrees);
float rad_to_deg(float radians);
```

### Distance/Encoder Conversions

```cpp
// Convert encoder ticks to meters
float ticks_to_distance(long ticks);

// Convert meters to encoder ticks
long distance_to_ticks(float distance);

// Calculate arc length for a turn angle
float angle_to_arc_length(float angle_rad);

// Calculate encoder ticks needed for turn
long angle_to_ticks(float angle_rad);
```

### Value Bounding

```cpp
// Constrain value to range
void bound_value(float &value, float min, float max);
```

---

## Integration Example

### Simple Forward Movement with Centering

```cpp
void setup() {
    setupFirmware();
    centering_enabled = true;
}

void loop() {
    // Move forward - centering automatically applies
    cmd_vel_.x = 0.5;  // 0.5 m/s forward
    cmd_vel_.w = 0.0;  // No rotation
    
    motor_loop_with_centering();  // Executes with wall centering
}
```

### Maze Navigation Pattern

```cpp
void navigate_maze() {
    // Move forward with centering
    cmd_vel_.x = 0.5;
    cmd_vel_.w = 0.0;
    delay(2000);  // Move for 2 seconds
    
    // Stop
    cmd_vel_.x = 0.0;
    cmd_vel_.w = 0.0;
    delay(100);
    
    // Turn right 90 degrees
    bool success = turn_right_90();
    if (!success) {
        Serial.println("Turn failed!");
        return;
    }
    
    // Continue forward
    cmd_vel_.x = 0.5;
    cmd_vel_.w = 0.0;
}
```

---

## System Parameters Reference

### Robot Dimensions
- **Encoder**: 100 ticks/revolution
- **Wheel radius**: 0.035 m (35mm)
- **Wheelbase**: 0.15 m (150mm) - front to back
- **Wheel track**: 0.26 m (260mm) - left to right

### Sensor Configuration
- **7 ToF sensors** (VL53L0X via TCA9548A multiplexer)
- Sensor indices: 0-6
- Default side sensors: 0 (left), 6 (right)

### IMU Configuration
- **BNO055** in NDOF mode
- Output: `rad_yaw` in radians [-π, π]
- Update rate: 20 Hz

---

## Troubleshooting

### Problem: Robot oscillates while centering

**Solution:**
1. Reduce `KP_CENTERING` to 0.3-0.4
2. Reduce `CENTERING_MAX_CORRECTION` to 30-40
3. Check ToF sensor alignment and calibration

### Problem: Turns are inaccurate

**Solution:**
1. Verify `WHEEL_TRACK` measurement (0.26m)
2. Check encoder wiring and direction
3. Calibrate IMU fully (magnetometer critical)
4. Adjust fusion weights (try 0.8 IMU, 0.2 encoder)

### Problem: Centering not working

**Solution:**
1. Verify sensor indices (`SENSOR_LEFT_SIDE`, `SENSOR_RIGHT_SIDE`)
2. Check ToF sensor readings with `printToFReadings()`
3. Ensure `centering_enabled = true`
4. Confirm forward movement: `cmd_vel_.x > 0.1`

### Problem: Timeout during turns

**Solution:**
1. Increase `TURN_TIMEOUT_MS` to 8000-10000ms
2. Check motor power and PWM signals
3. Verify PID gains are set correctly
4. Ensure battery is charged

---

## Performance Metrics

### Expected Performance

| Metric | Target | Notes |
|--------|--------|-------|
| Turn accuracy | ±2° | With calibrated IMU |
| Turn completion time | 1-3 seconds | Depends on angle |
| Centering error | ±10mm | With Kp=0.5 |
| Centering response time | 200-500ms | Wall detection to correction |

---

## Future Enhancements

### Potential Improvements

1. **PID Wall Centering**
   - Add integral term for steady-state error
   - Add derivative term for damping oscillations

2. **Adaptive Speed Control**
   - Slow down before turns
   - Accelerate gradually after turns

3. **Front Wall Detection**
   - Emergency stop before collision
   - Distance-based approach speed

4. **Dead Reckoning**
   - Track global position using encoders + IMU
   - Enable return-to-start navigation

5. **Dynamic Kp Adjustment**
   - Higher Kp at low speeds
   - Lower Kp at high speeds

---

## Code Architecture

```
include/
├── navigation_control.h     # Wall centering + discrete turning
├── firmware_control.h       # Command processing + main loop
├── motor_control.h          # PID velocity control
├── motor_driver.h           # Low-level motor + encoder functions
├── tof_main.h               # ToF sensor management
├── bno055-new.h             # IMU data extraction
└── header.h                 # Constants + data structures
```

---

## Contact & Support

For questions or issues related to this implementation:
- Check sensor calibration first
- Review tuning parameters
- Enable debug output to monitor behavior
- Consult system parameter reference

**Version**: 1.0  
**Date**: 2025-10-18  
**Compatible with**: Arduino ESP32, PlatformIO
