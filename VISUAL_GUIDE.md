# Navigation Control System - Visual Guide

## System Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    NAVIGATION CONTROL SYSTEM                 │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────────┐
        │      firmware_control.h (main loop)      │
        │  - Serial command processing             │
        │  - Sensor data streaming                 │
        │  - Command timeout handling              │
        └────────────┬────────────────┬────────────┘
                     │                │
          ┌──────────▼──────┐   ┌────▼────────────────┐
          │ Centering ON?   │   │ Turn Command?       │
          └──────────┬──────┘   └────┬────────────────┘
                     │               │
            ┌────────▼────────┐      │
            │ motor_loop_with_│      │
            │   centering()   │      │
            └────────┬────────┘      │
                     │               │
      ┌──────────────▼───────────────▼──────────────┐
      │       navigation_control.h                   │
      │  ┌──────────────┐    ┌─────────────────┐   │
      │  │ Wall         │    │ Discrete        │   │
      │  │ Centering    │    │ Turning         │   │
      │  └──────┬───────┘    └────────┬────────┘   │
      │         │                     │             │
      │    ┌────▼───────┐      ┌──────▼─────┐     │
      │    │ Kp Control │      │ IMU+Encoder │     │
      │    │ Algorithm  │      │   Fusion    │     │
      │    └────┬───────┘      └──────┬─────┘     │
      └─────────┼──────────────────────┼───────────┘
                │                      │
      ┌─────────▼──────────────────────▼───────────┐
      │         motor_driver.h                      │
      │  - PWM generation (sendPWM)                 │
      │  - Motor direction control                  │
      │  - Encoder reading                          │
      └─────────┬───────────────────────────────────┘
                │
      ┌─────────▼──────────────────────────┐
      │    Hardware (Motors + Sensors)      │
      │  - 4× DC Motors                     │
      │  - 4× Encoders                      │
      │  - 7× ToF Sensors                   │
      │  - 1× IMU (BNO055)                  │
      └─────────────────────────────────────┘
```

---

## Wall Centering Control Flow

```
                    ┌─────────────────┐
                    │ Moving Forward? │
                    └────────┬────────┘
                             │ YES
                    ┌────────▼────────┐
                    │ Read Side       │
                    │ ToF Sensors     │
                    │ (Left & Right)  │
                    └────────┬────────┘
                             │
              ┌──────────────┼──────────────┐
              │              │              │
    ┌─────────▼──────┐  ┌───▼────┐  ┌──────▼───────┐
    │ Both Walls     │  │ Left   │  │ Right Only   │
    │ Detected       │  │ Only   │  │ Detected     │
    └─────────┬──────┘  └───┬────┘  └──────┬───────┘
              │             │              │
    ┌─────────▼──────┐  ┌───▼────────┐  ┌──▼───────────┐
    │ Center Between │  │ Follow     │  │ Follow       │
    │ error = L - R  │  │ Left Wall  │  │ Right Wall   │
    └─────────┬──────┘  └───┬────────┘  └──┬───────────┘
              └─────────────┼──────────────┘
                            │
                   ┌────────▼────────┐
                   │ Calculate:      │
                   │ correction =    │
                   │   Kp × error    │
                   └────────┬────────┘
                            │
                   ┌────────▼────────┐
                   │ Bound to        │
                   │ [-50, +50] PWM  │
                   └────────┬────────┘
                            │
              ┌─────────────┴─────────────┐
              │                           │
    ┌─────────▼────────┐       ┌─────────▼────────┐
    │ Left Motor PWM   │       │ Right Motor PWM  │
    │ += correction    │       │ -= correction    │
    └──────────────────┘       └──────────────────┘
```

### Centering Logic

```
If both walls detected:
    error = left_distance - right_distance
    
    error > 0 → Too close to left wall
              → Correction positive (turn right)
              → Left motor speeds up
              → Right motor slows down
    
    error < 0 → Too close to right wall
              → Correction negative (turn left)
              → Left motor slows down
              → Right motor speeds up

If only left wall:
    error = left_distance - desired_distance (150mm)
    
If only right wall:
    error = desired_distance (150mm) - right_distance
```

---

## Discrete Turn State Machine

```
                    ┌──────────────┐
                    │  START TURN  │
                    └──────┬───────┘
                           │
                    ┌──────▼───────┐
                    │ Record:      │
                    │ - Initial yaw│
                    │ - Initial enc│
                    │ Calculate:   │
                    │ - Target yaw │
                    │ - Target enc │
                    └──────┬───────┘
                           │
                    ┌──────▼───────┐
                    │ Set angular  │
                    │ velocity     │
                    │ (TURN_SPEED) │
                    └──────┬───────┘
                           │
            ┌──────────────▼──────────────┐
            │      TURNING LOOP            │
            │                              │
            │  ┌────────────────────┐     │
            │  │ Update IMU & Enc   │     │
            │  └─────────┬──────────┘     │
            │            │                 │
            │  ┌─────────▼──────────┐     │
            │  │ Calculate errors:  │     │
            │  │ - IMU error        │     │
            │  │ - Encoder progress │     │
            │  └─────────┬──────────┘     │
            │            │                 │
            │  ┌─────────▼──────────┐     │
            │  │ Fuse measurements: │     │
            │  │ 70% IMU + 30% Enc  │     │
            │  └─────────┬──────────┘     │
            │            │                 │
            │  ┌─────────▼──────────┐     │
            │  │ Progress > 80%?    │     │
            │  │ YES → Reduce speed │     │
            │  │ NO  → Keep speed   │     │
            │  └─────────┬──────────┘     │
            │            │                 │
            │  ┌─────────▼──────────┐     │
            │  │ Error < 2°?        │───NO──┐
            │  └─────────┬──────────┘       │
            │            │ YES               │
            └────────────┼───────────────────┘
                         │                   │
                  ┌──────▼────────┐    ┌─────▼──────┐
                  │  TURN COMPLETE│    │ Timeout?   │
                  │  Stop motors  │    └─────┬──────┘
                  │  Return true  │          │ YES
                  └───────────────┘    ┌─────▼──────┐
                                       │ Stop motors│
                                       │ Return false│
                                       └────────────┘
```

### Turn Direction Logic

```
Target angle > 0:  Turn LEFT (counter-clockwise)
    cmd_vel.w = +TURN_SPEED
    Left motors: BACKWARD
    Right motors: FORWARD

Target angle < 0:  Turn RIGHT (clockwise)
    cmd_vel.w = -TURN_SPEED
    Left motors: FORWARD
    Right motors: BACKWARD
```

---

## Sensor Fusion Algorithm

```
┌────────────────────────────────────────────────────┐
│             ENCODER MEASUREMENT                     │
│                                                     │
│  Initial: count_left₀, count_right₀                │
│  Current: count_left, count_right                  │
│                                                     │
│  Δleft = |count_left - count_left₀|               │
│  Δright = |count_right - count_right₀|            │
│  Δavg = (Δleft + Δright) / 2                      │
│                                                     │
│  Encoder Progress = Δavg / expected_ticks         │
└─────────────────┬──────────────────────────────────┘
                  │
                  │ 30% weight
                  │
        ┌─────────▼──────────┐
        │  FUSED PROGRESS    │
        │  = 0.7×IMU_prog +  │
        │    0.3×Enc_prog    │
        └─────────┬──────────┘
                  │
                  │ 70% weight
                  │
┌─────────────────▼──────────────────────────────────┐
│              IMU MEASUREMENT                        │
│                                                     │
│  Initial: yaw₀                                     │
│  Current: yaw                                      │
│  Target:  yaw_target = normalize(yaw₀ + Δangle)   │
│                                                     │
│  Error = normalize(yaw_target - yaw)              │
│  IMU Progress = 1 - |error| / |target_angle|      │
└─────────────────────────────────────────────────────┘
```

### Why This Fusion?

| Sensor | Strength | Weakness | Weight |
|--------|----------|----------|--------|
| IMU | Absolute heading, drift-free (calibrated) | Noisy, affected by magnetic fields | 70% |
| Encoder | Smooth, repeatable | Cumulative error, slippage | 30% |

**Result:** Accurate absolute positioning with smooth progress tracking

---

## Robot Coordinate System

```
                    ┌─────────┐
                    │  FRONT  │
                    └─────────┘
                        
    Left Sensor             Right Sensor
         ○                       ○
         
         
    M2 ●─────────────────────● M1
       │                     │
       │      ROBOT          │
       │    (Top View)       │
       │                     │
    M4 ●─────────────────────● M3
    
    
         ○                       ○
    (Enc M4)               (Enc M3)


Wheel numbering:
- M1, M3: Right side motors
- M2, M4: Left side motors

Coordinate system:
- X-axis: Forward (positive = forward motion)
- Y-axis: Left (positive = left translation)
- Yaw (θ): Counter-clockwise rotation
    θ = 0°    → Facing forward
    θ = 90°   → Facing left
    θ = -90°  → Facing right
    θ = 180°  → Facing backward

ToF Sensor layout (assumed):
    Sensor 0: Left side
    Sensor 3: Front center
    Sensor 6: Right side
```

---

## PWM and Direction Control

```
┌──────────────────────────────────────────────────┐
│        MOTOR DIRECTION MAPPING                   │
└──────────────────────────────────────────────────┘

FORWARD:
    M1: IN1=HIGH, IN2=LOW   (Right front)
    M2: IN1=HIGH, IN2=LOW   (Left front)
    M3: IN1=LOW,  IN2=HIGH  (Right rear)
    M4: IN1=LOW,  IN2=HIGH  (Left rear)

BACKWARD:
    M1: IN1=LOW,  IN2=HIGH  (Right front)
    M2: IN1=LOW,  IN2=HIGH  (Left front)
    M3: IN1=HIGH, IN2=LOW   (Right rear)
    M4: IN1=HIGH, IN2=LOW   (Left rear)

TURN LEFT (CCW):
    M1: IN1=HIGH, IN2=LOW   (Right forward)
    M2: IN1=LOW,  IN2=HIGH  (Left backward)
    M3: IN1=LOW,  IN2=HIGH  (Right backward)
    M4: IN1=HIGH, IN2=LOW   (Left forward)

TURN RIGHT (CW):
    M1: IN1=LOW,  IN2=HIGH  (Right backward)
    M2: IN1=HIGH, IN2=LOW   (Left forward)
    M3: IN1=HIGH, IN2=LOW   (Right forward)
    M4: IN1=LOW,  IN2=HIGH  (Left backward)

PWM (Speed):
    EN_M1, EN_M2, EN_M3, EN_M4: 0-255
    (Applied to enable pins)
```

---

## Timing Diagram

```
Time (ms) →
0         10        20        30        40        50
│─────────│─────────│─────────│─────────│─────────│
│         │         │         │         │         │
│ Motor   │ Motor   │ Motor   │ Motor   │ Motor   │ (100 Hz)
│ Loop    │ Loop    │ Loop    │ Loop    │ Loop    │
│  (10ms) │         │         │         │         │
└─────────┴─────────┴─────────┴─────────┴─────────┘

0              50             100            150
│──────────────│──────────────│──────────────│
│              │              │              │
│  Update IMU  │  Update IMU  │  Update IMU  │ (20 Hz)
│  & ToF       │  & ToF       │  & ToF       │
│  (50ms)      │              │              │
└──────────────┴──────────────┴──────────────┘

0                   100                  200
│───────────────────│────────────────────│
│                   │                    │
│  Send Serial Data │  Send Serial Data  │ (10 Hz)
│  (100ms)          │                    │
└───────────────────┴────────────────────┘
```

**Loop Priorities:**
1. Motor control: 100 Hz (highest priority)
2. Sensor updates: 20 Hz (medium priority)
3. Serial communication: 10 Hz (low priority)

---

## Data Flow During Centering

```
┌──────────────┐
│ ToF Sensors  │
│ Left: 180mm  │
│ Right: 150mm │
└──────┬───────┘
       │
       │ Read distances
       ▼
┌──────────────────────┐
│ kp_centering_control │
│ error = 180 - 150    │
│      = 30mm          │
│ correction = Kp × 30 │
│           = 0.5 × 30 │
│           = 15 PWM   │
└──────┬───────────────┘
       │
       │ Apply correction
       ▼
┌─────────────────────────────┐
│ apply_centering_correction  │
│ Left PWM  = 100 + 15 = 115  │
│ Right PWM = 100 - 15 = 85   │
└──────┬──────────────────────┘
       │
       │ Send PWM
       ▼
┌──────────────────┐
│ sendPWM()        │
│ Set directions   │
│ Set PWM values   │
└──────┬───────────┘
       │
       │ Hardware control
       ▼
┌──────────────────┐
│ Motors           │
│ Left: faster     │
│ Right: slower    │
│ → Robot turns    │
│   right slightly │
└──────────────────┘
```

---

## Decision Tree: When to Use Each Feature

```
                    User wants robot to...
                            │
        ┌───────────────────┼───────────────────┐
        │                   │                   │
    Follow walls       Make precise        Navigate maze
    while moving        turn angle         autonomously
        │                   │                   │
        ▼                   ▼                   ▼
┌───────────────┐   ┌──────────────┐    ┌─────────────┐
│ Enable        │   │ Use          │    │ Combine:    │
│ Centering:    │   │ discrete_    │    │ - Centering │
│               │   │ turn()       │    │ - Turning   │
│ centering_    │   │              │    │ - Front     │
│ enabled=true  │   │ Example:     │    │   sensor    │
│               │   │ turn_left_90 │    │             │
│ Move forward  │   │ turn_right_  │    │ Logic:      │
│ cmd_vel.x>0   │   │   45()       │    │ - Move fwd  │
│               │   │              │    │ - Check     │
│ Auto-centers! │   │ Returns      │    │   front     │
└───────────────┘   │ true/false   │    │ - If wall,  │
                    └──────────────┘    │   turn      │
                                        │ - Repeat    │
                                        └─────────────┘
```

---

## Error Scenarios and Recovery

```
Scenario 1: ONE SENSOR FAILS
    ToF Left: 0mm (invalid)
    ToF Right: 150mm (valid)
    
    → Single-wall following mode
    → Follow right wall at 150mm
    → Robot continues safely

Scenario 2: BOTH SENSORS FAIL
    ToF Left: 0mm (invalid)
    ToF Right: 0mm (invalid)
    
    → Return correction = 0
    → No centering applied
    → Robot continues straight (dead reckoning)

Scenario 3: TURN TIMEOUT
    Turn requested: 90°
    After 5 seconds: Error still > 2°
    
    → Stop motors
    → Return false
    → User code handles failure

Scenario 4: ENCODER SLIP
    Turn 90°:
      IMU says: 85° complete
      Encoders say: 92° complete
      Fused: 0.7×85 + 0.3×92 = 87°
    
    → Trust IMU more (70% weight)
    → Continue until IMU confirms target

Scenario 5: IMU DRIFT (uncalibrated)
    Magnetometer: 0/3
    Turn 90°: Actually turns 75°
    
    → SOLUTION: Calibrate IMU before use!
    → Mag status must be 3/3
```

---

## Performance Graphs (Conceptual)

### Centering Response

```
Distance
from 
center
  ↑
  │     ┌─╮
20mm    │ │╲
  │     │ │ ╲___
10mm    │ │     ╲___
  │     │ │         ╲___
 0mm────┼─┴─────────────────→ Time
  │     │
  │    Start centering
  │    (Kp=0.5)
  
  Response time: ~500ms
  Steady-state error: ±5mm
```

### Turn Progress

```
Angle
  ↑
90°─────────────────────────┐
  │                         │╲
  │                         │ ╲
  │                        ╱│  ╲___
45°│                   ____│ │      ╲___
  │              _____╱    │ │          ╲
  │         ____╱          │Slowdown
 0°────────────────────────┴──────────────→ Time
  │         1s            1.5s           2s
  │
  │ ←─ Full speed ──→←─Approach phase─→
  
  Total time: ~2 seconds for 90°
```

---

## Memory Layout

```
┌─────────────────────────────────────────┐
│          GLOBAL VARIABLES               │
├─────────────────────────────────────────┤
│ cmd_vel_ (8 bytes)                      │ Current velocity command
│ centering_enabled (1 byte)              │ Centering flag
│ countM1-4 (16 bytes)                    │ Encoder counts
│ tof_distances[7] (14 bytes)             │ ToF readings
│ rad_yaw, rad_pitch, rad_roll (12 bytes) │ IMU angles
│ PID objects (varies)                    │ PID controllers
├─────────────────────────────────────────┤
│          STACK (during turn)            │
├─────────────────────────────────────────┤
│ initial_yaw (4 bytes)                   │ Turn start state
│ target_yaw (4 bytes)                    │ Turn target
│ initial_encoder_left/right (8 bytes)    │ Encoder start
│ expected_ticks (4 bytes)                │ Target ticks
│ Local variables (~20 bytes)             │ Loop vars
├─────────────────────────────────────────┤
│          CODE SPACE                     │
├─────────────────────────────────────────┤
│ navigation_control.h (~3KB)             │ New functions
│ firmware_control.h (~2KB)               │ Modified
│ Existing code (~varies)                 │ Unchanged
└─────────────────────────────────────────┘

Total added memory: ~4KB flash, ~100 bytes RAM
```

---

## Testing Checklist Flowchart

```
┌──────────────┐
│ START TESTING│
└──────┬───────┘
       │
       ▼
┌──────────────────┐
│ Hardware Check   │
│ □ Motors spin    │
│ □ Encoders count │
│ □ ToF reads OK   │
│ □ IMU calibrated │
└──────┬───────────┘
       │ All pass
       ▼
┌──────────────────┐
│ Static Tests     │
│ □ Sensor         │
│   validation     │
│ □ Print all data │
│ □ Verify values  │
└──────┬───────────┘
       │ Values OK
       ▼
┌──────────────────┐
│ Centering Test   │
│ □ Enable         │
│ □ Move forward   │
│ □ Check          │
│   corrections    │
└──────┬───────────┘
       │ Centers OK
       ▼
┌──────────────────┐
│ Turn Tests       │
│ □ 90° left       │
│ □ 90° right      │
│ □ Measure error  │
│ □ Tune if needed │
└──────┬───────────┘
       │ Turns OK
       ▼
┌──────────────────┐
│ Integration Test │
│ □ Square path    │
│ □ Maze pattern   │
│ □ Monitor        │
│   performance    │
└──────┬───────────┘
       │ All pass
       ▼
┌──────────────┐
│ READY FOR USE│
└──────────────┘
```

---

**This visual guide complements the text documentation.**  
**For implementation details, see NAVIGATION_CONTROL_README.md**  
**For parameter tuning, see TUNING_GUIDE.md**
