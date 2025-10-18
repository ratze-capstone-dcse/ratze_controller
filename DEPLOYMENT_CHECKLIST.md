# Deployment Checklist

Use this checklist to ensure the navigation control system is properly configured and tested before deployment.

## Pre-Deployment Configuration

### 1. Hardware Verification
- [ ] All 4 motors are properly connected
- [ ] All 4 encoders are connected and wired correctly
- [ ] All 7 ToF sensors are connected via TCA9548A multiplexer
- [ ] BNO055 IMU is connected via I2C
- [ ] Battery is fully charged (>11V recommended)
- [ ] Motor driver power supply is adequate
- [ ] All connectors are secure

### 2. Physical Measurements
- [ ] Measure actual wheel radius (update `WHEEL_RADIUS` in header.h if different from 0.035m)
- [ ] Measure actual wheel track distance (update `WHEEL_TRACK` in header.h if different from 0.26m)
- [ ] Measure actual wheelbase (update `WHEEL_BASE` in header.h if different from 0.15m)
- [ ] Verify encoder resolution is 100 ticks/rev (update `TICKS_PER_REV` if different)
- [ ] Document measurements:
  ```
  Wheel radius:    ___________ m
  Wheel track:     ___________ m
  Wheelbase:       ___________ m
  Encoder ticks:   ___________ per revolution
  ```

### 3. Sensor Mapping
- [ ] Identify which ToF sensor is mounted on the left side
- [ ] Identify which ToF sensor is mounted on the right side
- [ ] Identify which ToF sensor is front-facing (if any)
- [ ] Update sensor indices in `navigation_control.h`:
  ```cpp
  #define SENSOR_LEFT_SIDE ___    // Your left sensor number
  #define SENSOR_RIGHT_SIDE ___   // Your right sensor number
  ```
- [ ] Document sensor layout:
  ```
  Sensor 0: _____________
  Sensor 1: _____________
  Sensor 2: _____________
  Sensor 3: _____________
  Sensor 4: _____________
  Sensor 5: _____________
  Sensor 6: _____________
  ```

### 4. IMU Calibration
- [ ] Power on robot and keep still for 30 seconds
- [ ] Move IMU in figure-8 pattern for magnetometer calibration
- [ ] Check calibration status via serial output
- [ ] Wait for magnetometer calibration: 3/3
- [ ] Wait for system calibration: 3/3
- [ ] Do NOT proceed until fully calibrated!

### 5. Code Configuration
- [ ] Include `navigation_control.h` in your main firmware file
- [ ] Set initial centering state (`centering_enabled = true/false`)
- [ ] Review and adjust tuning parameters if needed:
  - [ ] `KP_CENTERING` (default: 0.5)
  - [ ] `TURN_SPEED` (default: 0.5 rad/s)
  - [ ] `TURN_TOLERANCE_DEG` (default: 2.0°)
- [ ] Compile code and check for errors
- [ ] Upload to robot

---

## Initial Testing Phase

### Phase 1: Static Sensor Tests

#### Test 1.1: ToF Sensor Validation
- [ ] Place robot in maze or between walls
- [ ] Run sensor validation test or send `G` command
- [ ] Verify all ToF sensors report reasonable distances
- [ ] Check left sensor reads correctly
- [ ] Check right sensor reads correctly
- [ ] Check for any sensor showing 0 or 8190 (error values)
- [ ] Document any sensor issues:
  ```
  Problematic sensors: _______________
  Actions taken: _______________
  ```

#### Test 1.2: IMU Reading Test
- [ ] Robot should be still
- [ ] Check yaw reading (should be stable)
- [ ] Rotate robot 90° by hand
- [ ] Verify yaw changes by ~90° (~1.57 rad)
- [ ] Check roll and pitch are reasonable
- [ ] IMU readings acceptable? (Yes/No): _____

#### Test 1.3: Encoder Test
- [ ] Reset encoders (send `E` command)
- [ ] Manually spin each wheel forward
- [ ] Verify encoder counts increase for all wheels
- [ ] Spin backwards and verify counts decrease
- [ ] Document results:
  ```
  M1 (right front):  Working? Yes/No
  M2 (left front):   Working? Yes/No
  M3 (right rear):   Working? Yes/No
  M4 (left rear):    Working? Yes/No
  ```

### Phase 2: Motor Tests

#### Test 2.1: Forward Movement
- [ ] Place robot in open area
- [ ] Send command: `F:0.3:0.0` (slow forward)
- [ ] Verify robot moves forward
- [ ] Verify robot moves relatively straight
- [ ] Send command: `S` (stop)
- [ ] Robot stops cleanly? (Yes/No): _____
- [ ] Any significant drift? (Yes/No): _____

#### Test 2.2: Motor Balance
- [ ] Move forward for 2 meters
- [ ] Measure deviation from straight line
- [ ] If deviation > 10cm, motor balance needs adjustment
- [ ] Document:
  ```
  Deviation: ________ cm
  Drifts left/right? ___________
  ```

### Phase 3: Wall Centering Tests

#### Test 3.1: Static Centering Check
- [ ] Place robot between walls (not centered)
- [ ] Enable centering: `N:1`
- [ ] Send command: `F:0.2:0.0` (slow forward)
- [ ] Observe robot behavior:
  - [ ] Robot corrects toward center
  - [ ] Robot doesn't oscillate wildly
  - [ ] Robot reaches stable centered position
- [ ] Stop robot: `S`

#### Test 3.2: Centering While Moving
- [ ] Start robot off-center in corridor
- [ ] Enable centering: `N:1`
- [ ] Command: `F:0.3:0.0`
- [ ] Let robot move forward 3-4 meters
- [ ] Observe behavior:
  - [ ] Robot moves toward center smoothly
  - [ ] No excessive oscillation
  - [ ] Final position is centered
- [ ] If oscillating, reduce `KP_CENTERING`
- [ ] Document performance:
  ```
  Oscillation? (None/Slight/Severe): ___________
  Time to center: ________ seconds
  Final centering error: ±________ mm
  ```

#### Test 3.3: Centering Response Test
- [ ] Robot moving centered in corridor
- [ ] Manually push robot to one side (while moving)
- [ ] Observe correction response
- [ ] Robot should smoothly return to center
- [ ] Response acceptable? (Yes/No): _____

### Phase 4: Discrete Turning Tests

#### Test 4.1: 90° Left Turn
- [ ] Reset encoders: `E`
- [ ] Reset IMU: `C`
- [ ] Mark starting orientation on floor
- [ ] Command: `T:90`
- [ ] Wait for turn to complete
- [ ] Measure actual angle turned
- [ ] Document:
  ```
  Target: 90°
  Actual: ______°
  Error:  ______°
  Time:   ______ seconds
  Success? (Yes/No): _____
  ```

#### Test 4.2: 90° Right Turn
- [ ] Command: `T:-90`
- [ ] Measure actual angle turned
- [ ] Document:
  ```
  Target: -90°
  Actual: ______°
  Error:  ______°
  Time:   ______ seconds
  Success? (Yes/No): _____
  ```

#### Test 4.3: 45° Turns
- [ ] Test left 45°: `T:45`
  - Actual: ______° | Error: ______°
- [ ] Test right 45°: `T:-45`
  - Actual: ______° | Error: ______°

#### Test 4.4: Small Corrections
- [ ] Test left 5°: `T:5`
  - Actual: ______° | Error: ______°
- [ ] Test right 5°: `T:-5`
  - Actual: ______° | Error: ______°

#### Test 4.5: 360° Cumulative Test
- [ ] Perform 4 consecutive 90° right turns
- [ ] Robot should return to initial heading
- [ ] Measure final heading error
- [ ] Document:
  ```
  After 4×90° turns:
  Cumulative error: ______°
  If error > 5°, adjust WHEEL_TRACK
  ```

### Phase 5: Integration Tests

#### Test 5.1: Square Path
- [ ] Mark a square path on floor
- [ ] Run square path test (example 6)
- [ ] Observe performance:
  - [ ] Completes all 4 sides
  - [ ] All turns successful
  - [ ] Returns near starting position
- [ ] Measure final position error
- [ ] Document:
  ```
  Position error: ________ cm
  Heading error:  ________°
  Time to complete: ________ seconds
  ```

#### Test 5.2: Corridor Navigation
- [ ] Place robot in maze corridor
- [ ] Enable centering
- [ ] Move forward 3 meters with centering
- [ ] Verify smooth centered movement
- [ ] Performance acceptable? (Yes/No): _____

#### Test 5.3: Turn + Centering Sequence
- [ ] Start in corridor
- [ ] Move forward 1 meter (centered)
- [ ] Stop
- [ ] Turn 90° right
- [ ] Move forward 1 meter (centered)
- [ ] Verify:
  - [ ] Both straight sections are centered
  - [ ] Turn is accurate
  - [ ] Smooth transitions
- [ ] Successful? (Yes/No): _____

---

## Tuning Phase (If Needed)

### If Robot Oscillates While Centering
- [ ] Current `KP_CENTERING`: _______
- [ ] Reduce by 30%: New value: _______
- [ ] Test again
- [ ] Still oscillating? Reduce further
- [ ] Final working value: _______

### If Robot Overshoots Turns
- [ ] Current `TURN_SPEED`: _______ rad/s
- [ ] Reduce by 20%: New value: _______ rad/s
- [ ] Edit line 332 in navigation_control.h:
  - [ ] Change `0.8f` to `0.7f` (start slowdown earlier)
- [ ] Recompile and test
- [ ] Final working values:
  ```
  TURN_SPEED: _______ rad/s
  Slowdown threshold: _______
  ```

### If Turns Are Consistently Off
- [ ] Measure error pattern:
  - All turns short? → Decrease `WHEEL_TRACK`
  - All turns long? → Increase `WHEEL_TRACK`
- [ ] Current `WHEEL_TRACK`: _______ m
- [ ] Calculate adjustment:
  - Error: ______°
  - Adjustment factor: error/90 = _______
  - New `WHEEL_TRACK`: current × (1 ± factor) = _______ m
- [ ] Update in header.h
- [ ] Recompile and retest

### If One Side Drifts During Straight Movement
- [ ] Identify drift direction: Left / Right
- [ ] Check motor wiring and connections
- [ ] Consider PWM scaling:
  ```cpp
  // In header.h
  MOTOR_PWM_SCALE_RIGHT = _______  // Adjust to balance
  MOTOR_PWM_SCALE_LEFT = _______
  ```
- [ ] Retest and document final values

---

## Final Validation

### Performance Metrics Check
- [ ] Turn accuracy: ±___°  (Target: ±2°)
- [ ] Turn time (90°): ___s  (Target: 1-3s)
- [ ] Centering error: ±___mm  (Target: ±10mm)
- [ ] Centering response: ___ms  (Target: 200-500ms)
- [ ] Success rate (10 turns): ___/10  (Target: 10/10)

### Repeatability Test
- [ ] Perform same test 5 times
- [ ] Measure variation in results
- [ ] Document:
  ```
  Test: 90° turn
  Run 1: ______°
  Run 2: ______°
  Run 3: ______°
  Run 4: ______°
  Run 5: ______°
  
  Mean: ______°
  Std dev: ______°
  Max variation: ______°
  
  Acceptable (variation < 3°)? Yes/No: _____
  ```

### Battery Test
- [ ] Run full test suite
- [ ] Monitor battery voltage
- [ ] Note any performance degradation
- [ ] Document:
  ```
  Starting voltage: _____ V
  Ending voltage: _____ V
  Performance degraded below: _____ V
  ```

### Long-Duration Test
- [ ] Run continuous operation for 10 minutes
- [ ] Mix of movements and turns
- [ ] Check for:
  - [ ] Motor overheating
  - [ ] IMU drift
  - [ ] Sensor failures
  - [ ] Memory issues
- [ ] Any issues? Document: _______________

---

## Documentation

### Final Configuration Record
```
Date: __________
Robot ID: __________
Firmware Version: __________

Physical Parameters:
- WHEEL_RADIUS: _______ m
- WHEEL_TRACK: _______ m
- WHEEL_BASE: _______ m
- TICKS_PER_REV: _______

Sensor Configuration:
- SENSOR_LEFT_SIDE: Sensor #___
- SENSOR_RIGHT_SIDE: Sensor #___

Control Parameters:
- KP_CENTERING: _______
- CENTERING_MAX_CORRECTION: _______
- TURN_SPEED: _______ rad/s
- TURN_TOLERANCE_DEG: _______°

Motor PID:
- MOTOR_DRIVER_PID_KP: _______
- MOTOR_DRIVER_PID_KI: _______
- MOTOR_DRIVER_PID_KD: _______

Performance:
- Turn accuracy: ±_______°
- Centering error: ±_______mm
- Turn time: _______s (90°)

Notes:
_________________________________
_________________________________
_________________________________
```

### Known Issues Log
```
Issue 1: _________________________________
Status: _________________________________
Workaround: _________________________________

Issue 2: _________________________________
Status: _________________________________
Workaround: _________________________________
```

---

## Sign-Off

### Testing Complete
- [ ] All critical tests passed
- [ ] Parameters tuned and documented
- [ ] Performance meets requirements
- [ ] Known issues documented
- [ ] Robot ready for deployment

**Tested by:** _______________  
**Date:** _______________  
**Signature:** _______________

**Reviewed by:** _______________  
**Date:** _______________  
**Signature:** _______________

---

## Quick Re-Deployment Checklist

Use this for subsequent deployments after initial setup:

- [ ] Battery charged
- [ ] All connections secure
- [ ] IMU calibrated (mag 3/3, sys 3/3)
- [ ] Sensor validation test passed
- [ ] Test turn: 90° right
- [ ] Test centering: Forward in corridor
- [ ] Ready for operation

**Date:** ___________  
**Operator:** ___________  
**Result:** Pass / Fail

---

## Emergency Procedures

### If Robot Crashes Into Wall
1. [ ] Immediate stop: Send `S` command or power off
2. [ ] Check for physical damage
3. [ ] Re-run sensor validation
4. [ ] Re-calibrate IMU if necessary
5. [ ] Restart from Phase 1 tests

### If Robot Won't Center
1. [ ] Check `centering_enabled` flag
2. [ ] Verify sensor indices are correct
3. [ ] Print sensor readings (should see distances)
4. [ ] Check `cmd_vel_.x > 0.1`
5. [ ] Increase `KP_CENTERING` temporarily for testing

### If Turns Are Erratic
1. [ ] Check IMU calibration status
2. [ ] Re-calibrate magnetometer fully
3. [ ] Verify battery voltage (>11V)
4. [ ] Check encoder connections
5. [ ] Reduce `TURN_SPEED` temporarily

### If Motors Don't Respond
1. [ ] Check power connections
2. [ ] Verify motor driver is powered
3. [ ] Check PWM signals with oscilloscope
4. [ ] Test motors with simple commands (no PID)
5. [ ] Check for software exception/crash

---

**This checklist should be completed for each robot deployment.**  
**Keep this document with the robot for future reference.**
