# Quick Reference: Tuning Parameters

## Wall Centering Control

### Primary Tuning Parameters

| Parameter | Location | Default | Range | Effect |
|-----------|----------|---------|-------|--------|
| `KP_CENTERING` | navigation_control.h | 0.5 | 0.1 - 2.0 | Higher = stronger correction |
| `CENTERING_MAX_CORRECTION` | navigation_control.h | 50 | 20 - 100 | Maximum PWM adjustment |
| `WALL_DETECTION_THRESHOLD` | navigation_control.h | 300mm | 200 - 500mm | Max distance to detect wall |
| `MIN_FORWARD_SPEED` | navigation_control.h | 0.1 m/s | 0.05 - 0.2 | Min speed to enable centering |

### Tuning Tips

**Oscillating behavior:**
- Decrease `KP_CENTERING` → 0.3
- Decrease `CENTERING_MAX_CORRECTION` → 30

**Slow to center:**
- Increase `KP_CENTERING` → 0.8
- Increase `CENTERING_MAX_CORRECTION` → 70

**Ignoring walls:**
- Increase `WALL_DETECTION_THRESHOLD` → 400mm
- Check sensor indices are correct

---

## Discrete Turning

### Primary Tuning Parameters

| Parameter | Location | Default | Range | Effect |
|-----------|----------|---------|-------|--------|
| `TURN_SPEED` | navigation_control.h | 0.5 rad/s | 0.3 - 1.0 | Angular velocity during turn |
| `TURN_TOLERANCE_DEG` | navigation_control.h | 2.0° | 1.0 - 5.0 | Acceptable heading error |
| `TURN_TIMEOUT_MS` | navigation_control.h | 5000ms | 3000 - 10000 | Max time before abort |

### IMU/Encoder Fusion Weights

**Location:** `navigation_control.h` line ~328
```cpp
float fused_progress = 0.7 * imu_progress + 0.3 * encoder_progress;
```

| Scenario | Recommended | Reason |
|----------|-------------|--------|
| Calibrated IMU | 0.8 IMU, 0.2 Enc | Trust absolute heading |
| Slippery floor | 0.5 IMU, 0.5 Enc | Equal trust |
| Encoder drift | 0.9 IMU, 0.1 Enc | Rely on IMU |

### Approach Phase Speed Reduction

**Location:** `navigation_control.h` line ~332
```cpp
if (fused_progress > 0.8f) {
    float reduced_speed = TURN_SPEED * 0.4f;
```

| Threshold | Speed Multiplier | Best For |
|-----------|------------------|----------|
| 0.8 | 0.4 | Default (balanced) |
| 0.7 | 0.5 | Fast turns (may overshoot) |
| 0.9 | 0.3 | Precise turns (slower) |

### Tuning Tips

**Overshooting turns:**
- Decrease `TURN_SPEED` → 0.3 rad/s
- Start slowdown earlier → `0.7f` instead of `0.8f`
- Reduce approach speed → `0.3f` instead of `0.4f`

**Undershooting turns:**
- Increase `TURN_TOLERANCE_DEG` → 3.0°
- Increase `TURN_SPEED` → 0.7 rad/s
- Delay slowdown → `0.85f` instead of `0.8f`

**Slow turns (timeout):**
- Increase `TURN_SPEED` → 0.8 rad/s
- Increase `TURN_TIMEOUT_MS` → 8000ms
- Check battery voltage

**Inaccurate heading:**
- Verify `WHEEL_TRACK` = 0.26m (measure!)
- Calibrate IMU magnetometer fully
- Adjust fusion weights (try 0.8 IMU)

---

## Motor PID Control

### Location: header.h

| Parameter | Default | Range | Effect |
|-----------|---------|-------|--------|
| `MOTOR_DRIVER_PID_KP` | 200.0 | 50 - 400 | Proportional gain |
| `MOTOR_DRIVER_PID_KI` | 70.0 | 10 - 150 | Integral gain |
| `MOTOR_DRIVER_PID_KD` | 30.0 | 5 - 80 | Derivative gain |

**Modify via serial:**
```
P:200    # Set Kp
I:70     # Set Ki
D:30     # Set Kd
```

### PID Tuning Tips

**Motors oscillate:**
- Decrease Kp → 150
- Decrease Kd → 20

**Slow response:**
- Increase Kp → 250
- Increase Kd → 40

**Steady-state error:**
- Increase Ki → 100

---

## Robot Physical Parameters

### Critical Measurements (header.h)

| Parameter | Value | How to Measure |
|-----------|-------|----------------|
| `TICKS_PER_REV` | 100 | Encoder specification |
| `WHEEL_RADIUS` | 0.035m | Measure wheel diameter ÷ 2 |
| `WHEEL_BASE` | 0.15m | Front wheel to rear wheel center |
| `WHEEL_TRACK` | 0.26m | Left wheel to right wheel center |

**IMPORTANT:** If turns are consistently off by a constant amount:
1. Measure `WHEEL_TRACK` accurately
2. Run a 360° test (4× 90° turns)
3. Adjust `WHEEL_TRACK` proportionally

---

## Sensor Configuration

### ToF Sensor Indices (tof_main.h)

```
Sensor 0: Left side     → SENSOR_LEFT_SIDE
Sensor 1: ?
Sensor 2: ?
Sensor 3: Front center  → (example)
Sensor 4: ?
Sensor 5: ?
Sensor 6: Right side    → SENSOR_RIGHT_SIDE
```

**Action Items:**
1. Map your actual sensor layout
2. Update `SENSOR_LEFT_SIDE` and `SENSOR_RIGHT_SIDE` in navigation_control.h
3. Test with `example_sensor_validation()`

---

## Serial Commands Quick Reference

| Command | Syntax | Example | Description |
|---------|--------|---------|-------------|
| Forward | `F:x:w` | `F:1.0:0.0` | Move forward (x m/s, w rad/s) |
| Backward | `B:x:w` | `B:-1.0:0.0` | Move backward |
| Turn | `T:angle` | `T:90` | Discrete turn (degrees) |
| Stop | `S` | `S` | Emergency stop |
| Centering | `N:value` | `N:1` | Enable (1) or disable (0) |
| Set Kp (motor) | `P:value` | `P:200` | Set motor PID Kp |
| Set Ki (motor) | `I:value` | `I:70` | Set motor PID Ki |
| Set Kd (motor) | `D:value` | `D:30` | Set motor PID Kd |
| Info | `Q` | `Q` | Print motor velocities |
| Reset Encoders | `E` | `E` | Zero all encoders |
| Reset IMU | `C` | `C` | Reinitialize IMU |

---

## Calibration Checklist

### Before First Run

- [ ] Measure `WHEEL_TRACK` accurately (critical!)
- [ ] Measure `WHEEL_RADIUS` accurately
- [ ] Verify encoder tick counts (spin wheel by hand)
- [ ] Map ToF sensor positions
- [ ] Calibrate IMU (magnetometer to 3/3, system to 3/3)
- [ ] Test motor directions (all should spin correctly)
- [ ] Check battery voltage (>11V recommended)

### During Tuning

- [ ] Start with default `KP_CENTERING = 0.5`
- [ ] Test on straight corridor first
- [ ] Verify left/right corrections work correctly
- [ ] Test 90° turn (should be ±2° accurate)
- [ ] Adjust `TURN_SPEED` if overshooting
- [ ] Fine-tune fusion weights if needed

### After Tuning

- [ ] Document your final parameters
- [ ] Test full maze navigation pattern
- [ ] Verify repeatability (run 3-5 times)
- [ ] Note any temperature/battery drift

---

## Common Issues & Fixes

| Issue | Likely Cause | Fix |
|-------|--------------|-----|
| Robot drifts to one side | Unequal motor speeds | Check wiring, adjust PWM scaling |
| No centering correction | Wrong sensor indices | Update `SENSOR_LEFT_SIDE/RIGHT_SIDE` |
| Turns too far | `WHEEL_TRACK` too small | Increase `WHEEL_TRACK` by 5-10% |
| Turns too little | `WHEEL_TRACK` too large | Decrease `WHEEL_TRACK` by 5-10% |
| Oscillates between walls | Kp too high | Reduce `KP_CENTERING` to 0.3 |
| Slow to react | Kp too low | Increase `KP_CENTERING` to 0.8 |
| Turn timeout | Speed too low | Increase `TURN_SPEED` or timeout |
| Erratic behavior | IMU not calibrated | Calibrate magnetometer fully |
| Encoders not counting | Wiring/interrupt issue | Check encoder connections |

---

## Performance Targets

| Metric | Target | Acceptable | Poor |
|--------|--------|------------|------|
| Turn accuracy | ±1° | ±2-3° | >5° |
| Turn time (90°) | 1-2s | 2-3s | >4s |
| Centering error | ±5mm | ±10mm | >20mm |
| Wall following | Smooth | Minor oscillation | Wild oscillation |

---

## Debugging Tips

### Enable Debug Output

1. **Centering debug** - already enabled (prints every 200ms)
2. **Turn debug** - already enabled (prints every 100ms)
3. **Motor debug** - uncomment in motor_control.h line ~73

### Serial Monitor Settings

- Baud rate: 115200
- Line ending: Newline or Both NL & CR
- Timestamp: Enabled (helps timing analysis)

### What to Monitor

**During centering:**
```
Centering correction: 15.5 | L: 180mm | R: 150mm
```
- Correction should be proportional to (L - R)
- Values should stabilize when centered

**During turns:**
```
Error: 45.2 deg | Progress: 50.1% | Encoders: 87/174
```
- Error should decrease smoothly
- Progress should increase linearly
- Encoders should match expected ticks

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-10-18 | Initial implementation |

---

**For detailed explanations, see:** `NAVIGATION_CONTROL_README.md`
