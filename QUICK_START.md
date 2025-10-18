# Quick Start Guide - Maze Navigation State Machine

## Overview
Your robot now uses a clean state machine for maze navigation with improved decision-making.

## What Changed?

### ✅ Kept Working:
- ✓ Wall centering with proportional control (Kp)
- ✓ 90° timing-based turns
- ✓ All serial commands
- ✓ ToF sensor integration
- ✓ Motor control and PID

### ✨ New Features:
- ✓ Explicit state machine with 8 states
- ✓ Priority-based decision making
- ✓ Better turn validation
- ✓ Comprehensive debug logging
- ✓ Cleaner code structure

## How to Use

### 1. Upload Firmware
```bash
platformio run --target upload --environment esp32doit-devkit-v1
```

### 2. Open Serial Monitor
```bash
platformio device monitor
```

### 3. Start Navigation
Send command: `W`

The robot will:
1. Check surroundings (sensors)
2. Decide action (based on priorities)
3. Execute maneuver (turn or move)
4. Validate result
5. Repeat

### 4. Stop Navigation
Send command: `S` or `W` (toggles)

## State Flow

```
START → CHECK → DECIDE:
                  ├─→ FRONT WALL? → TURN_FRONT → POST_TURN → CHECK
                  ├─→ NO RIGHT WALL? → TURN_RIGHT → POST_TURN → CHECK
                  ├─→ NO LEFT WALL? → TURN_LEFT → POST_TURN → CHECK
                  └─→ BOTH WALLS? → MOVE_FORWARD (centering) → CHECK
```

## Serial Monitor Output

You'll see messages like:
```
State transition: CHECK_SURROUNDINGS -> TURN_RIGHT
Priority: NO_RIGHT_WALL - turning right
Sensors - L:150 R:0 F:350 | Walls - L:1 R:0 F:0
Right turn complete
Front clear after right turn - moving forward
State transition: TURN_RIGHT -> POST_TURN_FORWARD
Post-turn forward complete
State transition: POST_TURN_FORWARD -> CHECK_SURROUNDINGS
```

## Calibration

### Tune Turn Durations:

1. **Test right turn:**
   ```
   r:1000    # Try 1000ms
   r:1100    # Adjust until 90° accurate
   ```

2. **Test left turn:**
   ```
   l:900     # Try 900ms
   l:950     # Adjust until 90° accurate
   ```

3. **Update in `header.h`:**
   ```cpp
   const int RIGHT_TURN_DELAY = 1100;  // Your calibrated value
   const int LEFT_TURN_DELAY = 900;    // Your calibrated value
   ```

4. **Rebuild and upload**

## Tuning Parameters

Edit `include/header.h`:

### Speed:
```cpp
#define BASE_SPEED 150      // Increase for faster navigation (0-255)
#define TURN_SPEED 200      // Turn speed
```

### Wall Following:
```cpp
#define Kp 0.2              // Increase for stronger centering
#define MAX_CORR 50         // Maximum correction value
```

### Sensor Thresholds:
```cpp
#define FRONT_THRESHOLD 200       // Closer = later turn
#define SIDE_WALL_THRESHOLD 300   // Distance to detect walls
```

### Timing:
```cpp
#define LOOP_MS 20                        // Main loop rate (50Hz)
#define FORWARD_AFTER_TURN_DURATION 400   // Distance after turn
```

## Troubleshooting

### Robot doesn't turn at intersections
**Cause:** Wall threshold too restrictive  
**Fix:** Increase `SIDE_WALL_THRESHOLD` to 400

### Robot oscillates between walls
**Cause:** Kp gain too high  
**Fix:** Reduce `Kp` to 0.15 or 0.1

### Turns are not 90 degrees
**Cause:** Turn timing not calibrated  
**Fix:** Use calibration commands `r:ms` and `l:ms`

### Robot stops after every turn
**Cause:** Post-turn validation failing  
**Fix:** Check sensor alignment and thresholds

### Stuck in one state
**Cause:** State timeout not triggering  
**Fix:** Check serial monitor for state name, verify sensors

## Command Reference

| Command | Action | Example |
|---------|--------|---------|
| `W` | Toggle navigation | `W` → start/stop |
| `S` | Stop all motors | `S` |
| `r:ms` | Calibrate right turn | `r:1100` |
| `l:ms` | Calibrate left turn | `l:900` |
| `G` | Get sensor data | `G` |
| `E` | Reset encoders | `E` |
| `C` | Reset IMU | `C` |

## Debug Mode

To enable detailed sensor debugging, add to `maze_navigation.h`:
```cpp
#define DEBUG_SENSORS  // Before update_sensor_readings()
```

## Performance Tips

1. **Start slow**: Use `BASE_SPEED = 120` initially
2. **Calibrate turns**: Use timing commands before maze runs
3. **Check sensors**: Verify ToF readings with `G` command
4. **Monitor states**: Watch serial output during navigation
5. **Tune gradually**: Adjust one parameter at a time

## State Descriptions

| State | What it does |
|-------|-------------|
| `NAV_IDLE` | Waiting for start command |
| `NAV_CHECK_SURROUNDINGS` | Read sensors, decide action |
| `NAV_MOVE_FORWARD` | Wall following with centering |
| `NAV_TURN_RIGHT` | Executing 90° right turn |
| `NAV_TURN_LEFT` | Executing 90° left turn |
| `NAV_TURN_FRONT` | Turn when facing wall |
| `NAV_POST_TURN_FORWARD` | Move forward after turn |
| `NAV_CALIBRATION_TURN` | Manual calibration turn |

## Next Steps

1. ✓ Build and upload firmware
2. ✓ Test wall following (`W` command)
3. ✓ Calibrate turn timings
4. ✓ Test in maze
5. ✓ Tune parameters for speed/accuracy
6. ✓ Document your specific calibration values

## Need Help?

- Check `MAZE_NAVIGATION_STATE_MACHINE.md` for detailed documentation
- Review `REFACTORING_SUMMARY.md` for technical changes
- Monitor serial output for state transitions
- Use calibration mode to test individual maneuvers

## Success Criteria

✅ Robot follows walls and stays centered  
✅ Robot turns 90° at intersections  
✅ Robot handles dead ends (front wall)  
✅ Robot completes maze without getting stuck  
✅ State transitions are smooth and logical  

Happy maze solving! 🐭🏁
