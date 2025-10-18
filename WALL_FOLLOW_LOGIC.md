# Wall Following Logic - Quick Reference

## Decision Tree (Timing-Based)

The robot uses a state machine with timing-based turns:

```
1. Is robot currently turning?
   YES → Continue turn until duration expires → Go to step 2
   NO  → Go to step 2

2. Is there a front wall? (rawF < FRONT_THRESHOLD)
   YES → Start TURN_FRONT state (400ms) → Turn left 90°
   NO  → Go to step 3

3. Is there a right wall? (filtR < SIDE_WALL_THRESHOLD)
   NO  → Start TURN_RIGHT state (300ms) → Turn right
   YES → Go to step 4

4. Is there a left wall? (filtL < SIDE_WALL_THRESHOLD)
   NO  → Start TURN_LEFT state (300ms) → Turn left
   YES → Go to step 5

5. Both walls present
   → Follow walls (stay centered using proportional control)
```

## State Machine

### Turn States
- `TURN_NONE` - Normal operation, not turning
- `TURN_RIGHT` - Executing right turn (300ms)
- `TURN_LEFT` - Executing left turn (300ms)
- `TURN_FRONT` - Executing front wall 90° turn (400ms)

### State Flow
1. **Detect condition** (no wall, front wall, etc.)
2. **Enter turn state** (set state, start time, duration)
3. **Execute turn** (apply PWM for duration)
4. **Exit turn state** (stop motors, reset filters, return to TURN_NONE)
5. **Resume** normal wall following

## Behavior Summary

| Condition | Action | Duration | PWM Command |
|-----------|--------|----------|-------------|
| Front wall detected | Turn left 90° in place | 400ms | `sendPWM(225, -225)` |
| No right wall | Turn right | 300ms | `sendPWM(-TURN_SPEED, TURN_SPEED)` |
| No left wall | Turn left | 300ms | `sendPWM(TURN_SPEED, -TURN_SPEED)` |
| Both walls present | Follow walls (centered) | Continuous | `sendPWM(BASE_SPEED+corr, BASE_SPEED-corr)` |

## Key Thresholds

- **FRONT_THRESHOLD**: 300mm - Closer than this triggers front wall turn
- **SIDE_WALL_THRESHOLD**: 200mm - Farther than this = "no wall detected"
- **BASE_SPEED**: 150 (0-255) - Speed when following walls
- **TURN_SPEED**: 200 (0-255) - Speed when searching for walls
- **TURN_DELAY**: 400ms - Duration for front wall 90° turn
- **RIGHT_TURN_DURATION**: 300ms - Duration for right turn
- **LEFT_TURN_DURATION**: 300ms - Duration for left turn

## Sensor Indices (Adjust for your robot!)

```cpp
int rawL = tof_distances[0]; // Left sensor - INDEX 0
int rawR = tof_distances[2]; // Right sensor - INDEX 2  
int rawF = tof_distances[5]; // Front sensor - INDEX 5
```

## Debug Output

When wall following is active, you'll see:
```
WF: L=250 R=280 F=1500
```
- L = Filtered left sensor reading (mm)
- R = Filtered right sensor reading (mm)
- F = Raw front sensor reading (mm)

Plus action and state messages:
- "Front wall detected - initiating left turn"
- "No right wall - initiating right turn"
- "No left wall - initiating left turn"
- "Turn complete: RIGHT" / "Turn complete: LEFT" / "Turn complete: FRONT"
- (Silent when following both walls)

## Typical Maze Navigation

### Scenario 1: Following Right Wall
```
Right wall present, left open → Turn left to find wall
Both walls present → Center between them
Front wall appears → Turn left 90°
```

### Scenario 2: Open Intersection
```
No right wall → Turn right immediately
(Robot will explore right-hand paths first)
```

### Scenario 3: Dead End
```
Front wall detected → Turn left 90°
Check walls again → Continue based on new orientation
```

## Tuning Tips

### Robot favors right turns
This is by design! The logic checks right wall first, so robot prefers turning right when walls are missing. To prefer left turns instead, swap the order in the code:

```cpp
// Change priority to left-first:
if (!hasLeftWall) {
    // Turn left
} else if (!hasRightWall) {
    // Turn right
}
```

### Adjusting turn sensitivity
- **Increase SIDE_WALL_THRESHOLD** (e.g., 300mm) → More aggressive turning
- **Decrease SIDE_WALL_THRESHOLD** (e.g., 150mm) → Less turning, tighter following

### Adjusting turn duration
- **Increase RIGHT/LEFT_TURN_DURATION** (e.g., 400ms) → Sharper turns
- **Decrease RIGHT/LEFT_TURN_DURATION** (e.g., 200ms) → Gentler turns
- **Adjust TURN_DELAY** (400ms default) → Calibrate for exact 90° front turns

### Smooth vs Responsive
- **Lower ALPHA** (e.g., 0.2) → Smoother, ignores noise, slower response
- **Higher ALPHA** (e.g., 0.5) → More responsive, may react to noise

## Command Reference

```
W:1   # Enable wall following
W:0   # Disable wall following
S     # Emergency stop (also disables wall following)
G     # Get sensor readings
```

## Safety Features

1. **Command timeout**: If no serial command for 1000ms and isMoving=true, robot stops
2. **Wall following disable**: Sending `W:0` immediately stops motors
3. **Front wall priority**: Always checked first to prevent collisions
4. **Filter reset**: After turning, filters reset to prevent stale data

## Common Issues

**Robot spins in circles**
- Check SIDE_WALL_THRESHOLD - may be too low
- Verify sensors are working: send `G` command
- Check sensor mounting - may be reading floor/ceiling

**Robot ignores walls**
- SIDE_WALL_THRESHOLD may be too high
- Verify sensor readings with debug output
- Check wiring and I2C multiplexer

**Unstable wall following**
- Reduce Kp (try 0.3)
- Increase filtering (lower ALPHA to 0.2)
- Check for mechanical issues (wobbling wheels)

**Turns are too sharp/not sharp enough**
- Adjust RIGHT_TURN_DURATION and LEFT_TURN_DURATION
- Modify TURN_SPEED (higher = sharper turns)
- Fine-tune based on robot's turning radius

**Front wall turns aren't 90 degrees**
- Adjust TURN_DELAY parameter (in header.h)
- Use encoder counts for precise calibration
- Test on actual maze walls and adjust timing
