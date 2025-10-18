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
- `MOVE_FORWARD_AFTER_TURN` - Moving forward after a turn to discover new walls (400ms)

### State Flow
1. **Detect condition** (no wall, front wall, etc.)
2. **Enter turn state** (set state, start time, duration)
3. **Execute turn** (apply PWM for duration)
4. **Check post-turn conditions**:
   - Read sensors (front, left, right)
   - After RIGHT turn: Check if left wall appeared AND front is clear
   - After LEFT turn: Check if right wall appeared AND front is clear
   - After FRONT turn: Check if front is clear
5. **Move forward (if conditions met)** or **Stop** (if conditions not met)
6. **Return to TURN_NONE** and resume normal operation

### Anti-Spin Logic
The robot will **NOT move forward** after a turn if:
- Front is blocked (rawF < FRONT_THRESHOLD)
- After right turn: No left wall detected (opposite wall didn't appear)
- After left turn: No right wall detected (opposite wall didn't appear)

This prevents infinite turning when no walls are discovered!

## Behavior Summary

| Condition | Action | Duration | Next Action |
|-----------|--------|----------|-------------|
| Front wall detected | Turn left 90° in place | 400ms | Check if front clear → Move forward or stop |
| No right wall | Turn right | 300ms | Check if left wall appeared AND front clear → Move forward or stop |
| No left wall | Turn left | 300ms | Check if right wall appeared AND front clear → Move forward or stop |
| Both walls present | Follow walls (centered) | Continuous | Continue following |
| After turn (conditions met) | Move forward | 400ms | Return to normal operation |

## Key Thresholds

- **FRONT_THRESHOLD**: 300mm - Closer than this triggers front wall turn
- **SIDE_WALL_THRESHOLD**: 200mm - Farther than this = "no wall detected"
- **BASE_SPEED**: 150 (0-255) - Speed when following walls
- **TURN_SPEED**: 200 (0-255) - Speed when searching for walls
- **TURN_DELAY**: 400ms - Duration for front wall 90° turn
- **RIGHT_TURN_DURATION**: 300ms - Duration for right turn
- **LEFT_TURN_DURATION**: 300ms - Duration for left turn
- **FORWARD_AFTER_TURN_DURATION**: 400ms - Forward movement after successful turn

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

Plus detailed action and state messages:
- "Front wall detected - initiating left turn"
- "No right wall - initiating right turn"
- "No left wall - initiating left turn"
- "Turn complete: RIGHT" / "Turn complete: LEFT" / "Turn complete: FRONT"
- "After turn - L=xxx R=xxx F=xxx"
- "After RIGHT turn: frontClear=1 hasLeftWall=1"
- "Front is clear - moving forward"
- "Cannot move forward - front blocked or no wall detected"
- "Forward movement complete"
- (Silent when following both walls)

## Typical Maze Navigation

### Scenario 1: Following Right Wall
```
Right wall present, left open → Turn left to find wall
Both walls present → Center between them
Front wall appears → Turn left 90°
```

### Scenario 2: Open Intersection (Anti-Spin Logic Active)
```
No right wall detected → Turn right
After turn: Check sensors
  - Front clear? YES
  - Left wall appeared? YES
  → Move forward 400ms
Return to wall following → Now both walls present
```

### Scenario 3: Dead End (Anti-Spin Logic Active)
```
Front wall detected → Turn left 90°
After turn: Check sensors
  - Front clear? YES
  → Move forward 400ms
Continue navigating
```

### Scenario 4: Robot Prevents Infinite Spin
```
No right wall detected → Turn right 300ms
After turn: Check sensors
  - Front clear? NO (wall ahead)
  → STOP, do not move forward
  → Check walls again, make new decision
```

OR

```
No right wall detected → Turn right 300ms
After turn: Check sensors
  - Front clear? YES
  - Left wall appeared? NO (still in open space)
  → STOP, do not move forward
  → Will detect no right wall again but won't spin endlessly
  → Robot stays in place until conditions change
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

**Robot spins indefinitely**
- Check FORWARD_AFTER_TURN_DURATION (increase to 500-600ms)
- Verify sensor readings after turn with debug output
- Ensure SIDE_WALL_THRESHOLD is appropriate for your maze
- Check that opposite wall is actually detected after turn

**Robot doesn't move after turning**
- This is normal if front is blocked or no wall detected!
- Check debug: "Cannot move forward - front blocked or no wall detected"
- Verify sensors are reading correctly
- May need to adjust FRONT_THRESHOLD or SIDE_WALL_THRESHOLD
