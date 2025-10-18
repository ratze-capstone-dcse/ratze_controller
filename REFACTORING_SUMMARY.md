# Maze Navigation Refactoring Summary

## Changes Made

### 1. New State Machine Architecture (`maze_navigation.h`)

Created a completely new state machine system that separates decision-making from motor control:

#### Key Components:
- **MazeNavigationState enum**: 8 distinct states for robot behavior
- **NavigationPriority enum**: Priority-based decision system
- **MazeNavigator struct**: Encapsulates all state machine data
- **State transitions**: Clean, explicit transitions with timing control

#### States Implemented:
1. `NAV_IDLE` - Inactive state
2. `NAV_CHECK_SURROUNDINGS` - Decision-making state
3. `NAV_MOVE_FORWARD` - Wall following with centering
4. `NAV_TURN_RIGHT` - Right turn execution
5. `NAV_TURN_LEFT` - Left turn execution
6. `NAV_TURN_FRONT` - Front wall turn execution
7. `NAV_POST_TURN_FORWARD` - Post-turn forward movement
8. `NAV_CALIBRATION_TURN` - Timing-based calibration turns

### 2. Refactored `firmware_control.h`

#### Removed:
- Complex nested if-else logic in `wallFollowingLoop()`
- `TurnState` enum (replaced by `MazeNavigationState`)
- Manual state tracking variables (`currentTurnState`, `turnStartTime`, etc.)
- `turn_right_timing()` and `turn_left_timing()` functions
- Complex turn completion logic

#### Added:
- Clean motor control helper functions:
  - `execute_wall_following()`
  - `execute_turn_right()`
  - `execute_turn_left()`
  - `execute_turn_front()`
  - `execute_stop()`
  - `execute_forward()`
- Integration with new state machine via `maze_navigation_update()`
- Updated command handlers to use new API functions

### 3. Decision-Making Improvements

#### Old System:
- Decisions mixed with motor control
- Hard to follow execution flow
- State transitions unclear
- Manual timing management

#### New System:
- **Priority-based decisions**:
  1. Front wall (highest priority)
  2. No right wall (right-hand rule)
  3. No left wall
  4. Wall following (both walls present)
- Sensor readings determine state transitions
- Clear separation: decide → execute → check result

### 4. Benefits

#### Readability:
- Each function has a single responsibility
- State machine logic is in one place
- Easy to understand current robot behavior
- Clear state transition logging

#### Maintainability:
- Adding new states is straightforward
- Modifying one state doesn't affect others
- Parameters centralized in `header.h`
- Well-documented code with comments

#### Robustness:
- Explicit state management prevents undefined behavior
- Post-turn validation checks
- Timeout-based state transitions
- Safety stops between maneuvers

#### Debugging:
- State transitions logged to serial
- Clear state names in debug output
- Easy to trace execution flow
- Can monitor current state at any time

## Code Structure

```
include/
├── maze_navigation.h          ← New state machine (400+ lines)
├── firmware_control.h         ← Refactored (reduced from 500+ to 300 lines)
├── header.h                   ← Configuration parameters (unchanged)
├── motor_control.h            ← Motor PID control (unchanged)
└── motor_driver.h             ← Low-level motor control (unchanged)
```

## Usage

### Starting/Stopping Navigation:
```cpp
// Via serial command 'W' or programmatically:
start_maze_navigation();    // Begin autonomous navigation
stop_maze_navigation();     // Stop and return to idle
```

### Calibration Turns:
```cpp
// Via serial commands 'r:duration' or 'l:duration':
execute_calibration_turn_right(1100);  // 1100ms right turn
execute_calibration_turn_left(900);    // 900ms left turn
```

### Status Check:
```cpp
bool active = is_maze_navigation_active();
```

## Serial Commands

| Command | Description | Example |
|---------|-------------|---------|
| `W` | Toggle wall following | `W` |
| `r:ms` | Right calibration turn | `r:1100` |
| `l:ms` | Left calibration turn | `l:900` |
| `S` | Stop navigation | `S` |
| `G` | Get sensor data | `G` |

## Testing Checklist

- [ ] Build compiles successfully
- [ ] Upload to ESP32
- [ ] Test wall following mode (`W` command)
- [ ] Verify state transitions in serial monitor
- [ ] Test right turns at intersections
- [ ] Test left turns at intersections
- [ ] Test front wall detection and turning
- [ ] Calibrate turn timings (`r:ms` and `l:ms`)
- [ ] Verify post-turn forward movement
- [ ] Test maze completion

## Tuning Parameters

Located in `header.h`:

```cpp
// Wall Following
#define BASE_SPEED 150          // Adjust for speed
#define Kp 0.2                  // Adjust for centering
#define MAX_CORR 50             // Max correction

// Turn Timing
const int RIGHT_TURN_DELAY = 1100;  // Calibrate for 90° right
const int LEFT_TURN_DELAY = 900;    // Calibrate for 90° left
const int TURN_DELAY = 400;         // Front wall turn

// Sensor Thresholds
#define FRONT_THRESHOLD 200     // Front wall distance
#define SIDE_WALL_THRESHOLD 300 // Side wall detection
```

## Future Enhancements

1. **Encoder-based distance**: Replace timing with encoder counts
2. **IMU integration**: Use gyroscope for precise turn angles
3. **Adaptive speed**: Slow down for turns, speed up on straights
4. **Maze mapping**: Store visited cells
5. **Path optimization**: Implement flood-fill algorithm
6. **Dynamic calibration**: Auto-tune turn durations using IMU

## Migration Notes

### If you need to revert:
The old `wallFollowingLoop()` logic has been completely removed. To revert, you would need to restore from git history.

### Backward compatibility:
- All serial commands remain the same
- Motor control functions are identical
- PID control is unchanged
- Sensor reading unchanged

## Performance Comparison

| Aspect | Old System | New System |
|--------|-----------|------------|
| Code lines | ~500 | ~700 (more documented) |
| States | Implicit | 8 explicit states |
| Decision logic | Mixed with control | Separate priority system |
| Debug output | Minimal | Comprehensive logging |
| Extensibility | Difficult | Easy to add states |
| Readability | Poor | Excellent |

## Documentation

- **MAZE_NAVIGATION_STATE_MACHINE.md**: Comprehensive guide
- **Code comments**: Extensive inline documentation
- **State flow diagram**: Visual representation of states
- **API documentation**: Public function descriptions

## Authors

Refactored by: GitHub Copilot
Original code: RATZE Capstone Team
Date: October 18, 2025
