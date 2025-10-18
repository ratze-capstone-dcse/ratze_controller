# Maze Navigation State Machine

## Overview

This document describes the refactored maze navigation system for the RATZE micromouse robot. The system has been redesigned as a clean, explicit state machine that improves decision-making and makes the code more maintainable.

## Architecture

### State Machine Design

The navigation system uses a **finite state machine (FSM)** approach with the following key components:

1. **Explicit States**: Each state represents a clear robot behavior
2. **Clean Transitions**: State changes are based on sensor readings and timing
3. **Separation of Concerns**: Decision logic is separate from motor control

### States

```cpp
enum MazeNavigationState {
  NAV_IDLE,                    // Robot is not navigating
  NAV_CHECK_SURROUNDINGS,      // Analyze sensors and decide next action
  NAV_MOVE_FORWARD,            // Moving forward with wall-following
  NAV_TURN_RIGHT,              // Executing right turn
  NAV_TURN_LEFT,               // Executing left turn
  NAV_TURN_FRONT,              // Executing front wall turn (90° left)
  NAV_POST_TURN_FORWARD,       // Moving forward after completing a turn
  NAV_CALIBRATION_TURN         // Timing-based calibration turn
}
```

### Decision Priorities

The robot uses a priority-based decision system:

```cpp
enum NavigationPriority {
  PRIORITY_FRONT_WALL,         // Highest: Front wall detected → must turn
  PRIORITY_NO_RIGHT_WALL,      // High: No right wall → turn right (right-hand rule)
  PRIORITY_NO_LEFT_WALL,       // Medium: No left wall → turn left
  PRIORITY_WALL_FOLLOW,        // Low: Both walls present → follow wall
  PRIORITY_NONE                // No clear decision
}
```

## State Descriptions

### NAV_IDLE
- **Purpose**: Waiting state when navigation is disabled
- **Entry**: When navigation is stopped or system starts
- **Exit**: When navigation is enabled (transition to `NAV_CHECK_SURROUNDINGS`)

### NAV_CHECK_SURROUNDINGS
- **Purpose**: Analyze sensor data and determine next action
- **Entry**: After turns, forward movements, or when starting navigation
- **Logic**: 
  1. Read ToF sensors (left, right, front)
  2. Apply filters
  3. Determine navigation priority
  4. Transition to appropriate action state
- **Exit**: Transitions to turn states or `NAV_MOVE_FORWARD`

### NAV_MOVE_FORWARD
- **Purpose**: Move forward while maintaining center position between walls
- **Entry**: When both left and right walls are detected
- **Logic**: 
  - Calculate error: `error = filtered_right - filtered_left`
  - Apply proportional correction: `correction = Kp * error`
  - Adjust motor speeds to center the robot
- **Exit**: Immediately transitions back to `NAV_CHECK_SURROUNDINGS` for continuous decision-making

### NAV_TURN_RIGHT
- **Purpose**: Execute 90° right turn
- **Entry**: When no right wall is detected (right-hand rule)
- **Duration**: `RIGHT_TURN_DELAY` ms (configurable, default: 1100ms)
- **Logic**: 
  - Apply differential drive: left motor forward, right motor backward
  - Monitor elapsed time
- **Exit**: After timeout, check if front is clear and opposite wall exists
  - If yes → `NAV_POST_TURN_FORWARD`
  - If no → `NAV_CHECK_SURROUNDINGS`

### NAV_TURN_LEFT
- **Purpose**: Execute 90° left turn
- **Entry**: When no left wall is detected
- **Duration**: `LEFT_TURN_DELAY` ms (configurable, default: 900ms)
- **Logic**: Similar to right turn but opposite motor directions
- **Exit**: Same as right turn with appropriate wall checks

### NAV_TURN_FRONT
- **Purpose**: Execute turn when facing a front wall
- **Entry**: When front wall is too close
- **Duration**: `TURN_DELAY` ms (default: 400ms)
- **Logic**: 90° left turn to avoid front obstacle
- **Exit**: After timeout, check if front is now clear
  - If yes → `NAV_POST_TURN_FORWARD`
  - If no → `NAV_CHECK_SURROUNDINGS`

### NAV_POST_TURN_FORWARD
- **Purpose**: Move forward a fixed distance after completing a turn
- **Entry**: When turn is complete and path is clear
- **Duration**: `FORWARD_AFTER_TURN_DURATION` ms (default: 400ms)
- **Logic**: Drive straight at base speed
- **Exit**: After timeout → `NAV_CHECK_SURROUNDINGS`

### NAV_CALIBRATION_TURN
- **Purpose**: Execute timing-based turns for calibration purposes
- **Entry**: Via serial commands `r:duration` or `l:duration`
- **Duration**: User-specified in milliseconds
- **Logic**: Execute precise timing-based turn
- **Exit**: After timeout → `NAV_IDLE`

## Flow Diagram

```
┌─────────────┐
│  NAV_IDLE   │
└──────┬──────┘
       │ Enable Navigation
       ▼
┌─────────────────────┐
│ NAV_CHECK_          │◄─────────┐
│ SURROUNDINGS        │          │
└──────┬──────────────┘          │
       │ Decision                │
       ├──────────────┐          │
       ▼              ▼          │
┌─────────────┐ ┌─────────────┐ │
│ NAV_TURN_   │ │ NAV_MOVE_   │ │
│ [DIRECTION] │ │ FORWARD     │─┤
└──────┬──────┘ └─────────────┘ │
       │                         │
       ▼                         │
┌─────────────────────┐          │
│ NAV_POST_TURN_      │          │
│ FORWARD             │──────────┘
└─────────────────────┘
```

## Sensor Configuration

The system uses Time-of-Flight (ToF) sensors:

- **Sensor 0**: Left wall detection
- **Sensor 2**: Right wall detection  
- **Sensor 5**: Front wall detection

### Filtering

Low-pass filter is applied to reduce noise:
```cpp
filtered = ALPHA * raw + (1.0 - ALPHA) * filtered_previous
```
- `ALPHA = 0.3` (configurable)

### Thresholds

- `FRONT_THRESHOLD = 200mm`: Front wall detection distance
- `SIDE_WALL_THRESHOLD = 300mm`: Side wall detection distance
- Sensor reading of `0` indicates no detection (out of range)

## Control Parameters

### Wall Following
- `BASE_SPEED = 150`: Base motor PWM (0-255)
- `Kp = 0.2`: Proportional gain for centering
- `MAX_CORR = 50`: Maximum correction value

### Turning
- `TURN_SPEED = 200`: Motor PWM during turns
- `RIGHT_TURN_DELAY = 1100ms`: Duration for 90° right turn
- `LEFT_TURN_DELAY = 900ms`: Duration for 90° left turn
- `TURN_DELAY = 400ms`: Duration for front wall turn

### Timing
- `LOOP_MS = 20`: Main loop period (50Hz)
- `FORWARD_AFTER_TURN_DURATION = 400ms`: Forward movement after turn

## API Functions

### Public Interface

```cpp
// Start maze navigation
void start_maze_navigation();

// Stop maze navigation
void stop_maze_navigation();

// Check if navigation is active
bool is_maze_navigation_active();

// Execute calibration turns
void execute_calibration_turn_right(int duration_ms);
void execute_calibration_turn_left(int duration_ms);
```

### Motor Control Functions

```cpp
// Implemented in firmware_control.h
void execute_wall_following();    // PID-based centering
void execute_turn_right();        // Right turn motor control
void execute_turn_left();         // Left turn motor control
void execute_turn_front();        // Front wall turn motor control
void execute_stop();              // Stop all motors
void execute_forward();           // Move forward at base speed
```

## Serial Commands

The robot accepts the following commands:

- `W`: Toggle wall following mode (enable/disable navigation)
- `r:duration`: Execute calibration right turn (e.g., `r:1100`)
- `l:duration`: Execute calibration left turn (e.g., `l:900`)
- `S`: Stop navigation and motors

## Advantages of State Machine Approach

### 1. **Clear Structure**
- Each state has a single, well-defined purpose
- Easy to understand what the robot is doing at any time

### 2. **Robust Decision Making**
- Priority-based decisions prevent ambiguous situations
- Sensor readings determine transitions, not just timing

### 3. **Easy Debugging**
- State transitions are logged to serial
- Current state is always known
- Easy to trace execution flow

### 4. **Maintainable**
- Adding new states is straightforward
- Modifying behavior in one state doesn't affect others
- Parameters are centralized and well-documented

### 5. **Testable**
- Individual states can be tested in isolation
- Calibration mode allows tuning turn durations
- Clear separation of concerns

## Tuning Guide

### Turn Duration Calibration

1. **Enable calibration mode** by sending timing-based turn commands:
   ```
   r:1000    # Test right turn for 1000ms
   l:1000    # Test left turn for 1000ms
   ```

2. **Measure actual turn angle** and adjust durations in `header.h`:
   ```cpp
   const int RIGHT_TURN_DELAY = 1100;  // Adjust for your robot
   const int LEFT_TURN_DELAY = 900;    // Adjust for your robot
   ```

3. **Test in maze** and fine-tune based on performance

### Wall Following Tuning

1. **Adjust proportional gain** for smoother centering:
   ```cpp
   #define Kp 0.2  // Increase for stronger correction
   ```

2. **Modify base speed** for desired velocity:
   ```cpp
   #define BASE_SPEED 150  // Increase for faster navigation
   ```

3. **Tune sensor thresholds** based on your maze:
   ```cpp
   #define FRONT_THRESHOLD 200       // Distance to trigger front turn
   #define SIDE_WALL_THRESHOLD 300   // Distance to detect side walls
   ```

## Future Enhancements

Potential improvements to the state machine:

1. **Encoder-based distance**: Replace timing with encoder counts for turns and forward movements
2. **Adaptive speed**: Adjust speed based on maze complexity
3. **Memory**: Remember visited cells to prevent loops
4. **Path planning**: Implement flood-fill or other maze-solving algorithms
5. **IMU integration**: Use gyroscope for precise turn angles
6. **Dynamic tuning**: Auto-calibrate turn durations based on IMU feedback

## Troubleshooting

### Robot doesn't turn at intersections
- Check `SIDE_WALL_THRESHOLD` - may be too low
- Verify sensor readings with `NAV_CHECK_SURROUNDINGS` debug output
- Increase filter alpha for faster response

### Robot oscillates while following walls
- Reduce `Kp` proportional gain
- Decrease `BASE_SPEED`
- Check for mechanical issues

### Turns are not 90 degrees
- Use calibration commands to tune turn durations
- Verify battery voltage (affects motor speed)
- Check wheel slippage

### Robot gets stuck after turns
- Adjust `FORWARD_AFTER_TURN_DURATION`
- Check post-turn wall detection logic
- Verify sensor alignment

## Code Location

- **State Machine**: `include/maze_navigation.h`
- **Motor Control**: `include/firmware_control.h`
- **Configuration**: `include/header.h`
- **Main Loop**: `src/main.cpp`

## License

Part of the RATZE Micromouse project.
