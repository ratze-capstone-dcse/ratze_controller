# Wall Following Implementation

## Overview
This implementation provides autonomous wall following capability for the RATZE micromouse robot using ToF (Time-of-Flight) sensors.

## How It Works

### Sensor Configuration
The wall following algorithm uses three ToF sensors:
- **Sensor 0**: Left wall sensor
- **Sensor 2**: Right wall sensor  
- **Sensor 6**: Front wall sensor

> **Note**: Adjust sensor indices in `wallFollowingLoop()` based on your actual sensor placement.

### Algorithm Components

1. **Low-Pass Filtering**
   - Reduces sensor noise using exponential moving average
   - Formula: `filtered = ALPHA * raw + (1 - ALPHA) * previous`
   - Default ALPHA = 0.3 (lower values = more smoothing)

2. **Front Wall Detection** (Highest Priority)
   - Triggers when front sensor reads < FRONT_THRESHOLD (300mm default)
   - Robot stops, turns left 90°, then continues
   - Turn duration: 400ms (adjust as needed for your robot)

3. **Wall Detection Logic**
   - **No right wall detected** (distance > SIDE_WALL_THRESHOLD): Turn right to find wall
   - **No left wall detected** (distance > SIDE_WALL_THRESHOLD): Turn left to find wall
   - **Both walls present**: Normal wall following mode

4. **Wall Following Control** (When both walls present)
   - Calculates error: `error = rightSensor - leftSensor`
   - Applies proportional correction: `correction = Kp * error`
   - Adjusts motor speeds: `left += correction`, `right -= correction`
   - Keeps robot centered between walls

## Configuration Parameters

Located in `firmware_control.h`:

```cpp
#define LOOP_MS 20              // Control loop period (50Hz)
#define ALPHA 0.3               // Filter coefficient (0-1)
#define FRONT_THRESHOLD 300     // Front detection (mm)
#define SIDE_WALL_THRESHOLD 400 // Side wall detection (mm)
#define BASE_SPEED 150          // Base motor speed (0-255)
#define TURN_SPEED 180          // Turn speed when seeking wall
#define MAX_CORR 80             // Max correction limit
#define Kp 0.5                  // Proportional gain
```

### Tuning Guide

**BASE_SPEED**: Overall robot speed
- Start low (100-150) for testing
- Increase for faster navigation
- Lower if robot is unstable

**Kp**: Wall following responsiveness
- Too low: Slow corrections, hits walls
- Too high: Oscillates between walls
- Start at 0.5 and adjust

**MAX_CORR**: Maximum steering correction
- Limits how much speed can differ between wheels
- Prevents excessive turns
- Default 80 works for most cases

**ALPHA**: Sensor noise filtering
- Higher (0.5-0.7): More responsive, noisier
- Lower (0.1-0.3): Smoother, slower response
- Default 0.3 is a good balance

**FRONT_THRESHOLD**: Wall detection distance
- Set based on robot size and desired margin
- Lower value = closer approach
- Typical range: 200-400mm

**SIDE_WALL_THRESHOLD**: Side wall detection limit
- If sensor reads > this value, "no wall" is detected
- Robot will turn to find a wall
- Set based on maze corridor width
- Default 400mm works for standard micromouse mazes

## Serial Commands

### Enable Wall Following
```
W:1
```
Response: `ACK:W` and "Wall following enabled"

### Disable Wall Following
```
W:0
```
Response: `ACK:W` and "Wall following disabled"

## Usage Example

1. **Connect via serial** (115200 baud)
2. **Wait for** `READY` message
3. **Send** `W:1` to enable wall following
4. **Robot will:**
   - Move forward when both side walls are detected
   - Turn right when no right wall is detected
   - Turn left when no left wall is detected  
   - Turn left when front wall is detected
   - Continue until `W:0` is sent

## Testing Procedure

1. **Stationary Test**
   ```
   G    # Get sensor readings
   ```
   Verify all ToF sensors are working

2. **Enable Wall Following**
   ```
   W:1
   ```

3. **Monitor Behavior**
   - Check sensor data stream (IMU, TOF, ENC)
   - Observe robot maintaining center position
   - Verify front wall turns

4. **Disable**
   ```
   W:0
   ```

## Troubleshooting

**Robot oscillates between walls**
- Decrease Kp gain (e.g., 0.3)
- Increase ALPHA for more filtering (e.g., 0.5)

**Robot drifts into walls**
- Increase Kp gain (e.g., 0.7)
- Check sensor calibration
- Verify sensor indices are correct

**Robot keeps turning even with walls present**
- Adjust SIDE_WALL_THRESHOLD
- Check if sensors are reading correct values
- Verify filtered values with debug output

**Robot doesn't turn when expected**
- Lower SIDE_WALL_THRESHOLD value
- Check sensor mounting and alignment
- Verify turn speed is sufficient

**Front turns are inconsistent**
- Adjust turn delay (400ms default)
- Check base speed is appropriate
- Ensure front sensor is working

**Robot stops unexpectedly**
- Check FRONT_THRESHOLD value
- Verify command timeout (1000ms)
- Monitor serial for error messages

## Sensor Index Reference

Based on `NUM_SENSORS = 7`, adjust these in `wallFollowingLoop()`:
```cpp
int rawL = tof_distances[0]; // Left sensor - adjust index
int rawR = tof_distances[2]; // Right sensor - adjust index  
int rawF = tof_distances[6]; // Front sensor - adjust index
```

Check your hardware setup to determine correct indices.

## Integration with Existing Commands

Wall following mode:
- **Disables** differential drive commands (F, B, L, R)
- **Can be interrupted** by STOP command (S)
- **Sends** normal sensor data stream
- **Works alongside** encoder and IMU updates

To use manual control again, send `W:0` first.

## Performance Notes

- Control loop runs at 50Hz (LOOP_MS = 20)
- Sensor updates at 20Hz (50ms in `loopFirmware`)
- Motor updates at 100Hz (10ms in `loopFirmware`)
- Independent of sensor data transmission (100ms)

## Future Enhancements

Consider adding:
- PID control instead of P-only
- Left wall following mode
- Adaptive speed based on corridor width
- Maze mapping integration
- Configurable sensor indices via command
