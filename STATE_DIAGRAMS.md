# State Machine Visual Diagrams

## Main Navigation Flow

```
                                    ┌─────────────┐
                                    │  NAV_IDLE   │
                                    │  (Waiting)  │
                                    └──────┬──────┘
                                           │
                                      'W' Command
                                           │
                                           ▼
                    ┌──────────────────────────────────────────┐
                    │     NAV_CHECK_SURROUNDINGS               │
                    │  - Read ToF sensors (L, R, F)            │
                    │  - Apply filters                         │
                    │  - Determine priority                    │
                    │  - Decide next action                    │
                    └─────────┬────────────────────────────────┘
                              │
        ┌─────────────────────┼──────────────────────┐
        │                     │                      │
        │ PRIORITY:           │ PRIORITY:            │ PRIORITY:
        │ FRONT_WALL          │ NO_RIGHT_WALL        │ NO_LEFT_WALL
        │                     │                      │
        ▼                     ▼                      ▼
┌──────────────┐      ┌──────────────┐      ┌──────────────┐
│ NAV_TURN_    │      │ NAV_TURN_    │      │ NAV_TURN_    │
│ FRONT        │      │ RIGHT        │      │ LEFT         │
│              │      │              │      │              │
│ Duration:    │      │ Duration:    │      │ Duration:    │
│ 400ms        │      │ 1100ms       │      │ 900ms        │
└──────┬───────┘      └──────┬───────┘      └──────┬───────┘
       │                     │                      │
       └─────────────────────┼──────────────────────┘
                             │
                   Check if front clear
                   && opposite wall exists
                             │
              ┌──────────────┴───────────────┐
              │ YES                          │ NO
              ▼                              ▼
    ┌──────────────────┐           ┌─────────────────┐
    │ NAV_POST_TURN_   │           │  Back to        │
    │ FORWARD          │           │  CHECK_         │
    │                  │           │  SURROUNDINGS   │
    │ Duration: 400ms  │           └─────────────────┘
    └────────┬─────────┘
             │
             │ Timeout reached
             │
             ▼
    ┌─────────────────┐
    │  Back to        │
    │  CHECK_         │
    │  SURROUNDINGS   │
    └─────────────────┘


              │ PRIORITY: WALL_FOLLOW
              │ (Both walls present)
              ▼
    ┌──────────────────────┐
    │  NAV_MOVE_FORWARD    │
    │                      │
    │  - Calculate error   │
    │  - Apply PID         │
    │  - Center robot      │
    │                      │
    └──────────┬───────────┘
               │
               │ Immediately return to
               ▼
    ┌──────────────────────┐
    │  NAV_CHECK_          │
    │  SURROUNDINGS        │
    └──────────────────────┘
```

## Priority Decision Tree

```
Check Sensors
     │
     ├─→ Front < 200mm? ──YES──→ PRIORITY_FRONT_WALL
     │         │
     │        NO
     │         │
     ├─→ Right > 300mm? ──YES──→ PRIORITY_NO_RIGHT_WALL
     │         │
     │        NO
     │         │
     ├─→ Left > 300mm? ──YES──→ PRIORITY_NO_LEFT_WALL
     │         │
     │        NO
     │         │
     └─→ Both walls present? ──YES──→ PRIORITY_WALL_FOLLOW
```

## Sensor Layout

```
                    [Front Sensor]
                         (5)
                          │
                          │
              ┌───────────┴───────────┐
              │                       │
              │                       │
              │                       │
[Left Sensor] │       ROBOT           │ [Right Sensor]
    (0)       │                       │     (2)
              │                       │
              │                       │
              └───────────────────────┘
                          │
                          │
                    [Rear sensors]
                      (1,3,4,6)
```

## State Timing Diagram

```
Time ──────────────────────────────────────────────────────►

CHECK_SURROUNDINGS
│ (~20ms)
├─────────────► TURN_RIGHT
                │ (1100ms)
                ├──────────────► POST_TURN_FORWARD
                                 │ (400ms)
                                 ├────────────► CHECK_SURROUNDINGS
                                                │
                                                ▼
                                            (Continue...)
```

## Wall Following Correction Logic

```
                Left Sensor        Right Sensor
                    │                   │
                    ▼                   ▼
                 filtL              filtR
                    │                   │
                    └────────┬──────────┘
                             │
                             ▼
                    error = filtR - filtL
                             │
                             ▼
                      corr = Kp × error
                             │
                    ┌────────┴────────┐
                    │                 │
          leftPWM = BASE + corr   rightPWM = BASE - corr
                    │                 │
                    └────────┬────────┘
                             │
                             ▼
                    Apply to motors
                    (Robot centers)

Example:
  filtL = 150mm, filtR = 200mm
  error = 200 - 150 = 50 (too close to right)
  corr = 0.2 × 50 = 10
  leftPWM = 150 + 10 = 160 (speed up left)
  rightPWM = 150 - 10 = 140 (slow down right)
  → Robot moves left to center
```

## Turn Decision Matrix

```
┌──────────┬────────────┬────────────┬──────────────┐
│  Front   │   Right    │    Left    │   Action     │
├──────────┼────────────┼────────────┼──────────────┤
│  WALL    │     X      │     X      │  TURN_FRONT  │
│  CLEAR   │   NO WALL  │     X      │  TURN_RIGHT  │
│  CLEAR   │   WALL     │  NO WALL   │  TURN_LEFT   │
│  CLEAR   │   WALL     │   WALL     │  MOVE_FORWARD│
└──────────┴────────────┴────────────┴──────────────┘

X = Don't care (any value)
Priority: Front > Right > Left > Forward
```

## Post-Turn Validation

```
After RIGHT Turn:
┌─────────────────────────────────────────────┐
│  Front Clear? ───NO──► Back to CHECK        │
│       │                                      │
│      YES                                     │
│       │                                      │
│  Left Wall Exists? ───NO──► Back to CHECK   │
│       │                                      │
│      YES                                     │
│       │                                      │
│  ──► MOVE FORWARD 400ms                      │
└─────────────────────────────────────────────┘

After LEFT Turn:
┌─────────────────────────────────────────────┐
│  Front Clear? ───NO──► Back to CHECK        │
│       │                                      │
│      YES                                     │
│       │                                      │
│  Right Wall Exists? ───NO──► Back to CHECK  │
│       │                                      │
│      YES                                     │
│       │                                      │
│  ──► MOVE FORWARD 400ms                      │
└─────────────────────────────────────────────┘
```

## Calibration Turn Flow

```
Serial Command: 'r:1100' or 'l:900'
         │
         ▼
┌─────────────────────┐
│ Set calibration     │
│ parameters:         │
│  - is_right/left    │
│  - duration         │
│  - state = CAL_TURN │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ Execute turn at     │
│ TURN_SPEED          │
│                     │
│ Monitor elapsed     │
│ time                │
└──────────┬──────────┘
           │
           │ Duration reached?
           ▼
┌─────────────────────┐
│ Stop motors         │
│ Print result        │
│ Return to IDLE      │
└─────────────────────┘
```

## Sensor Filtering

```
Raw Reading ──┐
              │
              ▼
      ┌───────────────┐
      │  Low-Pass     │
      │  Filter       │
      │               │
      │ filtered =    │
      │ α×raw +       │
      │ (1-α)×prev    │
      └───────┬───────┘
              │
              ▼
      Filtered Value

α = 0.3 (ALPHA)
  - Higher α = more responsive (follows raw quickly)
  - Lower α = smoother (filters noise better)
```

## Complete State Machine

```
         ┌──────────────────────────────────────────────┐
         │                                              │
    ┌────▼─────┐         ┌──────────────────┐          │
    │   IDLE   │◄────────│  CALIBRATION_    │          │
    │          │         │  TURN            │          │
    └────┬─────┘         └──────────────────┘          │
         │ 'W'                    ▲                     │
         ▼                        │                     │
    ┌──────────────────┐    'r' or 'l' cmd            │
    │  CHECK_          │◄───────┘                      │
    │  SURROUNDINGS    │◄─────────────────┐            │
    └─────┬────────────┘                  │            │
          │                               │            │
    ┌─────┼─────┬──────────┬──────────────┤            │
    │     │     │          │              │            │
    ▼     ▼     ▼          ▼              │            │
 ┌────┐ ┌────┐ ┌────┐  ┌────────┐        │            │
 │TURN│ │TURN│ │TURN│  │ MOVE_  │        │            │
 │FRNT│ │RGHT│ │LEFT│  │ FORWARD│────────┘            │
 └─┬──┘ └─┬──┘ └─┬──┘  └────────┘                     │
   │      │      │                                     │
   └──────┼──────┘                                     │
          │                                            │
          ▼                                            │
    ┌──────────────────┐                              │
    │  POST_TURN_      │                              │
    │  FORWARD         │──────────────────────────────┘
    └──────────────────┘
```

## Execution Timeline Example

```
Time    State                Action               Sensors
────────────────────────────────────────────────────────────
0ms     CHECK               Read sensors         L:150 R:200 F:500
20ms    MOVE_FORWARD        Wall follow          L:145 R:205 F:480
40ms    CHECK               Read sensors         L:140 R:210 F:450
...
2000ms  CHECK               Read sensors         L:150 R:0 F:400
2020ms  TURN_RIGHT          Start right turn     Turning...
3120ms  TURN_RIGHT          Complete (1100ms)    L:0 R:150 F:450
3120ms  POST_TURN_FORWARD   Move forward         Moving...
3520ms  CHECK               Complete (400ms)     L:0 R:150 F:500
3540ms  TURN_RIGHT          No right wall        Turning...
4640ms  POST_TURN_FORWARD   Complete turn        Moving...
5040ms  CHECK               Resume               L:150 R:180 F:600
5060ms  MOVE_FORWARD        Both walls           Centering...
```

## Error Recovery

```
State Stuck?
     │
     ├─→ Timeout mechanism in each state
     │   (state_duration variable)
     │
     ├─→ Safety stop if no command
     │   (COMMAND_TIMEOUT = 1000ms)
     │
     └─→ 'S' or 'W' command to reset
         Return to IDLE state
```

---

*Use these diagrams as reference while debugging or understanding the robot's behavior.*
