#pragma once
#include <Arduino.h>
#include "header.h"

// ============================================================================
// MAZE NAVIGATION STATE MACHINE
// ============================================================================

// Main navigation states
enum MazeNavigationState {
  NAV_IDLE,                    // Not navigating
  NAV_MOVE_TO_CENTER,          // Moving to center of current cell
  NAV_AT_INTERSECTION,         // At intersection, detecting walls
  NAV_DECIDE_TURN,             // Deciding which direction to turn
  NAV_MOVE_FORWARD_CELL,       // Moving forward one cell
  NAV_TURN_RIGHT,              // Executing right turn
  NAV_TURN_LEFT,               // Executing left turn
  NAV_TURN_AROUND,             // Executing 180° turn (dead end)
  NAV_ALIGN_AFTER_TURN,        // Aligning after turn completion
  NAV_CALIBRATION_TURN         // Timing-based calibration turn
};

// Decision priorities for navigation (right-hand rule)
enum NavigationPriority {
  PRIORITY_TURN_RIGHT,         // Right is open - turn right
  PRIORITY_MOVE_FORWARD,       // Front is open - move forward  
  PRIORITY_TURN_LEFT,          // Left is open - turn left
  PRIORITY_TURN_AROUND,        // Dead end - turn around
  PRIORITY_NONE                // No valid option
};

// Direction facing (for tracking orientation)
enum Direction {
  DIR_NORTH = 0,
  DIR_EAST = 1,
  DIR_SOUTH = 2,
  DIR_WEST = 3
};

// ============================================================================
// STATE MACHINE DATA STRUCTURE
// ============================================================================

struct MazeNavigator {
  // Current state
  MazeNavigationState state;
  MazeNavigationState previous_state;
  
  // Timing control
  unsigned long state_start_time;
  unsigned long state_duration;
  unsigned long last_loop_time;
  
  // Sensor data (filtered)
  float filtered_left;
  float filtered_right;
  int raw_front;
  
  // Wall detection flags
  bool has_left_wall;
  bool has_right_wall;
  bool has_front_wall;
  
  // Position tracking (encoder-based)
  long start_encoder_left;
  long start_encoder_right;
  long encoder_target;
  float distance_traveled;      // Distance in meters
  float cell_distance_traveled; // Distance as fraction of cell (0.0 to 1.0+)
  
  // Cell and position tracking
  int cell_x;                   // X position in maze
  int cell_y;                   // Y position in maze
  Direction facing;             // Current direction robot is facing
  bool at_intersection;         // Flag indicating we're at an intersection
  bool turn_in_progress;        // Flag to prevent re-entering turn state
  bool intersection_handled;    // Flag to prevent re-detecting same intersection
  
  // Turn decision tracking
  NavigationPriority last_decision;
  unsigned long last_decision_time;
  
  // Calibration turn data
  bool is_calibration_turn;
  bool calibration_turn_is_right;
  int calibration_turn_duration;
  
  // Navigation enabled flag
  bool enabled;
  
  // Constructor
  MazeNavigator() :
    state(NAV_IDLE),
    previous_state(NAV_IDLE),
    state_start_time(0),
    state_duration(0),
    last_loop_time(0),
    filtered_left(0.0),
    filtered_right(0.0),
    raw_front(0),
    has_left_wall(false),
    has_right_wall(false),
    has_front_wall(false),
    start_encoder_left(0),
    start_encoder_right(0),
    encoder_target(0),
    distance_traveled(0.0),
    cell_distance_traveled(0.0),
    cell_x(0),
    cell_y(0),
    facing(DIR_NORTH),
    at_intersection(false),
    turn_in_progress(false),
    intersection_handled(false),
    last_decision(PRIORITY_NONE),
    last_decision_time(0),
    is_calibration_turn(false),
    calibration_turn_is_right(false),
    calibration_turn_duration(0),
    enabled(false)
  {}
  
  // Reset all state
  void reset() {
    state = NAV_IDLE;
    previous_state = NAV_IDLE;
    state_start_time = 0;
    state_duration = 0;
    filtered_left = 0.0;
    filtered_right = 0.0;
    raw_front = 0;
    has_left_wall = false;
    has_right_wall = false;
    has_front_wall = false;
    start_encoder_left = 0;
    start_encoder_right = 0;
    encoder_target = 0;
    distance_traveled = 0.0;
    cell_distance_traveled = 0.0;
    cell_x = 0;
    cell_y = 0;
    facing = DIR_NORTH;
    at_intersection = false;
    turn_in_progress = false;
    intersection_handled = false;
    last_decision = PRIORITY_NONE;
    last_decision_time = 0;
    is_calibration_turn = false;
  }
  
  // Start a new state
  void transition_to(MazeNavigationState new_state, unsigned long duration_ms = 0) {
    previous_state = state;
    state = new_state;
    state_start_time = millis();
    state_duration = duration_ms;
    
    // Debug output
    Serial.print("State transition: ");
    Serial.print(state_name(previous_state));
    Serial.print(" -> ");
    Serial.println(state_name(state));
  }
  
  // Check if current state is complete
  bool state_timeout_reached() {
    if (state_duration == 0) return false;
    return (millis() - state_start_time) >= state_duration;
  }
  
  // Get elapsed time in current state
  unsigned long state_elapsed_time() {
    return millis() - state_start_time;
  }
  
  // Helper: Get state name for debugging
  const char* state_name(MazeNavigationState s) {
    switch (s) {
      case NAV_IDLE: return "IDLE";
      case NAV_MOVE_TO_CENTER: return "MOVE_TO_CENTER";
      case NAV_AT_INTERSECTION: return "AT_INTERSECTION";
      case NAV_DECIDE_TURN: return "DECIDE_TURN";
      case NAV_MOVE_FORWARD_CELL: return "MOVE_FORWARD_CELL";
      case NAV_TURN_RIGHT: return "TURN_RIGHT";
      case NAV_TURN_LEFT: return "TURN_LEFT";
      case NAV_TURN_AROUND: return "TURN_AROUND";
      case NAV_ALIGN_AFTER_TURN: return "ALIGN_AFTER_TURN";
      case NAV_CALIBRATION_TURN: return "CALIBRATION_TURN";
      default: return "UNKNOWN";
    }
  }
  
  // Get direction name for debugging
  const char* direction_name(Direction d) {
    switch (d) {
      case DIR_NORTH: return "NORTH";
      case DIR_EAST: return "EAST";
      case DIR_SOUTH: return "SOUTH";
      case DIR_WEST: return "WEST";
      default: return "UNKNOWN";
    }
  }
  
  // Update facing direction after a turn
  void update_facing_after_turn(bool is_right) {
    if (is_right) {
      facing = (Direction)((facing + 1) % 4);
    } else {
      facing = (Direction)((facing + 3) % 4);
    }
  }
  
  // Update cell position based on facing direction
  void update_cell_position() {
    switch (facing) {
      case DIR_NORTH: cell_y++; break;
      case DIR_EAST:  cell_x++; break;
      case DIR_SOUTH: cell_y--; break;
      case DIR_WEST:  cell_x--; break;
    }
  }
};

// ============================================================================
// GLOBAL NAVIGATOR INSTANCE
// ============================================================================

MazeNavigator navigator;

// ============================================================================
// ENCODER AND DISTANCE TRACKING
// ============================================================================

// External encoder variables (defined in header.h)
extern volatile long countM1_;
extern volatile long countM2_;
extern volatile long countM3_;
extern volatile long countM4_;

// Initialize encoder tracking for new movement
void start_distance_tracking() {
  navigator.start_encoder_left = (countM1_ + countM2_) / 2;
  navigator.start_encoder_right = (countM3_ + countM4_) / 2;
  navigator.distance_traveled = 0.0;
  navigator.cell_distance_traveled = 0.0;
}

// Calculate distance traveled since start_distance_tracking()
void update_distance_traveled() {
  long current_left = (countM1_ + countM2_) / 2;
  long current_right = (countM3_ + countM4_) / 2;
  
  long delta_left = current_left - navigator.start_encoder_left;
  long delta_right = current_right - navigator.start_encoder_right;
  long avg_ticks = (delta_left + delta_right) / 2;
  
  // Convert ticks to meters
  navigator.distance_traveled = (float)avg_ticks / TICKS_PER_METER;
  
  // Convert to cell distance (0.0 to 1.0+ for fraction of cell traveled)
  navigator.cell_distance_traveled = navigator.distance_traveled / CELL_SIZE_M;
}

// Check if robot has traveled approximately one cell
bool has_traveled_one_cell() {
  update_distance_traveled();
  return navigator.cell_distance_traveled >= 0.95; // 95% of cell distance
}

// Check if intersection is approaching (for early detection)
bool is_approaching_intersection() {
  // Check if we've traveled enough distance before detecting next intersection
  // Must be at least 80% of a cell to avoid false detection
  if (navigator.cell_distance_traveled < MIN_CELL_DISTANCE) {
    return false;
  }
  
  // Track previous wall states to detect transitions
  static bool prev_had_left = true;  // Start assuming walls present
  static bool prev_had_right = true;
  static unsigned long last_transition_time = 0;
  unsigned long now = millis();
  
  // Detect wall disappearance (transition from wall to no wall)
  bool left_transition = prev_had_left && !navigator.has_left_wall;
  bool right_transition = prev_had_right && !navigator.has_right_wall;
  
  // Debounce: only detect transition if it's been stable for a bit
  bool transition_detected = false;
  if (left_transition || right_transition) {
    if (now - last_transition_time > 50) { // 50ms debounce
      transition_detected = true;
      last_transition_time = now;
    }
  }
  
  // Update previous states
  prev_had_left = navigator.has_left_wall;
  prev_had_right = navigator.has_right_wall;
  
  // Intersection detected if:
  // 1. We've traveled far enough AND
  // 2. Either a clear wall transition occurred, OR both sides are now open
  bool side_opening = (!navigator.has_left_wall || !navigator.has_right_wall);
  
  return transition_detected || (side_opening && navigator.cell_distance_traveled > 0.9);
}

// ============================================================================
// SENSOR READING AND FILTERING
// ============================================================================

void update_sensor_readings(uint16_t tof_distances[], int num_sensors) {
  // Read raw sensor values (assuming sensor indices: 0=left, 2=right, 5=front)
  int raw_left = tof_distances[0];
  int raw_right = tof_distances[2];
  int raw_front = tof_distances[5];
  
  // Apply low-pass filter to reduce noise
  navigator.filtered_left = ALPHA * raw_left + (1.0 - ALPHA) * navigator.filtered_left;
  navigator.filtered_right = ALPHA * raw_right + (1.0 - ALPHA) * navigator.filtered_right;
  navigator.raw_front = raw_front;
  
  // Update wall detection flags
  // Use larger threshold for front wall to stop earlier and avoid collision
  navigator.has_front_wall = (raw_front > 0 && raw_front < (FRONT_THRESHOLD + 100)); // Extra 50mm safety margin
  navigator.has_right_wall = (navigator.filtered_right > 0 && navigator.filtered_right < SIDE_WALL_THRESHOLD);
  navigator.has_left_wall = (navigator.filtered_left > 0 && navigator.filtered_left < SIDE_WALL_THRESHOLD);
  
  // Debug output (can be enabled/disabled)
  #ifdef DEBUG_SENSORS
  Serial.print("Sensors - L:");
  Serial.print(navigator.filtered_left);
  Serial.print(" R:");
  Serial.print(navigator.filtered_right);
  Serial.print(" F:");
  Serial.print(navigator.raw_front);
  Serial.print(" | Walls - L:");
  Serial.print(navigator.has_left_wall);
  Serial.print(" R:");
  Serial.print(navigator.has_right_wall);
  Serial.print(" F:");
  Serial.println(navigator.has_front_wall);
  #endif
}

// ============================================================================
// DECISION MAKING LOGIC (Right-Hand Rule)
// ============================================================================

NavigationPriority determine_navigation_priority() {
  // Right-hand rule priority:
  // 1. Turn right if possible (right is open)
  // 2. Go straight if possible (front is open)
  // 3. Turn left if possible (left is open)
  // 4. Turn around (dead end)
  
  bool can_go_right = !navigator.has_right_wall;
  bool can_go_forward = !navigator.has_front_wall;
  bool can_go_left = !navigator.has_left_wall;
  
  // Debug output
  Serial.print("Decision - L:");
  Serial.print(can_go_left ? "Open" : "Wall");
  Serial.print(" F:");
  Serial.print(can_go_forward ? "Open" : "Wall");
  Serial.print(" R:");
  Serial.println(can_go_right ? "Open" : "Wall");
  
  // Priority 1: Turn right (right-hand rule)
  if (can_go_right) {
    return PRIORITY_TURN_RIGHT;
  }
  
  // Priority 2: Go straight
  if (can_go_forward) {
    return PRIORITY_MOVE_FORWARD;
  }
  
  // Priority 3: Turn left
  if (can_go_left) {
    return PRIORITY_TURN_LEFT;
  }
  
  // Priority 4: Dead end - turn around
  return PRIORITY_TURN_AROUND;
}

// Get decision name for debugging
const char* decision_name(NavigationPriority p) {
  switch (p) {
    case PRIORITY_TURN_RIGHT: return "TURN_RIGHT";
    case PRIORITY_MOVE_FORWARD: return "MOVE_FORWARD";
    case PRIORITY_TURN_LEFT: return "TURN_LEFT";
    case PRIORITY_TURN_AROUND: return "TURN_AROUND";
    case PRIORITY_NONE: return "NONE";
    default: return "UNKNOWN";
  }
}

// ============================================================================
// MOTOR CONTROL HELPERS (Forward declarations - implement in firmware_control.h)
// ============================================================================

void execute_wall_following();
void execute_turn_right();
void execute_turn_left();
void execute_turn_around();
void execute_stop();
void execute_forward();
void sendPWM(int left_pwm, int right_pwm); // Low-level motor control

// ============================================================================
// STATE MACHINE EXECUTION
// ============================================================================

void maze_navigation_update(uint16_t tof_distances[], int num_sensors) {
  // Only run if navigation is enabled
  if (!navigator.enabled) {
    return;
  }
  
  // Rate limiting for main loop
  unsigned long now = millis();
  if (now - navigator.last_loop_time < LOOP_MS) {
    return;
  }
  navigator.last_loop_time = now;
  
  // Always update sensor readings and distance
  update_sensor_readings(tof_distances, num_sensors);
  update_distance_traveled();
  
  // EMERGENCY STOP: If front wall is very close in any forward-moving state
  if ((navigator.state == NAV_MOVE_FORWARD_CELL || navigator.state == NAV_MOVE_TO_CENTER) 
      && navigator.raw_front > 0 && navigator.raw_front < 200) {
    Serial.print("EMERGENCY STOP! Front wall at ");
    Serial.print(navigator.raw_front);
    Serial.println("mm!");
    execute_stop();
    delay(200);
    
    if (navigator.state == NAV_MOVE_FORWARD_CELL) {
      navigator.at_intersection = true;
      navigator.transition_to(NAV_AT_INTERSECTION);
      return; // Exit immediately
    }
  }
  
  // State machine
  switch (navigator.state) {
    
    // ========================================
    case NAV_IDLE:
      // Waiting for activation
      break;
    
    // ========================================
    case NAV_MOVE_TO_CENTER:
      {
        // Move forward to center of cell before starting navigation
        execute_wall_following();
        
        if (has_traveled_one_cell()) {
          Serial.println("Reached cell center - starting navigation");
          start_distance_tracking();
          navigator.intersection_handled = false;
          navigator.transition_to(NAV_MOVE_FORWARD_CELL);
        }
      }
      break;
    
    // ========================================
    case NAV_MOVE_FORWARD_CELL:
      {
        // CRITICAL: Check front wall first to avoid collision
        // Check at ANY distance if front wall is detected
        if (navigator.has_front_wall && navigator.cell_distance_traveled > 0.3) {
          // Front wall detected - STOP IMMEDIATELY
          Serial.print("FRONT WALL! Distance: ");
          Serial.print(navigator.raw_front);
          Serial.println("mm - STOPPING!");
          
          execute_stop();
          delay(150); // Full stop
          
          navigator.at_intersection = true;
          navigator.transition_to(NAV_AT_INTERSECTION);
          break;
        }
        
        // Check for intersection (opening on sides + distance traveled)
        if (is_approaching_intersection() && !navigator.intersection_handled) {
          Serial.println("Intersection detected!");
          Serial.print("Distance: ");
          Serial.print(navigator.cell_distance_traveled);
          Serial.println(" cells");
          
          execute_stop();
          delay(100); // Full stop before transitioning
          
          navigator.at_intersection = true;
          navigator.transition_to(NAV_AT_INTERSECTION);
          break;
        }
        
        // Only move if no obstacles detected
        execute_wall_following();
      }
      break;
    
    // ========================================
    case NAV_AT_INTERSECTION:
      {
        // Ensure completely stopped
        execute_stop();
        
        if (navigator.state_elapsed_time() >= 200) {
          // Sensors stable - check distance to front wall
          
          // Safety check: if front wall is too close, back up
          if (navigator.has_front_wall && navigator.raw_front < 180) {
            Serial.print("TOO CLOSE! Front: ");
            Serial.print(navigator.raw_front);
            Serial.println("mm - backing up!");
            
            // Back up more aggressively
            sendPWM(-120, -120);
            delay(150);
            execute_stop();
            delay(100);
            
            // Re-read sensors after backing up
            update_sensor_readings(tof_distances, num_sensors);
            
            Serial.print("After backup - Front: ");
            Serial.print(navigator.raw_front);
            Serial.println("mm");
          }
          
          Serial.print("At intersection (");
          Serial.print(navigator.cell_x);
          Serial.print(", ");
          Serial.print(navigator.cell_y);
          Serial.print(") facing ");
          Serial.print(navigator.direction_name(navigator.facing));
          Serial.print(" | Walls: L=");
          Serial.print(navigator.has_left_wall ? "Y" : "N");
          Serial.print(" F=");
          Serial.print(navigator.has_front_wall ? "Y" : "N");
          Serial.print(" (");
          Serial.print(navigator.raw_front);
          Serial.print("mm) R=");
          Serial.println(navigator.has_right_wall ? "Y" : "N");
          
          navigator.transition_to(NAV_DECIDE_TURN);
        }
      }
      break;
    
    // ========================================
    case NAV_DECIDE_TURN:
      {
        // Make turning decision based on right-hand rule
        if (navigator.turn_in_progress) {
          // Don't make new decisions while turning
          break;
        }
        
        NavigationPriority decision = determine_navigation_priority();
        navigator.last_decision = decision;
        navigator.last_decision_time = millis();
        
        Serial.print("Decision: ");
        Serial.println(decision_name(decision));
        
        // Set turn_in_progress flag to prevent re-entry
        navigator.turn_in_progress = true;
        navigator.intersection_handled = true;
        
        switch (decision) {
          case PRIORITY_TURN_RIGHT:
            navigator.transition_to(NAV_TURN_RIGHT, RIGHT_TURN_DELAY);
            break;
            
          case PRIORITY_MOVE_FORWARD:
            // Move forward through intersection
            Serial.println("Continuing forward");
            start_distance_tracking();
            navigator.update_cell_position();
            navigator.turn_in_progress = false;
            navigator.intersection_handled = false;
            navigator.transition_to(NAV_MOVE_FORWARD_CELL);
            break;
            
          case PRIORITY_TURN_LEFT:
            navigator.transition_to(NAV_TURN_LEFT, LEFT_TURN_DELAY);
            break;
            
          case PRIORITY_TURN_AROUND:
            navigator.transition_to(NAV_TURN_AROUND, LEFT_TURN_DELAY * 2);
            break;
            
          default:
            Serial.println("ERROR: No valid decision!");
            navigator.turn_in_progress = false;
            break;
        }
      }
      break;
    
    // ========================================
    case NAV_TURN_RIGHT:
      execute_turn_right();
      
      if (navigator.state_timeout_reached()) {
        Serial.println("Right turn complete");
        execute_stop();
        delay(100);
        
        // Update facing direction
        navigator.update_facing_after_turn(true);
        
        Serial.print("Now facing: ");
        Serial.println(navigator.direction_name(navigator.facing));
        
        // Transition to alignment
        navigator.transition_to(NAV_ALIGN_AFTER_TURN, 200);
      }
      break;
    
    // ========================================
    case NAV_TURN_LEFT:
      execute_turn_left();
      
      if (navigator.state_timeout_reached()) {
        Serial.println("Left turn complete");
        execute_stop();
        delay(100);
        
        // Update facing direction
        navigator.update_facing_after_turn(false);
        
        Serial.print("Now facing: ");
        Serial.println(navigator.direction_name(navigator.facing));
        
        // Transition to alignment
        navigator.transition_to(NAV_ALIGN_AFTER_TURN, 200);
      }
      break;
    
    // ========================================
    case NAV_TURN_AROUND:
      execute_turn_left(); // Turn around = two left turns worth
      
      if (navigator.state_timeout_reached()) {
        Serial.println("Turn around complete");
        execute_stop();
        delay(100);
        
        // Update facing direction (180 degrees = 2 left turns)
        navigator.update_facing_after_turn(false);
        navigator.update_facing_after_turn(false);
        
        Serial.print("Now facing: ");
        Serial.println(navigator.direction_name(navigator.facing));
        
        // Transition to alignment
        navigator.transition_to(NAV_ALIGN_AFTER_TURN, 200);
      }
      break;
    
    // ========================================
    case NAV_ALIGN_AFTER_TURN:
      {
        // Brief forward movement to align in new corridor
        execute_forward();
        
        if (navigator.state_timeout_reached()) {
          Serial.println("Alignment complete - resuming navigation");
          execute_stop();
          delay(50);
          
          // Update cell position based on new direction
          navigator.update_cell_position();
          
          // Reset tracking for next cell
          start_distance_tracking();
          navigator.turn_in_progress = false;
          navigator.intersection_handled = false;
          
          // Resume forward movement
          navigator.transition_to(NAV_MOVE_FORWARD_CELL);
        }
      }
      break;
    
    // ========================================
    case NAV_CALIBRATION_TURN:
      // Execute timing-based turn for calibration
      if (navigator.is_calibration_turn) {
        if (navigator.calibration_turn_is_right) {
          execute_turn_right();
        } else {
          execute_turn_left();
        }
        
        if (navigator.state_timeout_reached()) {
          Serial.print("Calibration turn complete - Duration: ");
          Serial.print(navigator.calibration_turn_duration);
          Serial.println(" ms");
          execute_stop();
          navigator.is_calibration_turn = false;
          navigator.transition_to(NAV_IDLE);
        }
      }
      break;
    
    // ========================================
    default:
      Serial.println("ERROR: Unknown state");
      navigator.transition_to(NAV_IDLE);
      break;
  }
}

// ============================================================================
// PUBLIC API FUNCTIONS
// ============================================================================

void start_maze_navigation() {
  Serial.println("=== Starting Maze Navigation ===");
  Serial.println("=== Using Right-Hand Rule ===");
  navigator.reset();
  navigator.enabled = true;
  start_distance_tracking();
  
  // Start by moving to center of first cell
  navigator.transition_to(NAV_MOVE_TO_CENTER);
  
  Serial.print("Starting position: (");
  Serial.print(navigator.cell_x);
  Serial.print(", ");
  Serial.print(navigator.cell_y);
  Serial.print(") facing ");
  Serial.println(navigator.direction_name(navigator.facing));
}

void stop_maze_navigation() {
  Serial.println("=== Stopping Maze Navigation ===");
  navigator.enabled = false;
  navigator.transition_to(NAV_IDLE);
  execute_stop();
}

bool is_maze_navigation_active() {
  return navigator.enabled && navigator.state != NAV_IDLE;
}

void execute_calibration_turn_right(int duration_ms) {
  Serial.print("Starting calibration right turn for ");
  Serial.print(duration_ms);
  Serial.println(" ms");
  
  navigator.is_calibration_turn = true;
  navigator.calibration_turn_is_right = true;
  navigator.calibration_turn_duration = duration_ms;
  navigator.transition_to(NAV_CALIBRATION_TURN, duration_ms);
}

void execute_calibration_turn_left(int duration_ms) {
  Serial.print("Starting calibration left turn for ");
  Serial.print(duration_ms);
  Serial.println(" ms");
  
  navigator.is_calibration_turn = true;
  navigator.calibration_turn_is_right = false;
  navigator.calibration_turn_duration = duration_ms;
  navigator.transition_to(NAV_CALIBRATION_TURN, duration_ms);
}
