#pragma once
#include <Arduino.h>
#include "header.h"

// ============================================================================
// MAZE NAVIGATION STATE MACHINE
// ============================================================================

// Main navigation states
enum MazeNavigationState {
  NAV_IDLE,                    // Not navigating
  NAV_CHECK_SURROUNDINGS,      // Check walls and decide next action
  NAV_MOVE_FORWARD,            // Moving forward while wall-following
  NAV_TURN_RIGHT,              // Executing right turn
  NAV_TURN_LEFT,               // Executing left turn
  NAV_TURN_FRONT,              // Executing front wall turn (180° or 90° left)
  NAV_POST_TURN_FORWARD,       // Moving forward after completing a turn
  NAV_CALIBRATION_TURN         // Timing-based calibration turn
};

// Decision priorities for navigation
enum NavigationPriority {
  PRIORITY_FRONT_WALL,         // Front wall detected - must turn
  PRIORITY_NO_RIGHT_WALL,      // Right wall missing - turn right (right-hand rule)
  PRIORITY_NO_LEFT_WALL,       // Left wall missing - turn left
  PRIORITY_WALL_FOLLOW,        // Both walls present - follow wall
  PRIORITY_NONE                // No decision needed
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
  
  // Turn tracking
  bool completed_right_turn;
  bool completed_left_turn;
  bool completed_front_turn;
  
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
    completed_right_turn(false),
    completed_left_turn(false),
    completed_front_turn(false),
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
    completed_right_turn = false;
    completed_left_turn = false;
    completed_front_turn = false;
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
      case NAV_CHECK_SURROUNDINGS: return "CHECK_SURROUNDINGS";
      case NAV_MOVE_FORWARD: return "MOVE_FORWARD";
      case NAV_TURN_RIGHT: return "TURN_RIGHT";
      case NAV_TURN_LEFT: return "TURN_LEFT";
      case NAV_TURN_FRONT: return "TURN_FRONT";
      case NAV_POST_TURN_FORWARD: return "POST_TURN_FORWARD";
      case NAV_CALIBRATION_TURN: return "CALIBRATION_TURN";
      default: return "UNKNOWN";
    }
  }
};

// ============================================================================
// GLOBAL NAVIGATOR INSTANCE
// ============================================================================

MazeNavigator navigator;

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
  navigator.has_front_wall = (raw_front > 0 && raw_front < FRONT_THRESHOLD);
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
// DECISION MAKING LOGIC
// ============================================================================

NavigationPriority determine_navigation_priority() {
  // Priority 1: Front wall - must turn
  if (navigator.has_front_wall) {
    return PRIORITY_FRONT_WALL;
  }
  
  // Priority 2: No right wall - turn right (right-hand rule)
  if (!navigator.has_right_wall) {
    return PRIORITY_NO_RIGHT_WALL;
  }
  
  // Priority 3: No left wall - turn left
  if (!navigator.has_left_wall) {
    return PRIORITY_NO_LEFT_WALL;
  }
  
  // Priority 4: Both walls present - wall following
  if (navigator.has_right_wall && navigator.has_left_wall) {
    return PRIORITY_WALL_FOLLOW;
  }
  
  return PRIORITY_NONE;
}

// ============================================================================
// MOTOR CONTROL HELPERS (Forward declarations - implement in firmware_control.h)
// ============================================================================

void execute_wall_following();
void execute_turn_right();
void execute_turn_left();
void execute_turn_front();
void execute_stop();
void execute_forward();

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
  
  // Update sensor readings and wall detection
  update_sensor_readings(tof_distances, num_sensors);
  
  // State machine
  switch (navigator.state) {
    
    // ========================================
    case NAV_IDLE:
      // Waiting for activation
      break;
    
    // ========================================
    case NAV_CHECK_SURROUNDINGS:
      {
        // Determine what action to take based on sensor readings
        NavigationPriority priority = determine_navigation_priority();
        
        Serial.print("Priority: ");
        switch (priority) {
          case PRIORITY_FRONT_WALL:
            Serial.println("FRONT_WALL");
            navigator.transition_to(NAV_TURN_FRONT, TURN_DELAY);
            execute_stop();
            delay(100);
            break;
            
          case PRIORITY_NO_RIGHT_WALL:
            Serial.println("NO_RIGHT_WALL - turning right");
            navigator.transition_to(NAV_TURN_RIGHT, RIGHT_TURN_DELAY);
            navigator.completed_right_turn = true;
            navigator.completed_left_turn = false;
            navigator.completed_front_turn = false;
            break;
            
          case PRIORITY_NO_LEFT_WALL:
            Serial.println("NO_LEFT_WALL - turning left");
            navigator.transition_to(NAV_TURN_LEFT, LEFT_TURN_DELAY);
            navigator.completed_left_turn = true;
            navigator.completed_right_turn = false;
            navigator.completed_front_turn = false;
            break;
            
          case PRIORITY_WALL_FOLLOW:
            Serial.println("WALL_FOLLOW");
            navigator.transition_to(NAV_MOVE_FORWARD);
            break;
            
          default:
            // No clear decision - stay in check state
            Serial.println("NONE - rechecking");
            break;
        }
      }
      break;
    
    // ========================================
    case NAV_MOVE_FORWARD:
      // Execute wall following with centering correction
      execute_wall_following();
      
      // Continuously check if we need to change direction
      navigator.transition_to(NAV_CHECK_SURROUNDINGS);
      break;
    
    // ========================================
    case NAV_TURN_RIGHT:
      execute_turn_right();
      
      if (navigator.state_timeout_reached()) {
        Serial.println("Right turn complete");
        execute_stop();
        delay(100); // Brief pause to stabilize
        
        // Update sensors after turn
        update_sensor_readings(tof_distances, num_sensors);
        
        // Check if front is clear and we should move forward
        bool front_clear = !navigator.has_front_wall;
        bool opposite_wall = navigator.has_left_wall; // After right turn, left wall appears
        
        if (front_clear && opposite_wall) {
          Serial.println("Front clear after right turn - moving forward");
          navigator.transition_to(NAV_POST_TURN_FORWARD, FORWARD_AFTER_TURN_DURATION);
        } else {
          Serial.println("Front blocked or no wall - checking surroundings");
          navigator.completed_right_turn = false;
          navigator.transition_to(NAV_CHECK_SURROUNDINGS);
        }
      }
      break;
    
    // ========================================
    case NAV_TURN_LEFT:
      execute_turn_left();
      
      if (navigator.state_timeout_reached()) {
        Serial.println("Left turn complete");
        execute_stop();
        delay(100);
        
        update_sensor_readings(tof_distances, num_sensors);
        
        bool front_clear = !navigator.has_front_wall;
        bool opposite_wall = navigator.has_right_wall; // After left turn, right wall appears
        
        if (front_clear && opposite_wall) {
          Serial.println("Front clear after left turn - moving forward");
          navigator.transition_to(NAV_POST_TURN_FORWARD, FORWARD_AFTER_TURN_DURATION);
        } else {
          Serial.println("Front blocked or no wall - checking surroundings");
          navigator.completed_left_turn = false;
          navigator.transition_to(NAV_CHECK_SURROUNDINGS);
        }
      }
      break;
    
    // ========================================
    case NAV_TURN_FRONT:
      execute_turn_front();
      
      if (navigator.state_timeout_reached()) {
        Serial.println("Front wall turn complete");
        execute_stop();
        delay(100);
        
        update_sensor_readings(tof_distances, num_sensors);
        
        bool front_clear = !navigator.has_front_wall;
        
        if (front_clear) {
          Serial.println("Front clear after front turn - moving forward");
          navigator.completed_front_turn = true;
          navigator.transition_to(NAV_POST_TURN_FORWARD, FORWARD_AFTER_TURN_DURATION);
        } else {
          Serial.println("Front still blocked - checking surroundings");
          navigator.completed_front_turn = false;
          navigator.transition_to(NAV_CHECK_SURROUNDINGS);
        }
      }
      break;
    
    // ========================================
    case NAV_POST_TURN_FORWARD:
      execute_forward();
      
      if (navigator.state_timeout_reached()) {
        Serial.println("Post-turn forward complete");
        execute_stop();
        delay(50);
        
        // Reset turn completion flags
        navigator.completed_right_turn = false;
        navigator.completed_left_turn = false;
        navigator.completed_front_turn = false;
        
        // Check surroundings for next action
        navigator.transition_to(NAV_CHECK_SURROUNDINGS);
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
  navigator.reset();
  navigator.enabled = true;
  navigator.transition_to(NAV_CHECK_SURROUNDINGS);
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
