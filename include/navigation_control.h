#pragma once
#include <Arduino.h>
#include "header.h"
#include "motor_driver.h"
#include "motor_control.h"
#include "tof_main.h"
#include "bno055-new.h"

// ========================================
// NAVIGATION CONTROL CONSTANTS
// ========================================

// Wall centering control parameters
#define BASE_FORWARD_SPEED 200       // Base PWM speed for forward movement
#define KP_CENTERING 0.5f            // Proportional gain for centering (tunable)
#define CENTERING_MAX_CORRECTION 50  // Maximum PWM correction for centering
#define WALL_DETECTION_THRESHOLD 300 // Maximum distance (mm) to consider wall valid
#define MIN_FORWARD_SPEED 0.1f       // Minimum forward speed to apply centering

// Sensor indices (adjust based on your robot's sensor layout)
#define SENSOR_LEFT_SIDE 0    // Left side sensor index
#define SENSOR_RIGHT_SIDE 6   // Right side sensor index

// Discrete turning parameters
#define TURN_TOLERANCE_DEG 2.0f      // Acceptable error in degrees
#define TURN_TOLERANCE_RAD 0.0349f   // ~2 degrees in radians
#define TURN_TIMEOUT_MS 5000         // Maximum time for a turn (ms)
#define TURN_SPEED_PWM 230           // PWM speed for turning (0-255)

// Angle constants
#define DEG_TO_RAD_FACTOR (PI / 180.0f)
#define RAD_TO_DEG_FACTOR (180.0f / PI)

// ========================================
// HELPER FUNCTIONS
// ========================================

/**
 * @brief Normalize angle to [-PI, PI] range
 * @param angle Angle in radians
 * @return Normalized angle in radians
 */
inline float normalize_angle(float angle) {
    while (angle > PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

/**
 * @brief Convert encoder ticks to distance traveled
 * @param ticks Number of encoder ticks
 * @return Distance in meters
 */
inline float ticks_to_distance(long ticks) {
    return (float)ticks * 2.0f * PI * WHEEL_RADIUS / TICKS_PER_REV;
}

/**
 * @brief Convert distance to encoder ticks
 * @param distance Distance in meters
 * @return Number of encoder ticks
 */
inline long distance_to_ticks(float distance) {
    return (long)(distance * TICKS_PER_REV / (2.0f * PI * WHEEL_RADIUS));
}

/**
 * @brief Calculate arc length for a given angle and wheelbase
 * @param angle_rad Angle in radians
 * @return Arc length in meters
 */
inline float angle_to_arc_length(float angle_rad) {
    // For differential drive: arc_length = angle * (wheel_track / 2)
    return fabs(angle_rad) * (WHEEL_TRACK / 2.0f);
}

/**
 * @brief Calculate expected encoder ticks for a turn
 * @param angle_rad Desired turn angle in radians
 * @return Expected encoder ticks per wheel
 */
inline long angle_to_ticks(float angle_rad) {
    float arc_length = angle_to_arc_length(angle_rad);
    return distance_to_ticks(arc_length);
}

/**
 * @brief Bound a value between min and max
 * @param value Value to bound
 * @param min Minimum value
 * @param max Maximum value
 */
inline void bound_value(float &value, float min, float max) {
    if (value < min) value = min;
    else if (value > max) value = max;
}

/**
 * @brief Convert degrees to radians
 * @param degrees Angle in degrees
 * @return Angle in radians
 */
inline float deg_to_rad(float degrees) {
    return degrees * DEG_TO_RAD_FACTOR;
}

/**
 * @brief Convert radians to degrees
 * @param radians Angle in radians
 * @return Angle in degrees
 */
inline float rad_to_deg(float radians) {
    return radians * RAD_TO_DEG_FACTOR;
}

// ========================================
// WALL CENTERING CONTROL
// ========================================

/**
 * @brief Calculate proportional correction to center robot between walls
 * @return PWM correction value (positive = correct right, negative = correct left)
 */
float kp_centering_control() {
    // Get side sensor readings (in millimeters)
    uint16_t left_distance = getToFDistance(SENSOR_LEFT_SIDE);
    uint16_t right_distance = getToFDistance(SENSOR_RIGHT_SIDE);
    
    // Check if both walls are detected
    bool left_valid = (left_distance > 0 && left_distance < WALL_DETECTION_THRESHOLD);
    bool right_valid = (right_distance > 0 && right_distance < WALL_DETECTION_THRESHOLD);
    
    // If no valid walls detected, return no correction
    if (!left_valid && !right_valid) {
        return 0.0f;
    }
    
    // If only one wall is valid, use single-wall following
    if (left_valid && !right_valid) {
        // Follow left wall only - maintain constant distance
        float desired_distance = 150.0f; // 150mm from left wall
        float error = left_distance - desired_distance;
        float correction = KP_CENTERING * error;
        bound_value(correction, -CENTERING_MAX_CORRECTION, CENTERING_MAX_CORRECTION);
        return correction;
    }
    
    if (right_valid && !left_valid) {
        // Follow right wall only - maintain constant distance
        float desired_distance = 150.0f; // 150mm from right wall
        float error = desired_distance - right_distance;
        float correction = KP_CENTERING * error;
        bound_value(correction, -CENTERING_MAX_CORRECTION, CENTERING_MAX_CORRECTION);
        return correction;
    }
    
    // Both walls detected - center between them
    // Positive error means robot is too close to left wall (need to go right)
    float error = (float)left_distance - (float)right_distance;
    
    // Apply proportional control
    float correction = KP_CENTERING * error;
    
    // Limit correction to prevent aggressive movements
    bound_value(correction, -CENTERING_MAX_CORRECTION, CENTERING_MAX_CORRECTION);
    
    return correction;
}

/**
 * @brief Apply centering correction to motor speeds
 * @param correction Centering correction value
 * @param adjusted_left Output adjusted left motor speed
 * @param adjusted_right Output adjusted right motor speed
 */
void apply_centering_correction(float correction,
                                int &adjusted_left, int &adjusted_right) {
    // Start with base forward speed
    int base_speed = BASE_FORWARD_SPEED;
    
    // Positive correction means robot should turn right (increase left, decrease right)
    adjusted_left = base_speed + (int)correction;
    adjusted_right = base_speed - (int)correction;
    
    // Clamp speeds to valid PWM range [0, 255]
    if (adjusted_left < 0) adjusted_left = 0;
    if (adjusted_left > 255) adjusted_left = 255;
    if (adjusted_right < 0) adjusted_right = 0;
    if (adjusted_right > 255) adjusted_right = 255;
}

// ========================================
// DISCRETE TURNING CONTROL
// ========================================

/**
 * @brief Perform a discrete turn using IMU feedback (no PID, direct PWM control)
 * @param target_angle_deg Target angle in degrees (positive = left, negative = right)
 * @return true if turn completed successfully, false if timeout or error
 */
bool discrete_turn(float target_angle_deg) {
    // Convert target angle to radians
    float target_angle_rad = deg_to_rad(target_angle_deg);
    
    Serial.println("======================================");
    Serial.print("Starting turn: ");
    Serial.print(target_angle_deg);
    Serial.println(" degrees");
    
    // Update IMU to get fresh reading
    update_imu();
    
    // Record initial state
    float initial_yaw = rad_yaw;
    
    // Calculate target heading
    float target_yaw = normalize_angle(initial_yaw + target_angle_rad);
    
    Serial.print("Initial yaw: ");
    Serial.print(rad_to_deg(initial_yaw), 2);
    Serial.print(" deg | Target yaw: ");
    Serial.print(rad_to_deg(target_yaw), 2);
    Serial.print(" deg | Delta: ");
    Serial.print(target_angle_deg, 2);
    Serial.println(" deg");
    
    // Stop any current motion
    moveStop();
    
    unsigned long start_time = millis();
    bool turn_complete = false;
    bool slowing_down = false;
    
    // Determine turn direction
    bool turning_left = (target_angle_deg > 0);
    Serial.print("Direction: ");
    Serial.println(turning_left ? "LEFT (CCW)" : "RIGHT (CW)");
    
    unsigned long last_debug = 0;
    int turn_speed = TURN_SPEED_PWM;  // Declare here so it's in scope for the whole loop
    
    // Main turning loop (NO DELAYS!)
    while (!turn_complete && (millis() - start_time < TURN_TIMEOUT_MS)) {
        // Update IMU data
        update_imu();
        
        // Calculate how much we've turned from start (THIS IS THE PRIMARY METRIC!)
        float turned_so_far = normalize_angle(rad_yaw - initial_yaw);
        float turned_deg = rad_to_deg(turned_so_far);
        
        // Calculate remaining turn needed
        float remaining_deg = fabs(target_angle_deg) - fabs(turned_deg);
        
        // Debug output (throttled)
        if (millis() - last_debug > 100) {
            Serial.print("Current: ");
            Serial.print(rad_to_deg(rad_yaw), 2);
            Serial.print("° | Turned: ");
            Serial.print(turned_deg, 2);
            Serial.print("° | Remaining: ");
            Serial.print(remaining_deg, 2);
            Serial.println("°");
            last_debug = millis();
        }
        
        // Check if we've turned enough (USE TURNED AMOUNT, NOT ERROR TO TARGET!)
        if (fabs(turned_deg) >= fabs(target_angle_deg) - TURN_TOLERANCE_DEG) {
            Serial.println("=== TARGET REACHED! STOPPING ===");
            // Gentle deceleration to prevent motor ringing
            int slow_speed = turn_speed / 2;
            if (turning_left) {
                turnLeft(slow_speed, slow_speed);
            } else {
                turnRight(slow_speed, slow_speed);
            }
            delay(30);  // Brief deceleration period
            moveStop();
            turn_complete = true;
            break;
        }
        
        // Safety: if we've turned more than expected, stop
        if (fabs(turned_deg) > fabs(target_angle_deg) + 10.0f) {
            Serial.println("=== OVERSHOOT DETECTED! STOPPING ===");
            moveStop();
            turn_complete = true;
            break;
        }
        
        if (remaining_deg < 30.0f) {
            turn_speed = 200;
        } else {
            turn_speed = TURN_SPEED_PWM;  // 100% speed
        }
        
        // Apply PWM based on turn direction
        if (turning_left) {
            turnLeft(turn_speed, turn_speed);
        } else {
            turnRight(turn_speed, turn_speed);
        }
    }
    
    // CRITICAL: Stop turning immediately
    moveStop();
    delay(50);  // Brief settling time to prevent motor ringing
    
    // Final IMU read
    update_imu();
    
    // Calculate final results
    float final_error = normalize_angle(target_yaw - rad_yaw);
    float final_error_deg = rad_to_deg(fabs(final_error));
    
    // Calculate actual turn amount (preserve sign to match requested direction)
    float total_turned_rad = normalize_angle(rad_yaw - initial_yaw);
    
    // If the sign doesn't match the intended direction, it wrapped around ±180°
    // Correct the sign to match the requested turn direction
    float total_turned_deg;
    if (turning_left && total_turned_rad < 0) {
        // Left turn but got negative angle - it wrapped around
        total_turned_deg = rad_to_deg(total_turned_rad + 2.0f * PI);
    } else if (!turning_left && total_turned_rad > 0) {
        // Right turn but got positive angle - it wrapped around
        total_turned_deg = rad_to_deg(total_turned_rad - 2.0f * PI);
    } else {
        // Sign is correct
        total_turned_deg = rad_to_deg(total_turned_rad);
    }
    
    Serial.println("======================================");
    Serial.println("TURN COMPLETE");
    Serial.print("Requested:   ");
    Serial.print(target_angle_deg, 2);
    Serial.println("°");
    Serial.print("Actually turned: ");
    Serial.print(total_turned_deg, 2);
    Serial.println("°");
    Serial.print("Final yaw:   ");
    Serial.print(rad_to_deg(rad_yaw), 2);
    Serial.println("°");
    Serial.print("Final error: ");
    Serial.print(final_error_deg, 2);
    Serial.println("°");
    Serial.print("Time taken:  ");
    Serial.print(millis() - start_time);
    Serial.println(" ms");
    Serial.println("======================================");
    
    // Return success if within tolerance
    return (final_error_deg < TURN_TOLERANCE_DEG * 2.0f);
}

/**
 * @brief Convenience function for 90-degree right turn
 */
bool turn_right_90() {
    return discrete_turn(-90.0f);
}

/**
 * @brief Convenience function for 90-degree left turn
 */
bool turn_left_90() {
    return discrete_turn(90.0f);
}

/**
 * @brief Convenience function for 45-degree right turn
 */
bool turn_right_45() {
    return discrete_turn(-45.0f);
}

/**
 * @brief Convenience function for 45-degree left turn
 */
bool turn_left_45() {
    return discrete_turn(45.0f);
}

/**
 * @brief Convenience function for 5-degree right turn
 */
bool turn_right_5() {
    return discrete_turn(-5.0f);
}

/**
 * @brief Convenience function for 5-degree left turn
 */
bool turn_left_5() {
    return discrete_turn(5.0f);
}

// ========================================
// INTEGRATED MOTOR CONTROL WITH CENTERING
// ========================================

/**
 * @brief Simple motor loop with wall centering control (no PID)
 * Pure open-loop proportional control for wall following
 */
void motor_loop_with_centering() {
    // Always apply centering control when moving forward
    if (flag_forward_ == true) {
        // Robot is moving forward - apply simple proportional centering control
        float centering_correction = kp_centering_control();
        
        int adjusted_left, adjusted_right;
        apply_centering_correction(centering_correction,
                                   adjusted_left, adjusted_right);
        
        // Send PWM directly to motors with corrected speeds (no PID)
        sendPWM(adjusted_left, adjusted_right);
        
        // Debug output
        static unsigned long last_debug = 0;
        if (millis() - last_debug > 200) {
            Serial.print("Centering - L_PWM: ");
            Serial.print(adjusted_left);
            Serial.print(" | R_PWM: ");
            Serial.print(adjusted_right);
            Serial.print(" | Corr: ");
            Serial.print(centering_correction, 1);
            Serial.print(" | L_dist: ");
            Serial.print(getToFDistance(SENSOR_LEFT_SIDE));
            Serial.print("mm | R_dist: ");
            Serial.print(getToFDistance(SENSOR_RIGHT_SIDE));
            Serial.println("mm");
            last_debug = millis();
        }
    }
}
