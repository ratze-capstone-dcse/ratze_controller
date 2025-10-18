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
#define TURN_SPEED 0.5f              // Angular velocity for turning (rad/s)

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
 * @brief Perform a discrete turn using encoder and IMU fusion
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
    
    // Record initial state
    float initial_yaw = rad_yaw;
    long initial_encoder_left = countM2_;  // Left motors (M2, M4)
    long initial_encoder_right = countM1_; // Right motors (M1, M3)
    
    // Calculate target heading
    float target_yaw = normalize_angle(initial_yaw + target_angle_rad);
    
    // Calculate expected encoder change
    long expected_ticks = angle_to_ticks(target_angle_rad);
    
    Serial.print("Initial yaw: ");
    Serial.print(rad_to_deg(initial_yaw), 2);
    Serial.print(" deg | Target yaw: ");
    Serial.print(rad_to_deg(target_yaw), 2);
    Serial.println(" deg");
    Serial.print("Expected encoder ticks: ");
    Serial.println(expected_ticks);
    
    // Stop any current motion and reset PIDs
    cmd_vel_.x = 0.0;
    cmd_vel_.w = 0.0;
    pid_right.reset();
    pid_left.reset();
    delay(50);
    
    // Set turn command based on direction
    if (target_angle_rad > 0) {
        // Turn left (counter-clockwise)
        cmd_vel_.x = 0.0;
        cmd_vel_.w = TURN_SPEED;
    } else {
        // Turn right (clockwise)
        cmd_vel_.x = 0.0;
        cmd_vel_.w = -TURN_SPEED;
    }
    
    unsigned long start_time = millis();
    bool turn_complete = false;
    
    // Main turning loop
    while (!turn_complete && (millis() - start_time < TURN_TIMEOUT_MS)) {
        // Update IMU data
        update_imu();
        
        // Calculate current error
        float current_error = normalize_angle(target_yaw - rad_yaw);
        float error_deg = rad_to_deg(fabs(current_error));
        
        // Get encoder progress
        long encoder_left_change = abs(countM2_ - initial_encoder_left);
        long encoder_right_change = abs(countM1_ - initial_encoder_right);
        long avg_encoder_change = (encoder_left_change + encoder_right_change) / 2;
        
        // Calculate completion percentage using weighted fusion
        float imu_progress = 1.0f - (fabs(current_error) / fabs(target_angle_rad));
        float encoder_progress = (float)avg_encoder_change / (float)expected_ticks;
        
        // Fuse IMU and encoder data (70% IMU, 30% encoder for better accuracy)
        float fused_progress = 0.7f * imu_progress + 0.3f * encoder_progress;
        bound_value(fused_progress, 0.0f, 1.0f);
        
        // Slow down as we approach target
        if (fused_progress > 0.8f) {
            // Reduce speed to 40% for final approach
            float reduced_speed = TURN_SPEED * 0.4f;
            cmd_vel_.w = (target_angle_rad > 0) ? reduced_speed : -reduced_speed;
        }
        
        // Check if turn is complete
        if (error_deg < TURN_TOLERANCE_DEG) {
            turn_complete = true;
        }
        
        // Execute motor control
        motor_loop();
        
        // Debug output every 100ms
        static unsigned long last_debug = 0;
        if (millis() - last_debug > 100) {
            Serial.print("Error: ");
            Serial.print(error_deg, 2);
            Serial.print(" deg | Progress: ");
            Serial.print(fused_progress * 100.0f, 1);
            Serial.print("% | Encoders: ");
            Serial.print(avg_encoder_change);
            Serial.print("/");
            Serial.println(expected_ticks);
            last_debug = millis();
        }
        
        delay(10);
    }
    
    // Stop turning
    cmd_vel_.x = 0.0;
    cmd_vel_.w = 0.0;
    pid_right.reset();
    pid_left.reset();
    moveStop();
    
    // Final error check
    float final_error = normalize_angle(target_yaw - rad_yaw);
    float final_error_deg = rad_to_deg(fabs(final_error));
    
    Serial.println("======================================");
    Serial.print("Turn complete. Final error: ");
    Serial.print(final_error_deg, 2);
    Serial.println(" degrees");
    Serial.print("Time taken: ");
    Serial.print(millis() - start_time);
    Serial.println(" ms");
    Serial.println("======================================");
    
    // Return success if within tolerance
    return (final_error_deg < TURN_TOLERANCE_DEG);
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
