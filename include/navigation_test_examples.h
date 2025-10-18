/**
 * @file navigation_test_examples.cpp
 * @brief Example usage patterns for the navigation control system
 * 
 * This file contains example code demonstrating how to use the
 * wall centering and discrete turning features.
 */

#include <Arduino.h>
#include "firmware_control.h"
#include "navigation_control.h"

// ========================================
// EXAMPLE 1: Basic Wall Following
// ========================================
void example_wall_following() {
    Serial.println("Example 1: Basic Wall Following");
    
    // Enable centering
    centering_enabled = true;
    
    // Move forward at 0.5 m/s for 3 seconds
    // Robot will automatically center between walls
    cmd_vel_.x = 0.5;
    cmd_vel_.w = 0.0;
    
    unsigned long start = millis();
    while (millis() - start < 3000) {
        motor_loop_with_centering();
        delay(10);
    }
    
    // Stop
    cmd_vel_.x = 0.0;
    cmd_vel_.w = 0.0;
    moveStop();
    
    Serial.println("Wall following complete");
}

// ========================================
// EXAMPLE 2: 90-Degree Turn Sequence
// ========================================
void example_turn_sequence() {
    Serial.println("Example 2: Turn Sequence");
    
    // Turn left 90 degrees
    if (turn_left_90()) {
        Serial.println("Left turn successful");
    } else {
        Serial.println("Left turn failed!");
        return;
    }
    
    delay(500);
    
    // Turn right 90 degrees
    if (turn_right_90()) {
        Serial.println("Right turn successful");
    } else {
        Serial.println("Right turn failed!");
        return;
    }
    
    Serial.println("Turn sequence complete");
}

// ========================================
// EXAMPLE 3: Simple Maze Navigation
// ========================================
void example_maze_navigation() {
    Serial.println("Example 3: Maze Navigation");
    
    centering_enabled = true;
    
    // Move forward for 1 meter (approximately)
    cmd_vel_.x = 0.3;  // 30 cm/s
    cmd_vel_.w = 0.0;
    
    unsigned long start = millis();
    while (millis() - start < 3000) {  // 3 seconds ≈ 0.9 meters
        motor_loop_with_centering();
        
        // Check front sensor for wall
        uint16_t front_distance = getToFDistance(3);  // Assuming sensor 3 is front
        if (front_distance < 100) {  // Less than 10cm
            Serial.println("Wall detected! Stopping.");
            break;
        }
        
        delay(10);
    }
    
    // Stop
    cmd_vel_.x = 0.0;
    cmd_vel_.w = 0.0;
    moveStop();
    delay(200);
    
    // Turn left
    if (turn_left_90()) {
        Serial.println("Turned left at intersection");
    }
    
    // Continue forward
    cmd_vel_.x = 0.3;
    cmd_vel_.w = 0.0;
    start = millis();
    while (millis() - start < 2000) {
        motor_loop_with_centering();
        delay(10);
    }
    
    // Stop
    cmd_vel_.x = 0.0;
    cmd_vel_.w = 0.0;
    moveStop();
    
    Serial.println("Maze navigation complete");
}

// ========================================
// EXAMPLE 4: Custom Angle Turn
// ========================================
void example_custom_turn() {
    Serial.println("Example 4: Custom Angle Turn");
    
    // Turn 45 degrees left
    if (discrete_turn(45.0)) {
        Serial.println("45° turn successful");
    }
    
    delay(500);
    
    // Turn 30 degrees right
    if (discrete_turn(-30.0)) {
        Serial.println("30° turn successful");
    }
    
    delay(500);
    
    // Fine correction: 5 degrees left
    if (turn_left_5()) {
        Serial.println("5° correction successful");
    }
    
    Serial.println("Custom angle turns complete");
}

// ========================================
// EXAMPLE 5: Wall Centering Test
// ========================================
void example_centering_test() {
    Serial.println("Example 5: Wall Centering Test");
    
    centering_enabled = true;
    
    Serial.println("Starting forward movement with centering...");
    
    // Move forward and monitor centering behavior
    cmd_vel_.x = 0.4;
    cmd_vel_.w = 0.0;
    
    unsigned long start = millis();
    while (millis() - start < 5000) {  // 5 seconds
        motor_loop_with_centering();
        
        // Print sensor readings every 500ms
        static unsigned long last_print = 0;
        if (millis() - last_print > 500) {
            uint16_t left = getToFDistance(SENSOR_LEFT_SIDE);
            uint16_t right = getToFDistance(SENSOR_RIGHT_SIDE);
            float correction = kp_centering_control();
            
            Serial.print("L: ");
            Serial.print(left);
            Serial.print("mm | R: ");
            Serial.print(right);
            Serial.print("mm | Error: ");
            Serial.print(left - right);
            Serial.print("mm | Correction: ");
            Serial.println(correction, 1);
            
            last_print = millis();
        }
        
        delay(10);
    }
    
    // Stop
    cmd_vel_.x = 0.0;
    cmd_vel_.w = 0.0;
    moveStop();
    
    Serial.println("Centering test complete");
}

// ========================================
// EXAMPLE 6: Square Path Navigation
// ========================================
void example_square_path() {
    Serial.println("Example 6: Square Path (4 sides + 4 turns)");
    
    centering_enabled = true;
    
    for (int i = 0; i < 4; i++) {
        Serial.print("Side ");
        Serial.println(i + 1);
        
        // Move forward for 1.5 seconds
        cmd_vel_.x = 0.3;
        cmd_vel_.w = 0.0;
        
        unsigned long start = millis();
        while (millis() - start < 1500) {
            motor_loop_with_centering();
            delay(10);
        }
        
        // Stop
        cmd_vel_.x = 0.0;
        cmd_vel_.w = 0.0;
        moveStop();
        delay(200);
        
        // Turn right 90 degrees
        if (!turn_right_90()) {
            Serial.println("Turn failed! Aborting.");
            return;
        }
        
        delay(200);
    }
    
    Serial.println("Square path complete!");
}

// ========================================
// EXAMPLE 7: Dead Reckoning Position Tracking
// ========================================
struct Position {
    float x;  // meters
    float y;  // meters
    float theta;  // radians
};

Position robot_position = {0.0, 0.0, 0.0};

void update_position() {
    // Simple dead reckoning using encoders and IMU
    static long last_count_left = 0;
    static long last_count_right = 0;
    
    long delta_left = countM2_ - last_count_left;
    long delta_right = countM1_ - last_count_right;
    
    float dist_left = ticks_to_distance(delta_left);
    float dist_right = ticks_to_distance(delta_right);
    
    // Average distance traveled
    float dist_center = (dist_left + dist_right) / 2.0f;
    
    // Update position using current heading
    robot_position.x += dist_center * cos(rad_yaw);
    robot_position.y += dist_center * sin(rad_yaw);
    robot_position.theta = rad_yaw;
    
    last_count_left = countM2_;
    last_count_right = countM1_;
}

void example_position_tracking() {
    Serial.println("Example 7: Position Tracking");
    
    // Reset position
    robot_position = {0.0, 0.0, 0.0};
    resetEncoders();
    
    centering_enabled = true;
    
    // Move forward
    cmd_vel_.x = 0.3;
    cmd_vel_.w = 0.0;
    
    unsigned long start = millis();
    while (millis() - start < 2000) {
        motor_loop_with_centering();
        update_position();
        delay(10);
    }
    
    Serial.print("Position after forward: X=");
    Serial.print(robot_position.x, 3);
    Serial.print("m, Y=");
    Serial.print(robot_position.y, 3);
    Serial.println("m");
    
    // Stop and turn
    cmd_vel_.x = 0.0;
    cmd_vel_.w = 0.0;
    moveStop();
    delay(200);
    
    turn_right_90();
    update_position();
    
    Serial.print("Heading after turn: ");
    Serial.print(rad_to_deg(robot_position.theta), 1);
    Serial.println(" degrees");
    
    Serial.println("Position tracking complete");
}

// ========================================
// EXAMPLE 8: Emergency Stop on Front Wall
// ========================================
void example_emergency_stop() {
    Serial.println("Example 8: Emergency Stop");
    
    centering_enabled = true;
    
    cmd_vel_.x = 0.3;
    cmd_vel_.w = 0.0;
    
    Serial.println("Moving forward. Will stop if front wall < 80mm");
    
    while (true) {
        motor_loop_with_centering();
        
        // Check front sensor (assuming sensor 3)
        uint16_t front = getToFDistance(3);
        
        if (front < 80 && front > 0) {
            Serial.println("EMERGENCY STOP!");
            cmd_vel_.x = 0.0;
            cmd_vel_.w = 0.0;
            moveStop();
            
            Serial.print("Stopped at distance: ");
            Serial.print(front);
            Serial.println("mm");
            break;
        }
        
        delay(10);
    }
    
    Serial.println("Emergency stop test complete");
}

// ========================================
// EXAMPLE 9: Sensor Validation Test
// ========================================
void example_sensor_validation() {
    Serial.println("Example 9: Sensor Validation");
    
    Serial.println("Reading all sensors for 5 seconds...");
    
    unsigned long start = millis();
    while (millis() - start < 5000) {
        updateToFReadings();
        update_imu();
        
        static unsigned long last_print = 0;
        if (millis() - last_print > 500) {
            // Print ToF sensors
            Serial.print("ToF: ");
            for (int i = 0; i < NUM_SENSORS; i++) {
                Serial.print(tof_distances[i]);
                Serial.print("mm ");
            }
            Serial.println();
            
            // Print IMU
            Serial.print("IMU: Yaw=");
            Serial.print(rad_to_deg(rad_yaw), 1);
            Serial.print("° Roll=");
            Serial.print(rad_to_deg(rad_roll), 1);
            Serial.print("° Pitch=");
            Serial.print(rad_to_deg(rad_pitch), 1);
            Serial.println("°");
            
            // Print encoders
            Serial.print("Enc: M1=");
            Serial.print(countM1_);
            Serial.print(" M2=");
            Serial.print(countM2_);
            Serial.print(" M3=");
            Serial.print(countM3_);
            Serial.print(" M4=");
            Serial.println(countM4_);
            
            Serial.println("---");
            last_print = millis();
        }
        
        delay(10);
    }
    
    Serial.println("Sensor validation complete");
}

// ========================================
// MAIN DEMO FUNCTION
// ========================================
void run_navigation_demo() {
    Serial.println("\n========================================");
    Serial.println("NAVIGATION CONTROL DEMO");
    Serial.println("========================================\n");
    
    delay(1000);
    
    // Uncomment the examples you want to run:
    
    // example_wall_following();
    // delay(2000);
    
    // example_turn_sequence();
    // delay(2000);
    
    // example_maze_navigation();
    // delay(2000);
    
    // example_custom_turn();
    // delay(2000);
    
    // example_centering_test();
    // delay(2000);
    
    // example_square_path();
    // delay(2000);
    
    // example_position_tracking();
    // delay(2000);
    
    // example_emergency_stop();
    // delay(2000);
    
    example_sensor_validation();
    delay(2000);
    
    Serial.println("\n========================================");
    Serial.println("DEMO COMPLETE");
    Serial.println("========================================\n");
}
