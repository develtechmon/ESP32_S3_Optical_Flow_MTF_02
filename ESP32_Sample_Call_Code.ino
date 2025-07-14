#include "MTF02_Library_M.h"

// Your existing flight controller code here

void setup() {
    Serial.begin(115200);
    
    // Add these two lines to your existing setup:
    mtf02_init();                    // Initialize MTF-02
    Serial.println("MTF-02 Ready");
    
    // Your existing setup code...
}

void loop() {
    // Add this as the FIRST line in your loop:
    mtf02_update();                  

    // Your existing flight controller code...
    
    // Example usage for altitude hold:
    //if (mtf02_is_ready_for_altitude_hold()) {
    float current_altitude = mtf02_get_altitude_m() * 100; // Convert metre to cm if you're using MTF02_Librarry_M.h
    // Use current_altitude in your altitude hold code
    //}
    
    float pos_y = mtf02_get_position_y(); 
    float pos_x = mtf02_get_position_x();

    // Example usage for position hold:
    //if (mtf02_is_ready_for_position_hold()) {
    float vel_x = mtf02_get_velocity_x();
    float vel_y = mtf02_get_velocity_y();
    // Use velocities in your position hold code
    //}

    Serial.print("Distance: ");
    Serial.print(current_altitude);
    Serial.print(" - ");

    Serial.print("Pos X: ");
    Serial.print(pos_x);
    Serial.print(" - ");

    Serial.print("Pos Y: ");
    Serial.print(pos_x);
    Serial.print(" - ");

    Serial.print("Vel X: ");
    Serial.print(vel_x);
    Serial.print(" - ");

    Serial.print("Vel Y: ");
    Serial.println(vel_y);
}
