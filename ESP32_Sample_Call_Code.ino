#include "MTF02_Library.h"

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
    mtf02_update();                  // Update MTF-02 data
    
    // Your existing flight controller code...
    
    // Example usage for altitude hold:
    if (mtf02_is_ready_for_altitude_hold()) {
        float current_altitude = mtf02_get_altitude_m();
        // Use current_altitude in your altitude hold code
    }
    
    // Example usage for position hold:
    if (mtf02_is_ready_for_position_hold()) {
        float vel_x = mtf02_get_velocity_x();
        float vel_y = mtf02_get_velocity_y();
        // Use velocities in your position hold code
    }
}
