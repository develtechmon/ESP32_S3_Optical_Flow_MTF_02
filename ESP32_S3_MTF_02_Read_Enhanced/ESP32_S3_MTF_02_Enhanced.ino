// Enhanced MTF-02 Optical Flow & Distance Sensor Library for Flight Controller
// Version 2.0 - With calibration, position integration, and improved velocity calculation
// Usage: Call mtf02_init() in setup(), mtf02_update() in main loop

#ifndef MTF02_LIBRARY_H
#define MTF02_LIBRARY_H

#include <HardwareSerial.h>

// MTF-02 Hardware Configuration
#define MTF02_RX_PIN 1
#define MTF02_TX_PIN 3
#define MTF02_BAUD 115200

// Configurable Parameters
#define MTF02_FLOW_MIN_QUALITY 50      // Minimum quality threshold
#define MTF02_MIN_DISTANCE_CM 8         // Minimum valid distance
#define MTF02_MAX_DISTANCE_CM 300       // Maximum valid distance
#define MTF02_TIMEOUT_MS 1000           // Data validity timeout
#define MTF02_FLOW_SCALE_FACTOR 10.0    // Raw to pixel conversion factor
#define MTF02_MAX_BYTES_PER_UPDATE 64   // Max bytes to process per update (non-blocking)

// Sensor specifications
#define MTF02_FOV_RAD 0.733             // 42 degrees in radians
#define MTF02_SENSOR_FPS 50.0           // MTF-02 output rate (50Hz)
#define MTF02_SENSOR_RESOLUTION 35.0    // PMW3901 sensor resolution (35x35 pixels)

// Sensor orientation enum
enum MTF02_Orientation {
  MTF02_ORIENT_FORWARD,
  MTF02_ORIENT_BACKWARD,
  MTF02_ORIENT_LEFT,
  MTF02_ORIENT_RIGHT
};

// Calibration structure
struct MTF02_Calibration {
  float flow_scale_x;
  float flow_scale_y;
  int16_t flow_offset_x;
  int16_t flow_offset_y;
};

// Position tracking structure
struct MTF02_Position {
  float x;                    // Position X in meters
  float y;                    // Position Y in meters
  unsigned long last_update;  // Last position update timestamp
};

// Statistics structure
struct MTF02_Stats {
  uint32_t flow_packets_received;
  uint32_t distance_packets_received;
  uint32_t crc_errors;
  uint32_t parse_errors;
  float avg_flow_quality;
  uint32_t quality_samples;
};

// Main sensor data structure
struct MTF02_Data {
  // Optical Flow (raw values from sensor)
  int16_t flow_x_raw;         // Raw flow X (-32768 to 32767)
  int16_t flow_y_raw;         // Raw flow Y (-32768 to 32767)
  
  // Optical Flow (converted to pixels/sec)
  float flow_x_pixels;        // Flow X in pixels (calibrated)
  float flow_y_pixels;        // Flow Y in pixels (calibrated)
  
  // Velocity estimation
  float velocity_x;           // Velocity X in m/s
  float velocity_y;           // Velocity Y in m/s
  
  // Distance sensor
  uint16_t distance_cm;       // Distance in centimeters
  
  // Quality indicators
  uint8_t flow_quality;       // Flow confidence 0-255 (>100 = good)
  uint8_t sensor_id;          // Sensor ID (usually 0)
  
  // Status flags
  bool flow_data_valid;       // True if recent flow data received
  bool distance_data_valid;   // True if recent distance data received
  
  // Timestamps
  unsigned long last_flow_update;
  unsigned long last_distance_update;
};

// Global sensor data and configuration
MTF02_Data mtf02;
MTF02_Calibration mtf02_cal = {1.0, 1.0, 0, 0};
MTF02_Position mtf02_pos = {0, 0, 0};
MTF02_Stats mtf02_stats = {0, 0, 0, 0, 0.0, 0};
MTF02_Orientation mtf02_orientation = MTF02_ORIENT_FORWARD;

// Private variables
HardwareSerial mtf02_serial(1);

// Function declarations
void mtf02_init();
void mtf02_update();
bool mtf02_flow_ready();
void mtf02_get_velocity(float height_m, float* vel_x, float* vel_y);
float mtf02_get_height();
void mtf02_update_position();
void mtf02_reset_position();
void mtf02_set_calibration(float scale_x, float scale_y, int16_t offset_x, int16_t offset_y);
void mtf02_set_orientation(MTF02_Orientation orient);
void mtf02_print_debug();
void mtf02_print_stats();

// Private function declarations
void mtf02_crc_add(uint8_t d, uint16_t* crc);
void mtf02_parse_message(uint8_t id, uint8_t* p);
void mtf02_apply_orientation(float* x, float* y);
void mtf02_apply_calibration();

// Initialize MTF-02 sensor
void mtf02_init() {
  mtf02_serial.begin(MTF02_BAUD, SERIAL_8N1, MTF02_RX_PIN, MTF02_TX_PIN);
  
  // Initialize data structure
  mtf02.flow_x_raw = 0;
  mtf02.flow_y_raw = 0;
  mtf02.flow_x_pixels = 0.0;
  mtf02.flow_y_pixels = 0.0;
  mtf02.velocity_x = 0.0;
  mtf02.velocity_y = 0.0;
  mtf02.distance_cm = 0;
  mtf02.flow_quality = 0;
  mtf02.sensor_id = 0;
  mtf02.flow_data_valid = false;
  mtf02.distance_data_valid = false;
  mtf02.last_flow_update = 0;
  mtf02.last_distance_update = 0;
  
  // Reset position and stats
  mtf02_reset_position();
  memset(&mtf02_stats, 0, sizeof(mtf02_stats));
}

// Update sensor data - call this in your main loop
void mtf02_update() {
  static uint8_t state = 0, len, msgid, idx;
  static uint8_t buf[32];
  static uint16_t crc;
  static uint8_t ck_a;
  
  int bytes_processed = 0;
  
  // Non-blocking serial read
  while (mtf02_serial.available() && bytes_processed < MTF02_MAX_BYTES_PER_UPDATE) {
    uint8_t c = mtf02_serial.read();
    bytes_processed++;
    
    switch (state) {
      case 0: if (c == 0xFE) { crc = 0xFFFF; state = 1; } break;
      case 1: len = c; mtf02_crc_add(c, &crc); state = 2; break;
      case 2: mtf02_crc_add(c, &crc); state = 3; break;
      case 3: mtf02_crc_add(c, &crc); if (c != 200) { state = 0; mtf02_stats.parse_errors++; } else state = 4; break;
      case 4: mtf02_crc_add(c, &crc); state = 5; break;
      case 5: msgid = c; mtf02_crc_add(c, &crc); idx = 0; state = 6; break;
      case 6:
        if (idx < len && idx < 32) {
          buf[idx++] = c; mtf02_crc_add(c, &crc);
          if (idx == len) {
            if (msgid == 100) mtf02_crc_add(175, &crc);
            if (msgid == 132) mtf02_crc_add(85, &crc);
            state = 7;
          }
        } else { state = 0; mtf02_stats.parse_errors++; }
        break;
      case 7: ck_a = c; state = 8; break;
      case 8:
        uint16_t received = ck_a | (c << 8);
        if (received == crc) {
          mtf02_parse_message(msgid, buf);
        } else {
          mtf02_stats.crc_errors++;
        }
        state = 0;
        break;
    }
  }
  
  // Update validity flags based on timeout
  unsigned long now = millis();
  if (now - mtf02.last_flow_update > MTF02_TIMEOUT_MS) {
    mtf02.flow_data_valid = false;
  }
  if (now - mtf02.last_distance_update > MTF02_TIMEOUT_MS) {
    mtf02.distance_data_valid = false;
  }
  
  // Update velocity estimation
  if (mtf02_flow_ready()) {
    float height = mtf02_get_height();
    mtf02_get_velocity(height, &mtf02.velocity_x, &mtf02.velocity_y);
    
    // Update position integration
    mtf02_update_position();
  }
}

// Check if optical flow data is usable for position hold
bool mtf02_flow_ready() {
  return mtf02.flow_data_valid && 
         mtf02.flow_quality > MTF02_FLOW_MIN_QUALITY && 
         mtf02.distance_data_valid && 
         mtf02.distance_cm > MTF02_MIN_DISTANCE_CM && 
         mtf02.distance_cm < MTF02_MAX_DISTANCE_CM;
}

// Get velocity in m/s based on height (enhanced calculation)
void mtf02_get_velocity(float height_m, float* vel_x, float* vel_y) {
  if (mtf02_flow_ready() && height_m > 0) {
    // PMW3901 sensor has 35x35 pixel resolution
    // Angular rate = pixels * (FOV / sensor_resolution)
    // Velocity = angular_rate * height * fps
    float angular_rate_x = mtf02.flow_x_pixels * (MTF02_FOV_RAD / MTF02_SENSOR_RESOLUTION);
    float angular_rate_y = mtf02.flow_y_pixels * (MTF02_FOV_RAD / MTF02_SENSOR_RESOLUTION);
    
    *vel_x = angular_rate_x * height_m * MTF02_SENSOR_FPS;
    *vel_y = angular_rate_y * height_m * MTF02_SENSOR_FPS;
    
    // Apply orientation correction
    mtf02_apply_orientation(vel_x, vel_y);
  } else {
    *vel_x = 0.0;
    *vel_y = 0.0;
  }
}

// Get height above ground in meters
float mtf02_get_height() {
  if (mtf02.distance_data_valid) {
    return mtf02.distance_cm / 100.0;
  }
  return 0.0;
}

// Update position integration
void mtf02_update_position() {
  unsigned long now = millis();
  
  if (mtf02_pos.last_update > 0) {
    float dt = (now - mtf02_pos.last_update) / 1000.0;
    
    if (dt > 0 && dt < 0.5) { // Sanity check
      mtf02_pos.x += mtf02.velocity_x * dt;
      mtf02_pos.y += mtf02.velocity_y * dt;
    }
  }
  
  mtf02_pos.last_update = now;
}

// Reset position to origin
void mtf02_reset_position() {
  mtf02_pos.x = 0.0;
  mtf02_pos.y = 0.0;
  mtf02_pos.last_update = millis();
}

// Set calibration parameters
void mtf02_set_calibration(float scale_x, float scale_y, int16_t offset_x, int16_t offset_y) {
  mtf02_cal.flow_scale_x = scale_x;
  mtf02_cal.flow_scale_y = scale_y;
  mtf02_cal.flow_offset_x = offset_x;
  mtf02_cal.flow_offset_y = offset_y;
}

// Set sensor orientation
void mtf02_set_orientation(MTF02_Orientation orient) {
  mtf02_orientation = orient;
}

// Private: Apply orientation transformation
void mtf02_apply_orientation(float* x, float* y) {
  float temp_x = *x;
  float temp_y = *y;
  
  switch(mtf02_orientation) {
    case MTF02_ORIENT_BACKWARD:
      *x = -temp_x; *y = -temp_y; break;
    case MTF02_ORIENT_LEFT:
      *x = temp_y; *y = -temp_x; break;
    case MTF02_ORIENT_RIGHT:
      *x = -temp_y; *y = temp_x; break;
    default:
      break; // MTF02_ORIENT_FORWARD - no change
  }
}

// Private: Apply calibration to flow values
void mtf02_apply_calibration() {
  mtf02.flow_x_pixels = (mtf02.flow_x_raw - mtf02_cal.flow_offset_x) * mtf02_cal.flow_scale_x / MTF02_FLOW_SCALE_FACTOR;
  mtf02.flow_y_pixels = (mtf02.flow_y_raw - mtf02_cal.flow_offset_y) * mtf02_cal.flow_scale_y / MTF02_FLOW_SCALE_FACTOR;
}

// Private: CRC calculation
void mtf02_crc_add(uint8_t d, uint16_t* crc) {
  uint8_t tmp = d ^ (*crc & 0xFF);
  tmp ^= tmp << 4;
  *crc = (*crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

// Private: Parse MAVLink messages
void mtf02_parse_message(uint8_t id, uint8_t* p) {
  if (id == 100) { // Optical flow
    mtf02.flow_x_raw = p[20] | (p[21] << 8);
    mtf02.flow_y_raw = p[22] | (p[23] << 8);
    mtf02.flow_quality = p[25];
    mtf02.sensor_id = p[24];
    
    // Apply calibration
    mtf02_apply_calibration();
    
    mtf02.flow_data_valid = true;
    mtf02.last_flow_update = millis();
    
    // Update statistics
    mtf02_stats.flow_packets_received++;
    mtf02_stats.avg_flow_quality = 
      (mtf02_stats.avg_flow_quality * mtf02_stats.quality_samples + mtf02.flow_quality) / 
      (mtf02_stats.quality_samples + 1);
    mtf02_stats.quality_samples++;
    
  } else if (id == 132) { // Distance sensor
    mtf02.distance_cm = p[8] | (p[9] << 8);
    mtf02.distance_data_valid = true;
    mtf02.last_distance_update = millis();
    mtf02_stats.distance_packets_received++;
  }
}

// Debug printing function
void mtf02_print_debug() {
  Serial.print("MTF02 - FlowX: "); Serial.print(mtf02.flow_x_pixels, 1);
  Serial.print(" FlowY: "); Serial.print(mtf02.flow_y_pixels, 1);
  Serial.print(" VelX: "); Serial.print(mtf02.velocity_x, 2);
  Serial.print(" VelY: "); Serial.print(mtf02.velocity_y, 2);
  Serial.print(" Quality: "); Serial.print(mtf02.flow_quality);
  Serial.print(" Distance: "); Serial.print(mtf02.distance_cm);
  Serial.print("cm PosX: "); Serial.print(mtf02_pos.x, 2);
  Serial.print(" PosY: "); Serial.print(mtf02_pos.y, 2);
  Serial.print(" Ready: "); Serial.println(mtf02_flow_ready() ? "YES" : "NO");
}

// Statistics printing function
void mtf02_print_stats() {
  Serial.println("=== MTF02 Statistics ===");
  Serial.print("Flow packets: "); Serial.println(mtf02_stats.flow_packets_received);
  Serial.print("Distance packets: "); Serial.println(mtf02_stats.distance_packets_received);
  Serial.print("CRC errors: "); Serial.println(mtf02_stats.crc_errors);
  Serial.print("Parse errors: "); Serial.println(mtf02_stats.parse_errors);
  Serial.print("Avg flow quality: "); Serial.println(mtf02_stats.avg_flow_quality, 1);
  Serial.println("=======================");
}

#endif // MTF02_LIBRARY_H

/*
// ===============================================
// EXAMPLE USAGE IN YOUR FLIGHT CONTROLLER:
// ===============================================
*/

// Position hold PID controllers (example)
float pid_x_setpoint = 0, pid_y_setpoint = 0;
float pid_x_kp = 0.5, pid_x_ki = 0.1, pid_x_kd = 0.2;
float pid_y_kp = 0.5, pid_y_ki = 0.1, pid_y_kd = 0.2;
float pid_x_integral = 0, pid_y_integral = 0;
float pid_x_last_error = 0, pid_y_last_error = 0;

void setup() {
  Serial.begin(115200);
  mtf02_init();  // Initialize MTF-02 sensor
  
  // Optional: Set custom calibration
  // mtf02_set_calibration(1.1, 0.95, 10, -5);
  
  // Optional: Set sensor orientation if not forward-facing
  // mtf02_set_orientation(MTF02_ORIENT_BACKWARD);
  
  Serial.println("Enhanced MTF-02 Flight Controller Library Ready");
}

void loop() {
  mtf02_update();  // Update sensor data
  
  // Position hold mode example
  if (mtf02_flow_ready()) {

    mtf02_print_debug();
    // Simple position hold using velocity feedback
    float error_x = pid_x_setpoint - mtf02.velocity_x;
    float error_y = pid_y_setpoint - mtf02.velocity_y;
    
    // PID calculations (simplified)
    pid_x_integral += error_x * 0.01; // Assuming 100Hz loop
    pid_y_integral += error_y * 0.01;
    
    float pid_x_derivative = (error_x - pid_x_last_error) / 0.01;
    float pid_y_derivative = (error_y - pid_y_last_error) / 0.01;
    
    float roll_correction = pid_x_kp * error_x + pid_x_ki * pid_x_integral + pid_x_kd * pid_x_derivative;
    float pitch_correction = pid_y_kp * error_y + pid_y_ki * pid_y_integral + pid_y_kd * pid_y_derivative;
    
    // Apply corrections to your flight controller
    // apply_roll_pitch_corrections(roll_correction, pitch_correction);
    
    pid_x_last_error = error_x;
    pid_y_last_error = error_y;
    
    // Alternative: Use position feedback for absolute position hold
    // float pos_error_x = target_x - mtf02_pos.x;
    // float pos_error_y = target_y - mtf02_pos.y;
  }
  
  // Debug output
  static unsigned long last_debug = 0;
  if (millis() - last_debug > 100) {
    mtf02_print_debug();
    last_debug = millis();
  }
  
  // Print statistics every 10 seconds
  static unsigned long last_stats = 0;
  if (millis() - last_stats > 10000) {
    mtf02_print_stats();
    last_stats = millis();
  }
  
  // Reset position on user command (example)
  if (Serial.available() && Serial.read() == 'r') {
    mtf02_reset_position();
    Serial.println("Position reset to origin");
  }
}

/*
// ===============================================
// ADVANCED INTEGRATION TIPS:
// ===============================================

1. Sensor Fusion with IMU:
   - Combine optical flow velocity with accelerometer data
   - Use complementary filter: vel_fused = 0.9 * vel_flow + 0.1 * vel_imu
   
2. Surface Detection:
   - Monitor flow quality over time
   - If quality drops below threshold, switch to alternative navigation
   
3. Altitude Compensation:
   - Use barometer for absolute altitude when above optical flow range
   - Smooth transition between sensors
   
4. Vibration Filtering:
   - Add low-pass filter to flow values if needed
   - flow_filtered = 0.7 * flow_filtered + 0.3 * flow_new
   
5. Calibration Procedure:
   - Fly known distance patterns
   - Compare integrated position with GPS or known distance
   - Adjust calibration scales accordingly
   
6. Multi-Sensor Setup:
   - Use multiple MTF-02 sensors for redundancy
   - Average readings or use voting system
*/
