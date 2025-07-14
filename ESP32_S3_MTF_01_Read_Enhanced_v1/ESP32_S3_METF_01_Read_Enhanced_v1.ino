// Enhanced MTF-02 Optical Flow & Distance Sensor Library
// Optimized for 50Hz operation with clean function-based API
// Author: Flight Controller Integration Version

#ifndef MTF02_ENHANCED_H
#define MTF02_ENHANCED_H

#include <HardwareSerial.h>

// Hardware Configuration
#define MTF02_RX_PIN 1
#define MTF02_TX_PIN 3
#define MTF02_BAUD 115200
#define MTF02_MAVLINK_SYSID 200

// Performance Configuration for 50Hz
#define MTF02_MAX_BYTES_PER_UPDATE 150  // Increased for 50Hz (was 64)
#define MTF02_PACKET_SIZE_ESTIMATE 35   // MAVLink packet size
#define MTF02_EXPECTED_RATE_HZ 50       // Sensor update rate

// Quality and Range Parameters
#define MTF02_FLOW_MIN_QUALITY 50
#define MTF02_MIN_DISTANCE_CM 8
#define MTF02_MAX_DISTANCE_CM 300
#define MTF02_TIMEOUT_MS 100            // Reduced for 50Hz (was 1000)

// Sensor Specifications
#define MTF02_FOV_RAD 0.733             // 42 degrees
#define MTF02_SENSOR_RESOLUTION 35.0    // PMW3901 35x35 pixels
#define MTF02_FLOW_SCALE_FACTOR 10.0

// Orientation options
enum MTF02_Orientation {
    MTF02_ORIENT_FORWARD = 0,
    MTF02_ORIENT_BACKWARD,
    MTF02_ORIENT_LEFT,
    MTF02_ORIENT_RIGHT
};

// Internal data structures
struct MTF02_RawData {
    // Raw sensor values
    int16_t flow_x_raw;
    int16_t flow_y_raw;
    uint16_t distance_cm;
    uint8_t flow_quality;
    uint8_t sensor_id;
    
    // Processed values
    float flow_x_pixels;
    float flow_y_pixels;
    
    // Timestamps and validity
    uint32_t last_flow_update;
    uint32_t last_distance_update;
    bool flow_valid;
    bool distance_valid;
};

struct MTF02_Navigation {
    // Velocity in m/s
    float velocity_x;
    float velocity_y;
    float velocity_z;
    
    // Position in meters (integrated)
    float position_x;
    float position_y;
    float position_z;
    
    // Last update timestamp
    uint32_t last_nav_update;
    bool nav_valid;
};

struct MTF02_Calibration {
    float flow_scale_x;
    float flow_scale_y;
    int16_t flow_offset_x;
    int16_t flow_offset_y;
};

struct MTF02_Performance {
    uint32_t packets_received;
    uint32_t crc_errors;
    uint32_t parse_errors;
    uint32_t timeouts;
    float actual_rate_hz;
    uint32_t last_rate_calc;
    uint32_t rate_counter;
};

// Global instances
static MTF02_RawData mtf02_raw = {0};
static MTF02_Navigation mtf02_nav = {0};
static MTF02_Calibration mtf02_cal = {1.0, 1.0, 0, 0};
static MTF02_Performance mtf02_perf = {0};
static MTF02_Orientation mtf02_orient = MTF02_ORIENT_FORWARD;
static HardwareSerial mtf02_serial(1);

// Private function declarations
void mtf02_parse_mavlink();
void mtf02_process_optical_flow(uint8_t* payload);
void mtf02_process_distance_sensor(uint8_t* payload);
void mtf02_update_navigation();
void mtf02_apply_calibration();
void mtf02_apply_orientation(float* x, float* y);
uint16_t mtf02_calculate_crc(uint8_t* data, uint8_t len);
void mtf02_update_performance_stats();

// =============================================================================
// PUBLIC API FUNCTIONS - Use these in your flight controller
// =============================================================================

// Initialize the sensor
void mtf02_init() {
    mtf02_serial.begin(MTF02_BAUD, SERIAL_8N1, MTF02_RX_PIN, MTF02_TX_PIN);
    
    // Clear all data
    memset(&mtf02_raw, 0, sizeof(mtf02_raw));
    memset(&mtf02_nav, 0, sizeof(mtf02_nav));
    memset(&mtf02_perf, 0, sizeof(mtf02_perf));
    
    // Set initial timestamps
    uint32_t now = millis();
    mtf02_nav.last_nav_update = now;
    mtf02_perf.last_rate_calc = now;
    
    Serial.println("MTF-02 Enhanced Library Initialized for 50Hz");
}

// Main update function - call this in your main loop at >100Hz for 50Hz sensor
void mtf02_update() {
    mtf02_parse_mavlink();
    mtf02_update_navigation();
    mtf02_update_performance_stats();
}

// Distance Functions
float mtf02_get_distance_m() {
    return mtf02_raw.distance_valid ? (mtf02_raw.distance_cm / 100.0f) : -1.0f;
}

uint16_t mtf02_get_distance_cm() {
    return mtf02_raw.distance_valid ? mtf02_raw.distance_cm : 0;
}

bool mtf02_is_distance_valid() {
    uint32_t now = millis();
    return mtf02_raw.distance_valid && 
           (now - mtf02_raw.last_distance_update) < MTF02_TIMEOUT_MS &&
           mtf02_raw.distance_cm >= MTF02_MIN_DISTANCE_CM &&
           mtf02_raw.distance_cm <= MTF02_MAX_DISTANCE_CM;
}

// Flow Functions
void mtf02_get_flow_pixels(float* flow_x, float* flow_y) {
    if (flow_x) *flow_x = mtf02_raw.flow_valid ? mtf02_raw.flow_x_pixels : 0.0f;
    if (flow_y) *flow_y = mtf02_raw.flow_valid ? mtf02_raw.flow_y_pixels : 0.0f;
}

void mtf02_get_flow_raw(int16_t* flow_x, int16_t* flow_y) {
    if (flow_x) *flow_x = mtf02_raw.flow_valid ? mtf02_raw.flow_x_raw : 0;
    if (flow_y) *flow_y = mtf02_raw.flow_valid ? mtf02_raw.flow_y_raw : 0;
}

uint8_t mtf02_get_flow_quality() {
    return mtf02_raw.flow_valid ? mtf02_raw.flow_quality : 0;
}

bool mtf02_is_flow_valid() {
    uint32_t now = millis();
    return mtf02_raw.flow_valid && 
           (now - mtf02_raw.last_flow_update) < MTF02_TIMEOUT_MS &&
           mtf02_raw.flow_quality >= MTF02_FLOW_MIN_QUALITY;
}

// Velocity Functions
void mtf02_get_velocity(float* vel_x, float* vel_y, float* vel_z) {
    if (vel_x) *vel_x = mtf02_nav.nav_valid ? mtf02_nav.velocity_x : 0.0f;
    if (vel_y) *vel_y = mtf02_nav.nav_valid ? mtf02_nav.velocity_y : 0.0f;
    if (vel_z) *vel_z = mtf02_nav.nav_valid ? mtf02_nav.velocity_z : 0.0f;
}

float mtf02_get_velocity_x() {
    return mtf02_nav.nav_valid ? mtf02_nav.velocity_x : 0.0f;
}

float mtf02_get_velocity_y() {
    return mtf02_nav.nav_valid ? mtf02_nav.velocity_y : 0.0f;
}

float mtf02_get_velocity_z() {
    return mtf02_nav.nav_valid ? mtf02_nav.velocity_z : 0.0f;
}

bool mtf02_is_velocity_valid() {
    return mtf02_nav.nav_valid && mtf02_is_flow_valid() && mtf02_is_distance_valid();
}

// Position Functions
void mtf02_get_position(float* pos_x, float* pos_y, float* pos_z) {
    if (pos_x) *pos_x = mtf02_nav.position_x;
    if (pos_y) *pos_y = mtf02_nav.position_y;
    if (pos_z) *pos_z = mtf02_nav.position_z;
}

float mtf02_get_position_x() {
    return mtf02_nav.position_x;
}

float mtf02_get_position_y() {
    return mtf02_nav.position_y;
}

float mtf02_get_position_z() {
    return mtf02_nav.position_z;
}

void mtf02_reset_position() {
    mtf02_nav.position_x = 0.0f;
    mtf02_nav.position_y = 0.0f;
    mtf02_nav.position_z = 0.0f;
    mtf02_nav.last_nav_update = millis();
}

// System Status Functions
bool mtf02_is_ready_for_position_hold() {
    return mtf02_is_flow_valid() && mtf02_is_distance_valid() && mtf02_is_velocity_valid();
}

float mtf02_get_update_rate() {
    return mtf02_perf.actual_rate_hz;
}

void mtf02_get_performance_stats(uint32_t* packets, uint32_t* errors, float* rate) {
    if (packets) *packets = mtf02_perf.packets_received;
    if (errors) *errors = mtf02_perf.crc_errors + mtf02_perf.parse_errors;
    if (rate) *rate = mtf02_perf.actual_rate_hz;
}

// Configuration Functions
void mtf02_set_calibration(float scale_x, float scale_y, int16_t offset_x, int16_t offset_y) {
    mtf02_cal.flow_scale_x = scale_x;
    mtf02_cal.flow_scale_y = scale_y;
    mtf02_cal.flow_offset_x = offset_x;
    mtf02_cal.flow_offset_y = offset_y;
}

void mtf02_set_orientation(MTF02_Orientation orientation) {
    mtf02_orient = orientation;
}

// Debug Functions
void mtf02_print_status() {
    Serial.printf("MTF02: Dist=%.2fm FlowQ=%d VelX=%.2f VelY=%.2f PosX=%.2f PosY=%.2f Ready=%s Rate=%.1fHz\n",
                 mtf02_get_distance_m(),
                 mtf02_get_flow_quality(),
                 mtf02_get_velocity_x(),
                 mtf02_get_velocity_y(),
                 mtf02_get_position_x(),
                 mtf02_get_position_y(),
                 mtf02_is_ready_for_position_hold() ? "YES" : "NO",
                 mtf02_get_update_rate());
}

void mtf02_print_detailed_stats() {
    Serial.println("=== MTF-02 Detailed Statistics ===");
    Serial.printf("Packets received: %lu\n", mtf02_perf.packets_received);
    Serial.printf("CRC errors: %lu\n", mtf02_perf.crc_errors);
    Serial.printf("Parse errors: %lu\n", mtf02_perf.parse_errors);
    Serial.printf("Timeouts: %lu\n", mtf02_perf.timeouts);
    Serial.printf("Actual rate: %.1f Hz (target: %d Hz)\n", mtf02_perf.actual_rate_hz, MTF02_EXPECTED_RATE_HZ);
    Serial.printf("Flow quality: %d/255\n", mtf02_get_flow_quality());
    Serial.printf("Distance: %d cm (valid: %s)\n", mtf02_get_distance_cm(), mtf02_is_distance_valid() ? "YES" : "NO");
    Serial.printf("Flow valid: %s\n", mtf02_is_flow_valid() ? "YES" : "NO");
    Serial.printf("Ready for position hold: %s\n", mtf02_is_ready_for_position_hold() ? "YES" : "NO");
    Serial.println("==================================");
}

// =============================================================================
// PRIVATE IMPLEMENTATION FUNCTIONS
// =============================================================================

void mtf02_parse_mavlink() {
    static uint8_t state = 0, len, msgid, idx;
    static uint8_t buf[64];
    static uint16_t crc;
    static uint8_t ck_a;
    
    int bytes_processed = 0;
    
    // Process more bytes per update for 50Hz operation
    while (mtf02_serial.available() && bytes_processed < MTF02_MAX_BYTES_PER_UPDATE) {
        uint8_t c = mtf02_serial.read();
        bytes_processed++;
        
        switch (state) {
            case 0: // Wait for STX
                if (c == 0xFE) { 
                    crc = 0xFFFF; 
                    state = 1; 
                }
                break;
                
            case 1: // Length
                len = c; 
                crc ^= c; 
                crc = (crc >> 8) | (crc << 8);
                crc ^= c;
                state = 2; 
                break;
                
            case 2: // Sequence
                crc ^= c; 
                crc = (crc >> 8) | (crc << 8);
                crc ^= c;
                state = 3; 
                break;
                
            case 3: // System ID
                crc ^= c; 
                crc = (crc >> 8) | (crc << 8);
                crc ^= c;
                if (c != MTF02_MAVLINK_SYSID) { 
                    state = 0; 
                    mtf02_perf.parse_errors++; 
                } else {
                    state = 4; 
                }
                break;
                
            case 4: // Component ID
                crc ^= c; 
                crc = (crc >> 8) | (crc << 8);
                crc ^= c;
                state = 5; 
                break;
                
            case 5: // Message ID
                msgid = c; 
                crc ^= c; 
                crc = (crc >> 8) | (crc << 8);
                crc ^= c;
                idx = 0; 
                state = 6; 
                break;
                
            case 6: // Payload
                if (idx < len && idx < sizeof(buf)) {
                    buf[idx++] = c; 
                    crc ^= c; 
                    crc = (crc >> 8) | (crc << 8);
                    crc ^= c;
                    if (idx == len) {
                        // Add message-specific CRC
                        if (msgid == 100) { // OPTICAL_FLOW
                            uint8_t extra = 175;
                            crc ^= extra; 
                            crc = (crc >> 8) | (crc << 8);
                            crc ^= extra;
                        }
                        if (msgid == 132) { // DISTANCE_SENSOR
                            uint8_t extra = 85;
                            crc ^= extra; 
                            crc = (crc >> 8) | (crc << 8);
                            crc ^= extra;
                        }
                        state = 7;
                    }
                } else { 
                    state = 0; 
                    mtf02_perf.parse_errors++; 
                }
                break;
                
            case 7: // Checksum low
                ck_a = c; 
                state = 8; 
                break;
                
            case 8: // Checksum high
                {
                    uint16_t received_crc = ck_a | (c << 8);
                    if (received_crc == crc) {
                        // Valid packet - process it
                        if (msgid == 100) {
                            mtf02_process_optical_flow(buf);
                        } else if (msgid == 132) {
                            mtf02_process_distance_sensor(buf);
                        }
                        mtf02_perf.packets_received++;
                    } else {
                        mtf02_perf.crc_errors++;
                    }
                    state = 0;
                }
                break;
        }
    }
}

void mtf02_process_optical_flow(uint8_t* payload) {
    // Extract data from OPTICAL_FLOW message (ID 100)
    mtf02_raw.flow_x_raw = payload[20] | (payload[21] << 8);
    mtf02_raw.flow_y_raw = payload[22] | (payload[23] << 8);
    mtf02_raw.sensor_id = payload[24];
    mtf02_raw.flow_quality = payload[25];
    
    // Apply calibration
    mtf02_apply_calibration();
    
    mtf02_raw.flow_valid = true;
    mtf02_raw.last_flow_update = millis();
}

void mtf02_process_distance_sensor(uint8_t* payload) {
    // Extract data from DISTANCE_SENSOR message (ID 132)
    mtf02_raw.distance_cm = payload[8] | (payload[9] << 8);
    mtf02_raw.distance_valid = true;
    mtf02_raw.last_distance_update = millis();
}

void mtf02_update_navigation() {
    uint32_t now = millis();
    
    // Check if we have valid data for navigation
    if (!mtf02_is_flow_valid() || !mtf02_is_distance_valid()) {
        mtf02_nav.nav_valid = false;
        return;
    }
    
    // Calculate velocity from optical flow
    float height_m = mtf02_raw.distance_cm / 100.0f;
    if (height_m > 0) {
        // Convert pixels to angular rate, then to velocity
        float angular_rate_x = mtf02_raw.flow_x_pixels * (MTF02_FOV_RAD / MTF02_SENSOR_RESOLUTION);
        float angular_rate_y = mtf02_raw.flow_y_pixels * (MTF02_FOV_RAD / MTF02_SENSOR_RESOLUTION);
        
        mtf02_nav.velocity_x = angular_rate_x * height_m * MTF02_EXPECTED_RATE_HZ;
        mtf02_nav.velocity_y = angular_rate_y * height_m * MTF02_EXPECTED_RATE_HZ;
        
        // Apply orientation correction
        mtf02_apply_orientation(&mtf02_nav.velocity_x, &mtf02_nav.velocity_y);
    }
    
    // Calculate Z velocity from distance changes
    static uint16_t prev_distance = 0;
    if (prev_distance > 0) {
        float dt = (now - mtf02_nav.last_nav_update) / 1000.0f;
        if (dt > 0 && dt < 0.1f) { // Sanity check
            float distance_change = (int16_t)(mtf02_raw.distance_cm - prev_distance) / 100.0f;
            mtf02_nav.velocity_z = distance_change / dt;
        }
    }
    prev_distance = mtf02_raw.distance_cm;
    
    // Integrate position
    if (mtf02_nav.last_nav_update > 0) {
        float dt = (now - mtf02_nav.last_nav_update) / 1000.0f;
        if (dt > 0 && dt < 0.1f) { // Sanity check
            mtf02_nav.position_x += mtf02_nav.velocity_x * dt;
            mtf02_nav.position_y += mtf02_nav.velocity_y * dt;
        }
    }
    
    mtf02_nav.position_z = height_m;
    mtf02_nav.last_nav_update = now;
    mtf02_nav.nav_valid = true;
}

void mtf02_apply_calibration() {
    mtf02_raw.flow_x_pixels = (mtf02_raw.flow_x_raw - mtf02_cal.flow_offset_x) * 
                              mtf02_cal.flow_scale_x / MTF02_FLOW_SCALE_FACTOR;
    mtf02_raw.flow_y_pixels = (mtf02_raw.flow_y_raw - mtf02_cal.flow_offset_y) * 
                              mtf02_cal.flow_scale_y / MTF02_FLOW_SCALE_FACTOR;
}

void mtf02_apply_orientation(float* x, float* y) {
    float temp_x = *x;
    float temp_y = *y;
    
    switch(mtf02_orient) {
        case MTF02_ORIENT_BACKWARD:
            *x = -temp_x; 
            *y = -temp_y; 
            break;
        case MTF02_ORIENT_LEFT:
            *x = temp_y; 
            *y = -temp_x; 
            break;
        case MTF02_ORIENT_RIGHT:
            *x = -temp_y; 
            *y = temp_x; 
            break;
        default: // MTF02_ORIENT_FORWARD
            break;
    }
}

void mtf02_update_performance_stats() {
    uint32_t now = millis();
    mtf02_perf.rate_counter++;
    
    // Calculate actual update rate every second
    if (now - mtf02_perf.last_rate_calc >= 1000) {
        mtf02_perf.actual_rate_hz = mtf02_perf.rate_counter * 1000.0f / (now - mtf02_perf.last_rate_calc);
        mtf02_perf.rate_counter = 0;
        mtf02_perf.last_rate_calc = now;
    }
    
    // Check for timeouts
    if ((now - mtf02_raw.last_flow_update) > MTF02_TIMEOUT_MS && mtf02_raw.flow_valid) {
        mtf02_perf.timeouts++;
        mtf02_raw.flow_valid = false;
    }
    if ((now - mtf02_raw.last_distance_update) > MTF02_TIMEOUT_MS && mtf02_raw.distance_valid) {
        mtf02_perf.timeouts++;
        mtf02_raw.distance_valid = false;
    }
}

#endif // MTF02_ENHANCED_H

// =============================================================================
// EXAMPLE USAGE IN YOUR FLIGHT CONTROLLER
// =============================================================================

void setup() {
    Serial.begin(115200);
    mtf02_init();
    
    // Optional configuration
    // mtf02_set_calibration(1.1, 0.95, 10, -5);
    // mtf02_set_orientation(MTF02_ORIENT_BACKWARD);
    
    Serial.println("MTF-02 50Hz Enhanced Library Ready");
}

void loop() {
    // CRITICAL: Call this at >100Hz for proper 50Hz sensor operation
    mtf02_update();
    
    // Your position hold controller
    if (mtf02_is_ready_for_position_hold()) {
        // Get current velocity for damping
        float vel_x = mtf02_get_velocity_x();
        float vel_y = mtf02_get_velocity_y();
        
        // Get current position for position hold
        float pos_x = mtf02_get_position_x();
        float pos_y = mtf02_get_position_y();
        
        // Your PID controller logic here
        float target_vel_x = -0.5 * pos_x; // Simple proportional position hold
        float target_vel_y = -0.5 * pos_y;
        
        float vel_error_x = target_vel_x - vel_x;
        float vel_error_y = target_vel_y - vel_y;
        
        // Apply to flight controller
        // set_roll_pitch_commands(vel_error_x * 10.0, vel_error_y * 10.0);
    }
    
    // Debug output at reasonable rate
    static uint32_t last_debug = 0;
    if (millis() - last_debug > 200) { // 5Hz debug output
        mtf02_print_status();
        last_debug = millis();
    }
    
    // Detailed stats every 10 seconds
    static uint32_t last_stats = 0;
    if (millis() - last_stats > 10000) {
        mtf02_print_detailed_stats();
        last_stats = millis();
    }
}
