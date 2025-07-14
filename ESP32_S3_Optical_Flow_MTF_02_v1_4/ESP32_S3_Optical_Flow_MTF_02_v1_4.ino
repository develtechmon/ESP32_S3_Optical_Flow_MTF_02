// Enhanced MTF-02 Optical Flow & Distance Sensor Library
// Version 3.0 - Corrected trigonometric calculations with complete API
// Author: Flight Controller Integration Version

#ifndef MTF02_CORRECTED_H
#define MTF02_CORRECTED_H

#include <HardwareSerial.h>
#include <math.h>

// ===== HARDWARE CONFIGURATION =====
#define MTF02_RX_PIN 1
#define MTF02_TX_PIN 3
#define MTF02_BAUD 115200
#define MTF02_MAVLINK_SYSID 200

// ===== 50Hz PERFORMANCE OPTIMIZATION =====
#define MTF02_MAX_BYTES_PER_UPDATE 150  // Optimized for 50Hz
#define MTF02_EXPECTED_RATE_HZ 50       // Sensor update rate
#define MTF02_TIMEOUT_MS 100            // Fast timeout for 50Hz

// 100 hz
//#define MTF01_EXPECTED_RATE_HZ 100

// NEW (100Hz - double the throughput):
//#define MTF01_MAX_BYTES_PER_UPDATE 300

// NEW (100Hz - faster timeout):
//#define MTF01_TIMEOUT_MS 50

// NEW:
//velocity_x = flow_x_pixels * meters_per_pixel * MTF01_EXPECTED_RATE_HZ;

//Serial.printf("Actual rate: %.1f Hz (target: %d Hz)\n", actual_rate_hz, MTF01_EXPECTED_RATE_HZ);

// ===== SENSOR PARAMETERS =====
#define MTF02_FLOW_MIN_QUALITY 50       // Minimum quality threshold
#define MTF02_MIN_DISTANCE_CM 8         // Minimum valid distance
#define MTF02_MAX_DISTANCE_CM 300       // Maximum valid distance

// ===== SENSOR SPECIFICATIONS (CORRECTED) =====
#define MTF02_FOV_RAD 0.733             // 42 degrees in radians
#define MTF02_SENSOR_RESOLUTION 35.0    // PMW3901 sensor resolution (35x35 pixels)
#define MTF02_FLOW_SCALE_FACTOR 10.0    // Raw to calibrated conversion (for compatibility)

// ===== ORIENTATION OPTIONS =====
enum MTF02_Orientation {
    MTF02_ORIENT_FORWARD = 0,
    MTF02_ORIENT_BACKWARD,
    MTF02_ORIENT_LEFT,
    MTF02_ORIENT_RIGHT
};

// ===== DATA STRUCTURES =====
struct MTF02_RawData {
    // Raw sensor values
    int16_t flow_x_raw;
    int16_t flow_y_raw;
    uint16_t distance_cm;
    uint8_t flow_quality;
    uint8_t sensor_id;
    
    // Calibrated flow values
    float flow_x_pixels;
    float flow_y_pixels;
    
    // Timestamps and validity
    uint32_t last_flow_update;
    uint32_t last_distance_update;
    bool flow_valid;
    bool distance_valid;
};

struct MTF02_Navigation {
    // Velocity in m/s (CORRECTED CALCULATIONS)
    float velocity_x;
    float velocity_y;
    float velocity_z;
    
    // Position in meters (integrated from velocity)
    float position_x;
    float position_y;
    float position_z;
    
    // Navigation state
    uint32_t last_nav_update;
    bool nav_valid;
};

struct MTF02_Calibration {
    float flow_scale_x;
    float flow_scale_y;
    int16_t flow_offset_x;
    int16_t flow_offset_y;
    
    // Advanced calibration
    float rotation_correction;      // Correct for sensor rotation
    bool use_trigonometric_scaling; // Use tan() vs linear scaling
};

struct MTF02_Performance {
    uint32_t flow_packets_received;
    uint32_t distance_packets_received;
    uint32_t crc_errors;
    uint32_t parse_errors;
    uint32_t timeouts;
    float actual_rate_hz;
    uint32_t last_rate_calc;
    uint32_t rate_counter;
    float avg_flow_quality;
    uint32_t quality_samples;
    
    // Velocity statistics
    float max_velocity_x;
    float max_velocity_y;
    float velocity_variance_x;
    float velocity_variance_y;
};

// ===== GLOBAL INSTANCES =====
static MTF02_RawData mtf02_raw = {0};
static MTF02_Navigation mtf02_nav = {0};
static MTF02_Calibration mtf02_cal = {1.0, 1.0, 0, 0, 0.0, true};
static MTF02_Performance mtf02_perf = {0};
static MTF02_Orientation mtf02_orient = MTF02_ORIENT_FORWARD;
static HardwareSerial mtf02_serial(1);

// ===== PRIVATE FUNCTION DECLARATIONS =====
void mtf02_parse_mavlink();
void mtf02_process_optical_flow(uint8_t* payload);
void mtf02_process_distance_sensor(uint8_t* payload);
void mtf02_update_navigation();
void mtf02_apply_calibration();
void mtf02_apply_orientation(float* x, float* y);
void mtf02_crc_add(uint8_t d, uint16_t* crc);
void mtf02_update_performance_stats();
float mtf02_calculate_meters_per_pixel(float height_m);

// =============================================================================
// PUBLIC API FUNCTIONS - Complete API for Flight Controller Integration
// =============================================================================

// ===== INITIALIZATION =====
void mtf02_init() {
    mtf02_serial.begin(MTF02_BAUD, SERIAL_8N1, MTF02_RX_PIN, MTF02_TX_PIN);
    
    // Clear all data structures
    memset(&mtf02_raw, 0, sizeof(mtf02_raw));
    memset(&mtf02_nav, 0, sizeof(mtf02_nav));
    memset(&mtf02_perf, 0, sizeof(mtf02_perf));
    
    // Set default calibration
    mtf02_cal.flow_scale_x = 1.0;
    mtf02_cal.flow_scale_y = 1.0;
    mtf02_cal.flow_offset_x = 0;
    mtf02_cal.flow_offset_y = 0;
    mtf02_cal.rotation_correction = 0.0;
    mtf02_cal.use_trigonometric_scaling = true;
    
    // Set initial timestamps
    uint32_t now = millis();
    mtf02_nav.last_nav_update = now;
    mtf02_perf.last_rate_calc = now;
    
    Serial.println("MTF-02 Corrected Library Initialized for 50Hz");
}

// ===== MAIN UPDATE FUNCTION =====
void mtf02_update() {
    mtf02_parse_mavlink();
    mtf02_update_navigation();
    mtf02_update_performance_stats();
}

// =============================================================================
// DISTANCE/ALTITUDE API - For Altitude Hold
// =============================================================================

float mtf02_get_distance_m() {
    return mtf02_raw.distance_valid ? (mtf02_raw.distance_cm / 100.0f) : -1.0f;
}

uint16_t mtf02_get_distance_cm() {
    return mtf02_raw.distance_valid ? mtf02_raw.distance_cm : 0;
}

float mtf02_get_altitude_m() {
    return mtf02_get_distance_m();
}

uint16_t mtf02_get_altitude_cm() {
    return mtf02_get_distance_cm();
}

bool mtf02_is_distance_valid() {
    uint32_t now = millis();
    return mtf02_raw.distance_valid && 
           (now - mtf02_raw.last_distance_update) < MTF02_TIMEOUT_MS &&
           mtf02_raw.distance_cm >= MTF02_MIN_DISTANCE_CM &&
           mtf02_raw.distance_cm <= MTF02_MAX_DISTANCE_CM;
}

bool mtf02_is_altitude_valid() {
    return mtf02_is_distance_valid();
}

// ===== ADVANCED DISTANCE API =====
float mtf02_get_distance_filtered() {
    // Simple moving average filter
    static float distance_history[5] = {0};
    static int history_index = 0;
    static bool history_filled = false;
    
    if (!mtf02_is_distance_valid()) return -1.0f;
    
    distance_history[history_index] = mtf02_get_distance_m();
    history_index = (history_index + 1) % 5;
    if (history_index == 0) history_filled = true;
    
    float sum = 0;
    int count = history_filled ? 5 : history_index;
    for (int i = 0; i < count; i++) {
        sum += distance_history[i];
    }
    
    return sum / count;
}

// =============================================================================
// OPTICAL FLOW API - For Position Hold
// =============================================================================

void mtf02_get_flow_pixels(float* flow_x, float* flow_y) {
    if (flow_x) *flow_x = mtf02_raw.flow_valid ? mtf02_raw.flow_x_pixels : 0.0f;
    if (flow_y) *flow_y = mtf02_raw.flow_valid ? mtf02_raw.flow_y_pixels : 0.0f;
}

void mtf02_get_flow_raw(int16_t* flow_x, int16_t* flow_y) {
    if (flow_x) *flow_x = mtf02_raw.flow_valid ? mtf02_raw.flow_x_raw : 0;
    if (flow_y) *flow_y = mtf02_raw.flow_valid ? mtf02_raw.flow_y_raw : 0;
}

float mtf02_get_flow_x_pixels() {
    return mtf02_raw.flow_valid ? mtf02_raw.flow_x_pixels : 0.0f;
}

float mtf02_get_flow_y_pixels() {
    return mtf02_raw.flow_valid ? mtf02_raw.flow_y_pixels : 0.0f;
}

int16_t mtf02_get_flow_x_raw() {
    return mtf02_raw.flow_valid ? mtf02_raw.flow_x_raw : 0;
}

int16_t mtf02_get_flow_y_raw() {
    return mtf02_raw.flow_valid ? mtf02_raw.flow_y_raw : 0;
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

// ===== ADVANCED FLOW API =====
void mtf02_get_flow_angular_rates(float* rate_x, float* rate_y) {
    if (!mtf02_is_flow_valid()) {
        if (rate_x) *rate_x = 0.0f;
        if (rate_y) *rate_y = 0.0f;
        return;
    }
    
    float radians_per_pixel = MTF02_FOV_RAD / MTF02_SENSOR_RESOLUTION;
    if (rate_x) *rate_x = mtf02_raw.flow_x_pixels * radians_per_pixel * MTF02_EXPECTED_RATE_HZ;
    if (rate_y) *rate_y = mtf02_raw.flow_y_pixels * radians_per_pixel * MTF02_EXPECTED_RATE_HZ;
}

float mtf02_get_flow_magnitude() {
    return sqrt(mtf02_raw.flow_x_pixels * mtf02_raw.flow_x_pixels + 
                mtf02_raw.flow_y_pixels * mtf02_raw.flow_y_pixels);
}

float mtf02_get_flow_direction_rad() {
    return atan2(mtf02_raw.flow_y_pixels, mtf02_raw.flow_x_pixels);
}

// =============================================================================
// VELOCITY API - For Position Hold (CORRECTED TRIGONOMETRIC CALCULATIONS)
// =============================================================================

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

// ===== ADVANCED VELOCITY API =====
float mtf02_get_velocity_magnitude() {
    return sqrt(mtf02_nav.velocity_x * mtf02_nav.velocity_x + 
                mtf02_nav.velocity_y * mtf02_nav.velocity_y);
}

float mtf02_get_velocity_direction_rad() {
    return atan2(mtf02_nav.velocity_y, mtf02_nav.velocity_x);
}

void mtf02_get_velocity_filtered(float* vel_x, float* vel_y, float alpha) {
    static float filtered_x = 0, filtered_y = 0;
    static bool first_call = true;
    
    if (first_call || !mtf02_is_velocity_valid()) {
        filtered_x = mtf02_get_velocity_x();
        filtered_y = mtf02_get_velocity_y();
        first_call = false;
    } else {
        filtered_x = alpha * mtf02_get_velocity_x() + (1.0f - alpha) * filtered_x;
        filtered_y = alpha * mtf02_get_velocity_y() + (1.0f - alpha) * filtered_y;
    }
    
    if (vel_x) *vel_x = filtered_x;
    if (vel_y) *vel_y = filtered_y;
}

// =============================================================================
// POSITION API - For Position Hold
// =============================================================================

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
    mtf02_nav.last_nav_update = millis();
}

void mtf02_reset_position_xy() {
    mtf02_nav.position_x = 0.0f;
    mtf02_nav.position_y = 0.0f;
}

void mtf02_set_position(float pos_x, float pos_y) {
    mtf02_nav.position_x = pos_x;
    mtf02_nav.position_y = pos_y;
}

// ===== ADVANCED POSITION API =====
float mtf02_get_position_distance_from_origin() {
    return sqrt(mtf02_nav.position_x * mtf02_nav.position_x + 
                mtf02_nav.position_y * mtf02_nav.position_y);
}

float mtf02_get_position_bearing_from_origin_rad() {
    return atan2(mtf02_nav.position_y, mtf02_nav.position_x);
}

// =============================================================================
// SYSTEM STATUS API
// =============================================================================

bool mtf02_is_ready_for_position_hold() {
    return mtf02_is_flow_valid() && mtf02_is_distance_valid() && mtf02_is_velocity_valid();
}

bool mtf02_is_ready_for_altitude_hold() {
    return mtf02_is_distance_valid();
}

float mtf02_get_update_rate() {
    return mtf02_perf.actual_rate_hz;
}

void mtf02_get_performance_stats(uint32_t* packets, uint32_t* errors, float* rate) {
    if (packets) *packets = mtf02_perf.flow_packets_received + mtf02_perf.distance_packets_received;
    if (errors) *errors = mtf02_perf.crc_errors + mtf02_perf.parse_errors;
    if (rate) *rate = mtf02_perf.actual_rate_hz;
}

bool mtf02_is_system_healthy() {
    float error_rate = (float)(mtf02_perf.crc_errors + mtf02_perf.parse_errors) / 
                       (float)(mtf02_perf.flow_packets_received + mtf02_perf.distance_packets_received + 1);
    return error_rate < 0.05 && // Less than 5% error rate
           mtf02_perf.actual_rate_hz > 40.0 && // At least 40Hz
           mtf02_get_flow_quality() > MTF02_FLOW_MIN_QUALITY;
}

// =============================================================================
// CONFIGURATION API
// =============================================================================

void mtf02_set_calibration(float scale_x, float scale_y, int16_t offset_x, int16_t offset_y) {
    mtf02_cal.flow_scale_x = scale_x;
    mtf02_cal.flow_scale_y = scale_y;
    mtf02_cal.flow_offset_x = offset_x;
    mtf02_cal.flow_offset_y = offset_y;
}

void mtf02_set_advanced_calibration(float scale_x, float scale_y, int16_t offset_x, int16_t offset_y, 
                                   float rotation_rad, bool use_trigonometric) {
    mtf02_set_calibration(scale_x, scale_y, offset_x, offset_y);
    mtf02_cal.rotation_correction = rotation_rad;
    mtf02_cal.use_trigonometric_scaling = use_trigonometric;
}

void mtf02_set_orientation(MTF02_Orientation orientation) {
    mtf02_orient = orientation;
}

void mtf02_set_flow_quality_threshold(uint8_t min_quality) {
    // This would require modifying the #define, so we'll store it differently
    // For now, just document the current threshold
    Serial.printf("Current flow quality threshold: %d, requested: %d\n", MTF02_FLOW_MIN_QUALITY, min_quality);
}

// =============================================================================
// DEBUG API
// =============================================================================

void mtf02_print_status() {
    Serial.printf("MTF02: Alt=%.2fm FlowQ=%d VelX=%.2f VelY=%.2f PosX=%.2f PosY=%.2f Ready=%s Rate=%.1fHz\n",
                 mtf02_get_altitude_m(),
                 mtf02_get_flow_quality(),
                 mtf02_get_velocity_x(),
                 mtf02_get_velocity_y(),
                 mtf02_get_position_x(),
                 mtf02_get_position_y(),
                 mtf02_is_ready_for_position_hold() ? "YES" : "NO",
                 mtf02_get_update_rate());
}

void mtf02_print_detailed_debug() {
    Serial.printf("MTF02 DETAILED DEBUG:\n");
    Serial.printf("  Distance: %.2fm (%dcm) Valid: %s\n", 
                 mtf02_get_distance_m(), mtf02_get_distance_cm(), 
                 mtf02_is_distance_valid() ? "YES" : "NO");
    
    float flow_x, flow_y;
    mtf02_get_flow_pixels(&flow_x, &flow_y);
    Serial.printf("  Flow: X=%.1f Y=%.1f pixels, Quality=%d, Valid: %s\n",
                 flow_x, flow_y, mtf02_get_flow_quality(), 
                 mtf02_is_flow_valid() ? "YES" : "NO");
    
    Serial.printf("  Velocity: X=%.3f Y=%.3f Z=%.3f m/s, Valid: %s\n",
                 mtf02_get_velocity_x(), mtf02_get_velocity_y(), mtf02_get_velocity_z(),
                 mtf02_is_velocity_valid() ? "YES" : "NO");
    
    Serial.printf("  Position: X=%.2f Y=%.2f Z=%.2f m\n",
                 mtf02_get_position_x(), mtf02_get_position_y(), mtf02_get_position_z());
    
    Serial.printf("  System: Healthy=%s Position_Ready=%s Altitude_Ready=%s\n",
                 mtf02_is_system_healthy() ? "YES" : "NO",
                 mtf02_is_ready_for_position_hold() ? "YES" : "NO",
                 mtf02_is_ready_for_altitude_hold() ? "YES" : "NO");
    
    Serial.printf("  Calibration: ScaleX=%.2f ScaleY=%.2f OffsetX=%d OffsetY=%d\n",
                 mtf02_cal.flow_scale_x, mtf02_cal.flow_scale_y, 
                 mtf02_cal.flow_offset_x, mtf02_cal.flow_offset_y);
}

void mtf02_print_performance_stats() {
    Serial.println("=== MTF-02 Performance Statistics ===");
    Serial.printf("Flow packets: %lu\n", mtf02_perf.flow_packets_received);
    Serial.printf("Distance packets: %lu\n", mtf02_perf.distance_packets_received);
    Serial.printf("CRC errors: %lu\n", mtf02_perf.crc_errors);
    Serial.printf("Parse errors: %lu\n", mtf02_perf.parse_errors);
    Serial.printf("Timeouts: %lu\n", mtf02_perf.timeouts);
    Serial.printf("Actual rate: %.1f Hz (target: %d Hz)\n", mtf02_perf.actual_rate_hz, MTF02_EXPECTED_RATE_HZ);
    Serial.printf("Avg flow quality: %.1f/255\n", mtf02_perf.avg_flow_quality);
    Serial.printf("Max velocities: X=%.3f Y=%.3f m/s\n", mtf02_perf.max_velocity_x, mtf02_perf.max_velocity_y);
    Serial.println("====================================");
}

void mtf02_print_calibration_help() {
    Serial.println("=== MTF-02 Calibration Guide ===");
    Serial.println("1. Place drone at known height (e.g., 1.0m)");
    Serial.println("2. Move drone known distance (e.g., 1.0m) slowly");
    Serial.println("3. Compare integrated position with actual movement");
    Serial.println("4. Adjust scale factors accordingly");
    Serial.printf("Current scale factors: X=%.3f Y=%.3f\n", mtf02_cal.flow_scale_x, mtf02_cal.flow_scale_y);
    Serial.printf("Current offsets: X=%d Y=%d\n", mtf02_cal.flow_offset_x, mtf02_cal.flow_offset_y);
    Serial.println("===============================");
}

// =============================================================================
// PRIVATE IMPLEMENTATION - CORRECTED TRIGONOMETRIC CALCULATIONS
// =============================================================================

float mtf02_calculate_meters_per_pixel(float height_m) {
    if (mtf02_cal.use_trigonometric_scaling) {
        // CORRECTED: Use tangent for proper geometric scaling
        float half_fov = MTF02_FOV_RAD / 2.0f;
        float ground_width = 2.0f * height_m * tan(half_fov);
        return ground_width / MTF02_SENSOR_RESOLUTION;
    } else {
        // Legacy linear scaling for compatibility
        float radians_per_pixel = MTF02_FOV_RAD / MTF02_SENSOR_RESOLUTION;
        return height_m * radians_per_pixel;
    }
}

void mtf02_update_navigation() {
    uint32_t now = millis();
    
    // Check if we have valid data for navigation
    if (!mtf02_is_flow_valid() || !mtf02_is_distance_valid()) {
        mtf02_nav.nav_valid = false;
        return;
    }
    
    // Get current height
    float height_m = mtf02_raw.distance_cm / 100.0f;
    
    // CORRECTED: Calculate velocity using proper trigonometric scaling
    if (height_m > 0) {
        float meters_per_pixel = mtf02_calculate_meters_per_pixel(height_m);
        
        // Convert flow pixels to velocity (meters per second)
        // Note: We multiply by frame rate to convert from displacement per frame to velocity
        mtf02_nav.velocity_x = mtf02_raw.flow_x_pixels * meters_per_pixel * MTF02_EXPECTED_RATE_HZ;
        mtf02_nav.velocity_y = mtf02_raw.flow_y_pixels * meters_per_pixel * MTF02_EXPECTED_RATE_HZ;
        
        // Apply orientation correction
        mtf02_apply_orientation(&mtf02_nav.velocity_x, &mtf02_nav.velocity_y);
        
        // Apply rotation correction if calibrated
        if (abs(mtf02_cal.rotation_correction) > 0.01) {
            float cos_rot = cos(mtf02_cal.rotation_correction);
            float sin_rot = sin(mtf02_cal.rotation_correction);
            float temp_x = mtf02_nav.velocity_x * cos_rot - mtf02_nav.velocity_y * sin_rot;
            float temp_y = mtf02_nav.velocity_x * sin_rot + mtf02_nav.velocity_y * cos_rot;
            mtf02_nav.velocity_x = temp_x;
            mtf02_nav.velocity_y = temp_y;
        }
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
    
    // Integrate position (X and Y from flow, Z from distance sensor)
    if (mtf02_nav.last_nav_update > 0) {
        float dt = (now - mtf02_nav.last_nav_update) / 1000.0f;
        if (dt > 0 && dt < 0.1f) { // Sanity check
            mtf02_nav.position_x += mtf02_nav.velocity_x * dt;
            mtf02_nav.position_y += mtf02_nav.velocity_y * dt;
        }
    }
    
    // Z position directly from distance sensor
    mtf02_nav.position_z = height_m;
    
    mtf02_nav.last_nav_update = now;
    mtf02_nav.nav_valid = true;
    
    // Update performance statistics
    mtf02_perf.max_velocity_x = max(mtf02_perf.max_velocity_x, abs(mtf02_nav.velocity_x));
    mtf02_perf.max_velocity_y = max(mtf02_perf.max_velocity_y, abs(mtf02_nav.velocity_y));
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

// ===== MAVLINK PARSING (UNCHANGED) =====
void mtf02_parse_mavlink() {
    static uint8_t state = 0, len, msgid, idx;
    static uint8_t buf[64];
    static uint16_t crc;
    static uint8_t ck_a;
    
    int bytes_processed = 0;
    
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
                mtf02_crc_add(c, &crc); 
                state = 2; 
                break;
                
            case 2: // Sequence
                mtf02_crc_add(c, &crc); 
                state = 3; 
                break;
                
            case 3: // System ID
                mtf02_crc_add(c, &crc); 
                if (c != MTF02_MAVLINK_SYSID) { 
                    state = 0; 
                    mtf02_perf.parse_errors++; 
                } else {
                    state = 4; 
                }
                break;
                
            case 4: // Component ID
                mtf02_crc_add(c, &crc); 
                state = 5; 
                break;
                
            case 5: // Message ID
                msgid = c; 
                mtf02_crc_add(c, &crc); 
                idx = 0; 
                state = 6; 
                break;
                
            case 6: // Payload
                if (idx < len && idx < sizeof(buf)) {
                    buf[idx++] = c; 
                    mtf02_crc_add(c, &crc);
                    if (idx == len) {
                        // Add message-specific CRC
                        if (msgid == 100) mtf02_crc_add(175, &crc);  // OPTICAL_FLOW
                        if (msgid == 132) mtf02_crc_add(85, &crc);   // DISTANCE_SENSOR
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
    
    // Update statistics
    mtf02_perf.flow_packets_received++;
    mtf02_perf.avg_flow_quality = 
        (mtf02_perf.avg_flow_quality * mtf02_perf.quality_samples + mtf02_raw.flow_quality) / 
        (mtf02_perf.quality_samples + 1);
    mtf02_perf.quality_samples++;
}

void mtf02_process_distance_sensor(uint8_t* payload) {
    // Extract data from DISTANCE_SENSOR message (ID 132)
    mtf02_raw.distance_cm = payload[8] | (payload[9] << 8);
    mtf02_raw.distance_valid = true;
    mtf02_raw.last_distance_update = millis();
    mtf02_perf.distance_packets_received++;
}

void mtf02_crc_add(uint8_t d, uint16_t* crc) {
    uint8_t tmp = d ^ (*crc & 0xFF);
    tmp ^= tmp << 4;
    *crc = (*crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
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

#endif // MTF02_CORRECTED_H

// =============================================================================
// COMPLETE USAGE EXAMPLE FOR POSITION AND ALTITUDE HOLD
// =============================================================================

void setup() {
    Serial.begin(115200);
    mtf02_init();
    
    // Optional: Advanced calibration
    // mtf02_set_advanced_calibration(1.1, 0.95, 10, -5, 0.05, true);
    // mtf02_set_orientation(MTF02_ORIENT_BACKWARD);
    
    Serial.println("MTF-02 Complete API Library Ready");
    Serial.println("Commands: 'r'=reset pos, 'd'=debug, 's'=stats, 'c'=calibration help");
}

void loop() {
    // CRITICAL: Call this at >100Hz for proper 50Hz sensor operation
    mtf02_update();
    
    // =============================================================================
    // COMPLETE ALTITUDE HOLD EXAMPLE
    // =============================================================================
    
    static bool altitude_hold_enabled = false;
    static float target_altitude = 1.0f;
    static float alt_pid_integral = 0.0f;
    static float alt_pid_prev_error = 0.0f;
    
    // Altitude hold PID gains
    const float alt_kp = 200.0f;
    const float alt_ki = 50.0f; 
    const float alt_kd = 100.0f;
    
    if (mtf02_is_ready_for_altitude_hold()) {
        float current_altitude = mtf02_get_altitude_m();
        float altitude_velocity = mtf02_get_velocity_z();
        
        // PID controller for altitude
        float altitude_error = target_altitude - current_altitude;
        alt_pid_integral += altitude_error * 0.004f; // 250Hz loop = 4ms
        
        // Anti-windup
        if (alt_pid_integral > 1.0f) alt_pid_integral = 1.0f;
        if (alt_pid_integral < -1.0f) alt_pid_integral = -1.0f;
        
        float altitude_derivative = (altitude_error - alt_pid_prev_error) / 0.004f;
        alt_pid_prev_error = altitude_error;
        
        // PID output
        float throttle_adjustment = alt_kp * altitude_error + 
                                   alt_ki * alt_pid_integral + 
                                   alt_kd * altitude_derivative;
        
        // Rate damping
        throttle_adjustment -= altitude_velocity * 50.0f;
        
        // Apply to your flight controller:
        // base_throttle += (int)throttle_adjustment;
        
        // Debug output (reduce frequency)
        static uint32_t last_alt_debug = 0;
        if (millis() - last_alt_debug > 500) {
            Serial.printf("ALT: %.2fm->%.2fm Err=%.2f VelZ=%.2f Adj=%.0f\n",
                         current_altitude, target_altitude, altitude_error, 
                         altitude_velocity, throttle_adjustment);
            last_alt_debug = millis();
        }
    }
    
    // =============================================================================
    // COMPLETE POSITION HOLD EXAMPLE
    // =============================================================================
    
    static bool position_hold_enabled = false;
    static float target_pos_x = 0.0f, target_pos_y = 0.0f;
    static float pos_pid_integral_x = 0.0f, pos_pid_integral_y = 0.0f;
    static float pos_pid_prev_error_x = 0.0f, pos_pid_prev_error_y = 0.0f;
    
    // Position hold PID gains
    const float pos_kp = 0.5f;
    const float pos_ki = 0.1f;
    const float pos_kd = 0.2f;
    const float vel_kp = 1.0f;
    
    if (mtf02_is_ready_for_position_hold()) {
        // Get current state
        float current_x = mtf02_get_position_x();
        float current_y = mtf02_get_position_y();
        float vel_x = mtf02_get_velocity_x();
        float vel_y = mtf02_get_velocity_y();
        
        // Position errors
        float pos_error_x = target_pos_x - current_x;
        float pos_error_y = target_pos_y - current_y;
        
        // Position PID for X
        pos_pid_integral_x += pos_error_x * 0.004f;
        if (pos_pid_integral_x > 0.5f) pos_pid_integral_x = 0.5f;
        if (pos_pid_integral_x < -0.5f) pos_pid_integral_x = -0.5f;
        
        float pos_derivative_x = (pos_error_x - pos_pid_prev_error_x) / 0.004f;
        pos_pid_prev_error_x = pos_error_x;
        
        float target_vel_x = pos_kp * pos_error_x + 
                            pos_ki * pos_pid_integral_x + 
                            pos_kd * pos_derivative_x;
        
        // Position PID for Y
        pos_pid_integral_y += pos_error_y * 0.004f;
        if (pos_pid_integral_y > 0.5f) pos_pid_integral_y = 0.5f;
        if (pos_pid_integral_y < -0.5f) pos_pid_integral_y = -0.5f;
        
        float pos_derivative_y = (pos_error_y - pos_pid_prev_error_y) / 0.004f;
        pos_pid_prev_error_y = pos_error_y;
        
        float target_vel_y = pos_kp * pos_error_y + 
                            pos_ki * pos_pid_integral_y + 
                            pos_kd * pos_derivative_y;
        
        // Velocity errors and control
        float vel_error_x = target_vel_x - vel_x;
        float vel_error_y = target_vel_y - vel_y;
        
        float roll_adjustment = vel_error_x * vel_kp;
        float pitch_adjustment = vel_error_y * vel_kp;
        
        // Apply to your flight controller:
        // roll_command += roll_adjustment;
        // pitch_command += pitch_adjustment;
        
        // Debug output (reduce frequency)
        static uint32_t last_pos_debug = 0;
        if (millis() - last_pos_debug > 500) {
            Serial.printf("POS: X=%.2f(%.2f) Y=%.2f(%.2f) VX=%.2f VY=%.2f\n",
                         current_x, target_pos_x, current_y, target_pos_y, vel_x, vel_y);
            last_pos_debug = millis();
        }
    }
    
    // =============================================================================
    // SENSOR MONITORING AND COMMANDS
    // =============================================================================
    
    // Print basic status every 1000ms
    static uint32_t last_status = 0;
    if (millis() - last_status > 1000) {
        mtf02_print_status();
        last_status = millis();
    }
    
    // Print detailed stats every 10 seconds
    static uint32_t last_stats = 0;
    if (millis() - last_stats > 10000) {
        mtf02_print_performance_stats();
        last_stats = millis();
    }
    
    // Handle serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        switch(cmd) {
            case 'r':
                mtf02_reset_position();
                Serial.println("Position reset to origin");
                break;
            case 'd':
                mtf02_print_detailed_debug();
                break;
            case 's':
                mtf02_print_performance_stats();
                break;
            case 'c':
                mtf02_print_calibration_help();
                break;
            case 'h':
                Serial.println("Commands: r=reset, d=debug, s=stats, c=calibration");
                break;
        }
    }
}

/*
=============================================================================
INTEGRATION SUMMARY FOR YOUR FLIGHT CONTROLLER
=============================================================================

KEY CORRECTED CALCULATIONS:
1. Using tan(half_FOV) for proper geometric scaling
2. Velocity = pixels × meters_per_pixel × frame_rate
3. Position integration with proper time steps

ESSENTIAL API FUNCTIONS:

ALTITUDE HOLD:
- mtf02_get_altitude_m() / mtf02_get_distance_m()
- mtf02_get_velocity_z() for rate damping
- mtf02_is_ready_for_altitude_hold()

POSITION HOLD:
- mtf02_get_position_x(), mtf02_get_position_y()
- mtf02_get_velocity_x(), mtf02_get_velocity_y()
- mtf02_is_ready_for_position_hold()
- mtf02_reset_position()

SYSTEM MONITORING:
- mtf02_is_system_healthy()
- mtf02_get_update_rate()
- mtf02_print_status()

CALIBRATION:
- mtf02_set_calibration() for basic calibration
- mtf02_set_advanced_calibration() for rotation correction
- mtf02_print_calibration_help()

USAGE IN YOUR FLIGHT CONTROLLER:
1. Call mtf02_update() in main loop at >100Hz
2. Use the PID examples above as starting points
3. Monitor system health with mtf02_is_system_healthy()
4. Calibrate using mtf02_print_calibration_help() guidance

This library provides mathematically correct optical flow calculations
with a complete API for professional flight controller integration.
*/
