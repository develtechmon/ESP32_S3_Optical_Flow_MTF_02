// MTF-02 Optical Flow & Distance Sensor Library for Flight Controller
// Usage: Call mtf02_init() in setup(), mtf02_update() in main loop

#include <HardwareSerial.h>

// MTF-02 Hardware Configuration
#define MTF02_RX_PIN 1
#define MTF02_TX_PIN 3
#define MTF02_BAUD 115200

// Sensor data structure
struct MTF02_Data {
  // Optical Flow (raw values from sensor)
  int16_t flow_x_raw;      // Raw flow X (-32768 to 32767)
  int16_t flow_y_raw;      // Raw flow Y (-32768 to 32767)
  
  // Optical Flow (converted to pixels/sec)
  float flow_x_pixels;     // Flow X in pixels (divide raw by 10)
  float flow_y_pixels;     // Flow Y in pixels (divide raw by 10)
  
  // Distance sensor
  uint16_t distance_cm;    // Distance in centimeters
  
  // Quality indicators
  uint8_t flow_quality;    // Flow confidence 0-255 (>100 = good)
  uint8_t sensor_id;       // Sensor ID (usually 0)
  
  // Status flags
  bool flow_data_valid;    // True if recent flow data received
  bool distance_data_valid; // True if recent distance data received
  
  // Timestamps
  unsigned long last_flow_update;
  unsigned long last_distance_update;
};

// Global sensor data - access this from your flight controller
MTF02_Data mtf02;

// Private variables
HardwareSerial mtf02_serial(1);

// Initialize MTF-02 sensor
void mtf02_init() {
  mtf02_serial.begin(MTF02_BAUD, SERIAL_8N1, MTF02_RX_PIN, MTF02_TX_PIN);
  
  // Initialize data structure
  mtf02.flow_x_raw = 0;
  mtf02.flow_y_raw = 0;
  mtf02.flow_x_pixels = 0.0;
  mtf02.flow_y_pixels = 0.0;
  mtf02.distance_cm = 0;
  mtf02.flow_quality = 0;
  mtf02.sensor_id = 0;
  mtf02.flow_data_valid = false;
  mtf02.distance_data_valid = false;
  mtf02.last_flow_update = 0;
  mtf02.last_distance_update = 0;
}

// Update sensor data - call this in your main loop
void mtf02_update() {
  static uint8_t state = 0, len, msgid, idx;
  static uint8_t buf[32];
  static uint16_t crc;
  static uint8_t ck_a;
  
  while (mtf02_serial.available()) {
    uint8_t c = mtf02_serial.read();
    
    switch (state) {
      case 0: if (c == 0xFE) { crc = 0xFFFF; state = 1; } break;
      case 1: len = c; mtf02_crc_add(c, &crc); state = 2; break;
      case 2: mtf02_crc_add(c, &crc); state = 3; break;
      case 3: mtf02_crc_add(c, &crc); if (c != 200) state = 0; else state = 4; break;
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
        } else state = 0;
        break;
      case 7: ck_a = c; state = 8; break;
      case 8:
        uint16_t received = ck_a | (c << 8);
        if (received == crc) mtf02_parse_message(msgid, buf);
        state = 0;
        break;
    }
  }
  
  // Update validity flags based on timeout
  unsigned long now = millis();
  if (now - mtf02.last_flow_update > 1000) {
    mtf02.flow_data_valid = false;
  }
  if (now - mtf02.last_distance_update > 1000) {
    mtf02.distance_data_valid = false;
  }
}

// Check if optical flow data is usable for position hold
bool mtf02_flow_ready() {
  return mtf02.flow_data_valid && 
         mtf02.flow_quality > 50 && 
         mtf02.distance_data_valid && 
         mtf02.distance_cm > 8 && 
         mtf02.distance_cm < 300;
}

// Get velocity in m/s based on height
void mtf02_get_velocity(float height_m, float* vel_x, float* vel_y) {
  if (mtf02_flow_ready() && height_m > 0) {
    // Convert flow pixels to velocity (approximation)
    *vel_x = (mtf02.flow_x_pixels * height_m) / 100.0;
    *vel_y = (mtf02.flow_y_pixels * height_m) / 100.0;
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

// Private functions
void mtf02_crc_add(uint8_t d, uint16_t* crc) {
  uint8_t tmp = d ^ (*crc & 0xFF);
  tmp ^= tmp << 4;
  *crc = (*crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

void mtf02_parse_message(uint8_t id, uint8_t* p) {
  if (id == 100) { // Optical flow
    mtf02.flow_x_raw = p[20] | (p[21] << 8);
    mtf02.flow_y_raw = p[22] | (p[23] << 8);
    mtf02.flow_quality = p[25];
    mtf02.sensor_id = p[24];
    
    // Convert to pixels
    mtf02.flow_x_pixels = mtf02.flow_x_raw / 10.0;
    mtf02.flow_y_pixels = mtf02.flow_y_raw / 10.0;
    
    mtf02.flow_data_valid = true;
    mtf02.last_flow_update = millis();
    
  } else if (id == 132) { // Distance sensor
    mtf02.distance_cm = p[8] | (p[9] << 8);
    mtf02.distance_data_valid = true;
    mtf02.last_distance_update = millis();
  }
}

// Optional: Debug printing function
void mtf02_print_debug() {
  Serial.print("MTF02 - FlowX: "); Serial.print(mtf02.flow_x_pixels, 1);
  Serial.print(" FlowY: "); Serial.print(mtf02.flow_y_pixels, 1);
  Serial.print(" Quality: "); Serial.print(mtf02.flow_quality);
  Serial.print(" Distance: "); Serial.print(mtf02.distance_cm);
  Serial.print("cm Ready: "); Serial.println(mtf02_flow_ready() ? "YES" : "NO");
}

/*
// EXAMPLE USAGE IN YOUR FLIGHT CONTROLLER:
*/

void setup() {
  Serial.begin(115200);
  mtf02_init();  // Initialize MTF-02 sensor
  Serial.println("MTF-02 Flight Controller Library Ready");
}

void loop() {
  mtf02_update();  // Update sensor data
  
  // Use in position hold mode
  if (mtf02_flow_ready()) {
    float height = mtf02_get_height();
    float vel_x, vel_y;
    mtf02_get_velocity(height, &vel_x, &vel_y);
    
    // Apply velocity corrections to your PID controllers
    // Example:
    // pid_x_input = vel_x;
    // pid_y_input = vel_y;
    // your_position_hold_control(vel_x, vel_y);
  }
  
  // Debug every 500ms
  static unsigned long last_debug = 0;
  if (millis() - last_debug > 100) {
    mtf02_print_debug();
    last_debug = millis();
  }
}

// Access sensor data anywhere in your code:
// mtf02.flow_x_pixels    - Flow X in pixels
// mtf02.flow_y_pixels    - Flow Y in pixels  
// mtf02.distance_cm      - Distance in cm
// mtf02.flow_quality     - Flow quality (0-255)
// mtf02_flow_ready()     - True if data is good for position hold
