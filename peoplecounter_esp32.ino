#include <Wire.h>
#include <Adafruit_VL53L1X.h>


// Pin definitions
#define SDA_PIN 21
#define SCL_PIN 22


// Sensor and display objects
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();


// Counting variables
int in_count = 0;
int out_count = 0;
const float THRESHOLD_MIN = 500.0; // Min distance (mm) for person detection
const float THRESHOLD_MAX = 2900.0; // Max distance (mm)
const int HISTORY_SIZE = 10; // Store last 10 readings
float distance_history[HISTORY_SIZE];
int history_idx = 0;
bool person_detected = false;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize VL53L1X
  if (!vl53.begin(0x29, &Wire)) {
    Serial.println("Failed to initialize VL53L1X!");
    while (1);
  }
  vl53.startRanging();
  vl53.setTimingBudget(50); // 50ms for 20 Hz sampling

 

  // Initialize history
  for (int i = 0; i < HISTORY_SIZE; i++) {
    distance_history[i] = THRESHOLD_MAX + 100;
  }

  Serial.println("People Counter Started");
}

void loop() {
  if (vl53.dataReady()) {
    float distance = vl53.distance();
    //Serial.println(distance);
    
    // Update history
    distance_history[history_idx % HISTORY_SIZE] = distance;
    history_idx++;

    // Detect person
    if (distance > THRESHOLD_MIN && distance < THRESHOLD_MAX) {
      if (!person_detected) { // New person entering detection zone
        person_detected = true;
        analyze_movement();
      }
    } else {
      person_detected = false; // Reset when no person detected
    }

  }
  delay(50); // Match sensor timing budget
}

void analyze_movement() {
  // Wait for enough data to analyze direction
  if (history_idx < HISTORY_SIZE) return;

  // Calculate average of first and last 5 readings
  float avg_early = 0, avg_late = 0;
  for (int i = 0; i < 5; i++) {
    avg_early += distance_history[(history_idx - HISTORY_SIZE + i) % HISTORY_SIZE];
    avg_late += distance_history[(history_idx - 5 + i) % HISTORY_SIZE];
  }
  avg_early /= 5;
  avg_late /= 5;

  // Determine direction (50mm threshold to filter noise)
  if (avg_late > avg_early + 50 && avg_early < THRESHOLD_MAX) {
    out_count++; // Person moving away (out)
    Serial.println("Person exited. Out: " + String(out_count));
    delay(300); // Debounce to avoid double-counting
  } else if (avg_early > avg_late + 50 && avg_late < THRESHOLD_MAX) {
    in_count++; // Person approaching (in)
    Serial.println("Person entered. In: " + String(in_count));
    delay(300); // Debounce
  }
}