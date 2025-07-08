#include <Wire.h>
#include "SparkFun_VL53L1X.h"


char peopleCounterArray[50];


//Optional interrupt and shutdown pins.  Vanno cambiati e messi quelli che hanno i collegamenti i^2C
#define SHUTDOWN_PIN 3    
#define INTERRUPT_PIN 4

SFEVL53L1X distanceSensor(Wire);//, SHUTDOWN_PIN, INTERRUPT_PIN);

static int NOBODY = 0;
static int SOMEONE = 1;
static int LEFT = 0;
static int RIGHT = 1;

static int DIST_THRESHOLD_MAX[] = {0, 0};   // treshold of the two zones

static int PathTrack[] = {0,0,0,0};
static int PathTrackFillingSize = 1; // init this to 1 as we start from state where nobody is any of the zones
static int LeftPreviousStatus = NOBODY;
static int RightPreviousStatus = NOBODY;

static int center[2] = {239,175}; /* center of the two zones */  
//static int center[2] = {223,39}; /* center of the two zones */  
static int Zone = 0;
static int PplCounter = 0;

static int ROI_height = 5;
static int ROI_width = 5;

void define_threshold(){
  // the sensor does 100 measurements for each zone (zones are predefined)
  // each measurements is done with a timing budget of 100 ms, to increase the precision
  
  Zone = 0;
  float sum_zone_0 = 0;
  float sum_zone_1 = 0;
  uint16_t distance;
  int number_attempts = 100;
  for (int i=0; i<number_attempts; i++){
      // increase sum of values in Zone 0
      distanceSensor.setROI(ROI_height, ROI_width, center[Zone]);  // first value: height of the zone, second value: width of the zone
      delay(50);
      distanceSensor.setTimingBudgetInMs(50);
      distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
      distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
      distanceSensor.stopRanging();      
      sum_zone_0 = sum_zone_0 + distance;
      
      Zone++;
      Zone = Zone%2;

      // increase sum of values in Zone 1
      distanceSensor.setROI(ROI_height, ROI_width, center[Zone]);  // first value: height of the zone, second value: width of the zone
      delay(50);
      distanceSensor.setTimingBudgetInMs(50);
      distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
      distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
      distanceSensor.stopRanging();      
      sum_zone_1 = sum_zone_1 + distance;
      
      Zone++;
      Zone = Zone%2;
  }
  // after we have computed the sum for each zone, we can compute the average distance of each zone
  float average_zone_0 = sum_zone_0 / number_attempts;
  float average_zone_1 = sum_zone_1 / number_attempts;
  

  float threshold_zone_0 = average_zone_0 * 80/100; // they can be int values, as we are not interested in the decimal part when defining the threshold
  float threshold_zone_1 = average_zone_1 * 80/100;
  

  DIST_THRESHOLD_MAX[0] = threshold_zone_0;
  DIST_THRESHOLD_MAX[1] = threshold_zone_1;
 
  
  delay(2000);
}



void setup(void)
{
  Wire.begin();

  Serial.begin(115200);
  Serial.println("VL53L1X Qwiic Test");

  if (distanceSensor.init() == false)
    Serial.println("Sensor online!");
  distanceSensor.setIntermeasurementPeriod(100);
  distanceSensor.setDistanceModeLong();

  Serial.setTimeout(500);// Set time out for setup_wifi();
 
  delay(1000);
  define_threshold();
  Serial.println("Sensor Ready!");
   
  
}



void loop(void)
{
  uint16_t distance;
  
  
  distanceSensor.setROI(ROI_height, ROI_width, center[Zone]);  // first value: height of the zone, second value: width of the zone
  delay(50);
  distanceSensor.setTimingBudgetInMs(50);
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.stopRanging();

  // Serial.println(distance);
  

   // inject the new ranged distance in the people counting algorithm
  processPeopleCountingData(distance, Zone);

  Zone++;
  Zone = Zone%2;


}

// NOBODY = 0, SOMEONE = 1, LEFT = 0, RIGHT = 1

void processPeopleCountingData(int16_t Distance, uint8_t zone) {

    int CurrentZoneStatus = NOBODY;
    int AllZonesCurrentStatus = 0;
    int AnEventHasOccured = 0;

  if (Distance < DIST_THRESHOLD_MAX[Zone]) {
    // Someone is in !
    CurrentZoneStatus = SOMEONE;
  }

  // left zone
  if (zone == LEFT) {

    if (CurrentZoneStatus != LeftPreviousStatus) {
      // event in left zone has occured
      AnEventHasOccured = 1;

      if (CurrentZoneStatus == SOMEONE) {
        AllZonesCurrentStatus += 1;
      }
      // need to check right zone as well ...
      if (RightPreviousStatus == SOMEONE) {
        // event in left zone has occured
        AllZonesCurrentStatus += 2;
      }
      // remember for next time
      LeftPreviousStatus = CurrentZoneStatus;
    }
  }
  // right zone
  else {

    if (CurrentZoneStatus != RightPreviousStatus) {

      // event in left zone has occured
      AnEventHasOccured = 1;
      if (CurrentZoneStatus == SOMEONE) {
        AllZonesCurrentStatus += 2;
      }
      // need to left right zone as well ...
      if (LeftPreviousStatus == SOMEONE) {
        // event in left zone has occured
        AllZonesCurrentStatus += 1;
      }
      // remember for next time
      RightPreviousStatus = CurrentZoneStatus;
    }
  }

  // if an event has occured
  if (AnEventHasOccured) {
    if (PathTrackFillingSize < 4) {
      PathTrackFillingSize ++;
    }

    // if nobody anywhere lets check if an exit or entry has happened
    if ((LeftPreviousStatus == NOBODY) && (RightPreviousStatus == NOBODY)) {

      // check exit or entry only if PathTrackFillingSize is 4 (for example 0 1 3 2) and last event is 0 (nobobdy anywhere)
      if (PathTrackFillingSize == 4) {
        // check exit or entry. no need to check PathTrack[0] == 0 , it is always the case
        //Serial.println();
        if ((PathTrack[1] == 1)  && (PathTrack[2] == 3) && (PathTrack[3] == 2)) {
          // this is an entry
          Serial.println("IN");
        } else if ((PathTrack[1] == 2)  && (PathTrack[2] == 3) && (PathTrack[3] == 1)) {
          // This an exit
          Serial.println("OUT");
          }
      }
      // for (int i=0; i<4; i++){
      //   PathTrack[i] = 0;
      // }
      PathTrackFillingSize = 1;
    }
    else {
      // update PathTrack
      // example of PathTrack update
      // 0
      // 0 1
      // 0 1 3
      // 0 1 3 1
      // 0 1 3 3
      // 0 1 3 2 ==> if next is 0 : check if exit
      PathTrack[PathTrackFillingSize-1] = AllZonesCurrentStatus;
    }
  }
}