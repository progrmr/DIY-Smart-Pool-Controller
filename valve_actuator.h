#include "esphome.h"

class ValveActuator : public Component {
 public:
 
  static ValveActuator* instance;		// singleton instance

  typedef unsigned long MilliSec;

  Sensor* peakCurrentSensor = new Sensor();     // solar diverter valve actuator motor current
  Sensor* actuationTimeSensor = new Sensor();	// solar diverter valve actuation time (measured)

  float peakActuatorCurrent = 0;		// peak current draw during valve actuation
  MilliSec msActuatorStartTime = 0;		// millis() time when valve actuation started
  float actuationTime = 0;			    // time interval (seconds) to perform the valve actuation

  void setup() override {
  }

  void loop() override {
  }

  //
  // startMeasurements()
  //
  void startMeasurements() {
    peakActuatorCurrent = 0;		// starting a new measurement
    msActuatorStartTime = millis();	// starting time is now
  }

  
  //
  // updateActuatorCurrent
  //
  void updateActuatorCurrent(float amps) {
    if (msActuatorStartTime != 0) {
      // measurement in progress, start time is set
      if (amps > peakActuatorCurrent) {
        peakActuatorCurrent = amps;
        peakCurrentSensor->publish_state(amps);
      }

      MilliSec elapsed = millis() - msActuatorStartTime;
      actuationTimeSensor->publish_state(elapsed / 1000.0);

      if (amps == 0) {
        msActuatorStartTime = 0;    // actuation done, reset start clock
      }
    }
  } 

};

ValveActuator* ValveActuator::instance = 0;
