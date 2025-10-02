//
//  valve_actuator.h
//  Pool Controller
//
//  Created by Gary Morris on 2025-09-26.
//  Refactored by Gemini AI 2025-09-26
//
//  This controls the diverter valve in the pool water circulation loop.
//  When the valve is open water flow up to the rooftop solar panel to be heated
//  by the sun (daytime) or cooled by the air (nighttime).
//
//  The pool pump must be running for the water circulation to take place.
//
//  The position of the valve is controlled by two relays, one sets the turning
//  direction and one for power on/off.  These feed power to the valve actuator motor.
//  There is a sensor to measure the current draw by the valve actuator motor to
//  help detect problems (overcurrent, stuck valve) and to be able to tell when
//  the valve has stopped (limit switches turn off the valve when it reaches its limit).
//  We track peak motor current and measure how long it takes for the valve to turn.
//

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "common_types.h"

// This component manages the state and position of the valve.
class ValveActuator : public esphome::Component {
public:
    // singleton access
    static ValveActuator* getInstance();
    // Constructor
    ValveActuator();                    // singleton constructor


    // --- PUBLIC GETTERS (Read-Only Access) ---
    // Anyone can call these methods to get the current sensor pointers.
    // The 'const' at the end means the method doesn't change the object's state.
    esphome::sensor::Sensor* get_peakCurrentSensor() const { return peakCurrentSensor_; }
    esphome::sensor::Sensor* get_actuationTimeSensor() const { return actuationTimeSensor_; }
    esphome::text_sensor::TextSensor* get_valvePositionSensor() const { return valvePositionSensor_; }

    // --- PUBLIC SETTERS (Needed for ESPHome Setup) ---
    // These allow the ESPHome framework to link the sensors during startup.
    void set_peakCurrentSensor(esphome::sensor::Sensor *s) { peakCurrentSensor_ = s; }
    void set_actuationTimeSensor(esphome::sensor::Sensor *s) { actuationTimeSensor_ = s; }
    void set_valvePositionSensor(esphome::text_sensor::TextSensor *s) { valvePositionSensor_ = s; }

    // Method declarations
    void setup() override;

    void setValvePowerRelayOn(bool relayOn);
    void setValveDirectionRelayOn(bool relayOn);
    void setCurrent(float amps);

private:
    static inline ValveActuator* instance_{nullptr};  // pointer to instance

    // --- PRIVATE MEMBER VARIABLES ---
    // These can now only be directly written to by methods within the ValveActuator class.
    esphome::sensor::Sensor *peakCurrentSensor_{nullptr};
    esphome::sensor::Sensor *actuationTimeSensor_{nullptr};
    esphome::text_sensor::TextSensor *valvePositionSensor_{nullptr};

    // An enumeration to represent the possible states of the valve.
    enum class ValveStates {
        valveUnknown,       // position of valve is unknown
        valveClosed,        // water is NOT being directed to the solar panels
        valveOpening,       // valve is moving from closed towards open
        valveOpened,        // water IS being directed to the solar panels
        valveClosing,       // valve is moving from open towards closed
    };

    void setValveState(ValveStates newState);
    const char *const valveStateText() const;

    // State variables
    bool valvePowerRelayOn = false;        // relay enables power to valve actuator
    bool valveDirectionRelayOn = false;    // relay controls valve turning direction

    ValveStates valveState = ValveStates::valveUnknown; // current valve position
    MilliSec valveStateTime = 0;           // time of last valveState change

    float peakCurrent{0.0};             // peak actuator current in amps

    // private method declarations
    void setPeakCurrent(float amps);
    void setActuationTime(float seconds);
};
