//
//  valve_actuator.h
//  Pool Controller
//
//  Created by Gary Morris on 2025-09-26.
//  Refactored by Gemini AI 2025-09-26
//

#pragma once

#include "esphome.h"

// Define a custom type for milliseconds to improve readability.
typedef unsigned long MilliSec;

// This component measures the duration of the valve's movement.
class ActuationTime : public Component, public Sensor {
public:
    static ActuationTime *instance;  // Singleton instance
    float actuationTime = 0;

    // Method declarations
    void setup() override;
    void loop() override;
    void setActuationTime(float duration);
};

// This component tracks the peak electrical current used by the valve actuator.
class PeakCurrent : public Component, public Sensor {
public:
    static PeakCurrent *instance;  // Singleton instance
    float peakCurrent = 0;

    // Method declarations
    void setup() override;
    void loop() override;
    void setPeakCurrent(float amps);
    void setCurrent(float amps);
};

// This component manages the state and position of the valve.
class ValvePosition : public PollingComponent, public TextSensor {
public:
    static ValvePosition *instance;  // Singleton instance

    // An enumeration to represent the possible states of the valve.
    enum ValveStates {
        valveUnknown,       // position of valve is unknown
        valveClosed,        // water is NOT being directed to the solar panels
        valveOpening,       // valve is moving from closed towards open
        valveOpened,        // water IS being directed to the solar panels
        valveClosing,       // valve is moving from open towards closed
    };

    // State variables
    bool valvePowerRelayOn = false;        // relay enables power to valve actuator
    bool valveDirectionRelayOn = false;    // relay controls valve turning direction
    ValveStates valveState = valveUnknown;      // current valve position state
    MilliSec valveStateTime = 0;                // time of last valveState update

    // Constructor
    ValvePosition();

    // Method declarations
    void setup() override;
    void update() override;
    void setValveState(ValveStates newState);
    const char *const valveStateText(ValveStates state);
    void setValvePowerRelayOn(bool relayOn);
    void setValveDirectionRelayOn(bool relayOn);
    void setCurrent(float amps);
};
