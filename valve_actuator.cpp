//
//  valve_actuator.cpp
//  Pool Controller
//
//  Created by Gary Morris on 2025-09-26.
//  Refactored by Gemini AI 2025-09-26
//

#include "valve_actuator.h"

// Initialize singleton instances to null.
ActuationTime *ActuationTime::instance = nullptr;
PeakCurrent *PeakCurrent::instance = nullptr;
ValvePosition *ValvePosition::instance = nullptr;

// --- ActuationTime Implementation ---
void ActuationTime::setup() {}
void ActuationTime::loop() {}

void ActuationTime::setActuationTime(float duration) {
    actuationTime = duration;
    publish_state(actuationTime);
}

// --- PeakCurrent Implementation ---
void PeakCurrent::setup() {}
void PeakCurrent::loop() {}

void PeakCurrent::setPeakCurrent(float amps) {
    peakCurrent = amps;
    publish_state(peakCurrent);
}

void PeakCurrent::setCurrent(float amps) {
    if (amps > peakCurrent) {
        setPeakCurrent(amps);
    }
}

// --- ValvePosition Implementation ---
ValvePosition::ValvePosition() : PollingComponent(60000) {}  // Poll every 60s
void ValvePosition::setup() {}
void ValvePosition::update() {}

void ValvePosition::setValveState(ValveStates newState) {
    if (newState != valveState) {
        // state changed
        valveStateTime = millis();      // record state change time
        valveState = newState;          // update state
        publish_state(valveStateText(newState));
    }
}

const char *const ValvePosition::valveStateText(ValveStates state) {
    switch (state) {
        case valveClosed:
            return "closed";
        case valveOpening:
            return "opening";
        case valveOpened:
            return "opened";
        case valveClosing:
            return "closing";
        default:
            return "unknown";
    }
}

void ValvePosition::setValvePowerRelayOn(bool relayOn) {
    if (relayOn != valvePowerRelayOn) {
        // power relay changed, update state
        valvePowerRelayOn = relayOn;

        if (valvePowerRelayOn) {
            // relay just turned on, reset actuation time and peak measurements
            PeakCurrent::instance->setPeakCurrent(0);
            ActuationTime::instance->setActuationTime(0);

            // update valve state based on turning direction
            // actuator power is on, set one of the moving states based on direction
            setValveState(valveDirectionRelayOn ? valveOpening : valveClosing);
        }
    }
}

void ValvePosition::setValveDirectionRelayOn(bool relayOn) {
    if (relayOn != valveDirectionRelayOn) {
        // direction relay changed, update state
        valveDirectionRelayOn = relayOn;

        if (valvePowerRelayOn) {
            // actuator changed direction while power was on, reset actuation time
            ActuationTime::instance->setActuationTime(0);

            // actuator changed direction while power was on, update direction
            setValveState(valveDirectionRelayOn ? valveOpening : valveClosing);
        }
    }
}

//
// setCurrent - current tells us whether valve position is moving or not
//
void ValvePosition::setCurrent(float amps) {
    MilliSec msInState = millis() - valveStateTime;
    float secInState = msInState / 1000.0;

    if (amps > 0) {
        // Actuator is drawing current, so it's moving.
        PeakCurrent::instance->setCurrent(amps);

        // we have current, actuator is moving, update state
        ESP_LOGD("custom", "----- Actuator current %0.3f, valve is %s, elapsed: %0.1fs",
                 amps, valveDirectionRelayOn ? "OPENING" : "CLOSING", secInState);
        ActuationTime::instance->setActuationTime(secInState);
        setValveState(valveDirectionRelayOn ? valveOpening : valveClosing);

    } else {
        // no actuator current, actuator stopped by its limit switch (or power relay off)
        if (valveState == valveOpening) {
            ESP_LOGD("custom",
                     "----- Actuator current 0 while OPENING, valve OPENED, elapsed: "
                     "%0.1fs",
                     secInState);
            ActuationTime::instance->setActuationTime(secInState);
            setValveState(valveOpened);
            
        } else if (valveState == valveClosing) {
            ESP_LOGD("custom",
                     "----- Actuator current 0 while CLOSING, valve CLOSED, elapsed: "
                     "%0.1fs",
                     secInState);
            ActuationTime::instance->setActuationTime(secInState);
            setValveState(valveClosed);
        }
    }
}
