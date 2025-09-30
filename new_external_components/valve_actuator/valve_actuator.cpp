#include "valve_actuator.h"

static const char *const TAG = "valve_position";

ValveActuator* ValveActuator::getInstance() {
    static ValveActuator instance;
    return &instance;
}

// constructor
ValveActuator::ValveActuator() {}

void ValveActuator::setup() override {
    if (valvePositionSensor != nullptr) return;       // already set up

    // Create and register the sensors
    peakCurrentSensor = new Sensor();
    actuationTimeSensor = new Sensor();
    valvePositionSensor = new TextSensor();

    App.register_sensor(peakCurrentSensor);
    App.register_sensor(actuationTimeSensor);
    App.register_sensor(valvePositionSensor);

    // Register this component with ESPHome
    App.register_component(this)

    // Publish the initial unknown state on startup
    valvePosition->publish_state(valveStateText(valveState));
}

const char *ValveActuator::valveStateText(ValveStates state) const {
    switch (state) {
        case valveClosed:  return "closed";
        case valveOpening: return "opening";
        case valveOpened:  return "opened";
        case valveClosing: return "closing";
        default:           return "unknown";
    }
}

//
// setters
//
void ValveActuator::setPeakCurrent(float amps) {
    peakCurrent = amps;
    if (peakCurrentSensor != nullptr) {
        peakCurrentSensor->publish_state(amps);
    }
}

void ValveActuator::setActuationTime(float seconds) {
    if (actuationTimeSensor != nullptr) {
        actuationTimeSensor->publish_state(seconds);
    }
}

void ValveActuator::setValveState(ValveStates newState) {
    if (newState != valveState) {
        valveStateTime = millis();
        valveState = newState;
        valvePositionSensor->publish_state(valveStateText(newState));
        ESP_LOGD(TAG, "Valve state changed to: %s", valveStateText(newState));
    }
}

void ValveActuator::setValvePowerRelayOn(bool relayOn) {
    if (relayOn != valvePowerRelayOn) {
        // power relay state changed, update state
        valvePowerRelayOn = relayOn;

        if (relayOn) {
            // relay just turned on, reset actuation time and peak measurements
            setPeakCurrent(0.0);
            setActuationTime(0.0);

            // update valve state based on turning direction
            // actuator power is on, set one of the moving states based on direction
            setValveState(valveDirectionRelayOn ? valveOpening : valveClosing);
        }
    }
}

void ValveActuator::setValveDirectionRelayOn(bool relayOn) {
    if (relayOn != valveDirectionRelayOn) {
        // direction relay changed, update state
        valveDirectionRelayOn = relayOn;

        if (valvePowerRelayOn) {
            // actuator changed direction while power was on,
            // this really should not happen but update the direction
            setValveState(valveDirectionRelayOn ? valveOpening : valveClosing);
        }
    }
}

//
// setCurrent - current tells us whether valve position is moving or not
//
void ValveActuator::setCurrent(float amps) {
    MilliSec msInState = millis() - valveStateTime;
    float secInState = msInState / 1000.0;

    if (amps >= 0.05) {
        // update peak current tracking
        if (amps > peakCurrent) {
            setPeakCurrent(amps);
        }
        setActuationTime(secInState);

        // we have current, actuator is moving, update state
        ESP_LOGD(TAG,"----- actuator current %0.3f, valve is %s, elapsed: %0.1fs",
                 amps, valveDirectionRelayOn ? "OPENING" : "CLOSING", secInState);
        setValveState(valveDirectionRelayOn ? valveOpening : valveClosing);

    } else {
        // no actuator current, actuator stopped by its limit switch (or power relay off)
        if (valveState == valveOpening) {
            ESP_LOGD(TAG,"----- actuator current 0 while OPENING, valve OPENED, elapsed: %0.1fs", secInState);
            setActuationTime(secInState);
            setValveState( valveOpened );

        } else if (valveState == valveClosing) {
            ESP_LOGD(TAG,"----- actuator current 0 while CLOSING, valve CLOSED, elapsed: %0.1fs", secInState);
            setActuationTime(secInState);
            setValveState( valveClosed );
        }
    }
}
