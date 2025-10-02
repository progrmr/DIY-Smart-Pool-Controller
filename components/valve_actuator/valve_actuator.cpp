#include "valve_actuator.h"
#include "esphome/core/preferences.h"
#include "esphome/core/helpers.h"

static const char *const TAG = "valve_position";
static const char* const PREF_KEY = "valve_actuator_state";

ValveActuator* ValveActuator::getInstance() {
    return instance_;
}

// constructor
ValveActuator::ValveActuator() {
  instance_ = this;
}

void ValveActuator::setup() {
    // Create a preferences object
//    auto prefs = global_preferences->make_preference<uint32_t>(PREF_KEY);
    auto prefs = esphome::global_preferences->make_preference<uint32_t>(sizeof(uint32_t), esphome::fnv1_hash(PREF_KEY));
    uint32_t saved_state;

    // Attempt to load the saved value from flash
    if (prefs.load(&saved_state)) {
        // If successful, cast the integer back to your enum and set the state
        valveState = static_cast<ValveStates>(saved_state);
        ESP_LOGD(TAG, "Restored valve state from flash: %s", valveStateText());
    } else {
        valveState = ValveStates::valveUnknown;
    }

    // Publish the initial sensor states on startup
    if (valvePositionSensor_ != nullptr) {
        valvePositionSensor_->publish_state(valveStateText());
    }
    if (peakCurrentSensor_ != nullptr) {
        peakCurrentSensor_->publish_state(peakCurrent);
    }
    if (actuationTimeSensor_ != nullptr) {
        actuationTimeSensor_->publish_state(0);
    }
}

const char* const ValveActuator::valveStateText() const {
    switch (valveState) {
        case ValveStates::valveClosed:  return "closed";
        case ValveStates::valveOpening: return "opening";
        case ValveStates::valveOpened:  return "opened";
        case ValveStates::valveClosing: return "closing";
        default:
            return "unknown";
    }
}

//
// setters
//
void ValveActuator::setPeakCurrent(float amps) {
    peakCurrent = amps;
    if (peakCurrentSensor_ != nullptr) {
        peakCurrentSensor_->publish_state(amps);
    }
}

void ValveActuator::setActuationTime(float seconds) {
    if (actuationTimeSensor_ != nullptr) {
        actuationTimeSensor_->publish_state(seconds);
    }
}

void ValveActuator::setValveState(ValveStates newState) {
    if (newState != valveState) {
        valveStateTime = millis();
        valveState = newState;
        if (valvePositionSensor_ != nullptr) {
            valvePositionSensor_->publish_state(valveStateText());
        }
        ESP_LOGD(TAG, "Valve state changed to: %s", valveStateText());

        // only write state to flash if it is opened || closed, the other
        // states are transient and it is not needed to write those to flash,
        // (which wears out the flash drive too)
        if (newState == ValveStates::valveClosed || newState == ValveStates::valveOpened) {
            // Create a preferences object and save the new state to flash
//            auto prefs = global_preferences->make_preference<uint32_t>(PREF_KEY);
            auto prefs = esphome::global_preferences->make_preference<uint32_t>(sizeof(uint32_t), esphome::fnv1_hash(PREF_KEY));
            // Cast the enum to an integer for saving
            const uint32_t saved_state = static_cast<uint32_t>(valveState);
            prefs.save(&saved_state);
        }
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
            setValveState(valveDirectionRelayOn ? ValveStates::valveOpening : ValveStates::valveClosing);
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
            setValveState(valveDirectionRelayOn ? ValveStates::valveOpening : ValveStates::valveClosing);
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
        setValveState(valveDirectionRelayOn ? ValveStates::valveOpening : ValveStates::valveClosing);

    } else {
        // no actuator current, actuator stopped by its limit switch (or power relay off)
        if (valveState == ValveStates::valveOpening) {
            ESP_LOGD(TAG,"----- actuator current 0 while OPENING, valve OPENED, elapsed: %0.1fs", secInState);
            setActuationTime(secInState);
            setValveState( ValveStates::valveOpened );

        } else if (valveState == ValveStates::valveClosing) {
            ESP_LOGD(TAG,"----- actuator current 0 while CLOSING, valve CLOSED, elapsed: %0.1fs", secInState);
            setActuationTime(secInState);
            setValveState( ValveStates::valveClosed );
        }
    }
}
