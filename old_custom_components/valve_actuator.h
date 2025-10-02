#include "esphome.h"

typedef unsigned long MilliSec;

class ActuationTime : public Component, public Sensor {
  public:
    static ActuationTime* instance;     // singleton instance
    
    float actuationTime = 0;
    
    void setup() override {
    }
    
    void loop() override {
    }
    
    void setActuationTime(float duration) {
        actuationTime = duration;
        publish_state(actuationTime);
    }
};

class PeakCurrent : public Component, public Sensor {
  public:
    static PeakCurrent* instance;       // singleton instance
    
    float peakCurrent = 0;

    void setup() override {
    }
    
    void loop() override {
    }
    
    void setPeakCurrent(float amps) {
        peakCurrent = amps;
        publish_state(peakCurrent);
    }
    
    void setCurrent(float amps) {
        if (amps > peakCurrent) {
            setPeakCurrent(amps);
        }
    }
};

class ValvePosition : public PollingComponent, public TextSensor {
  public:
    static ValvePosition* instance;     // singleton instance
    
    ValvePosition() : PollingComponent(60000) {}        // call update() every 60s
    
    enum ValveStates {
        valveUnknown,       // position of valve is unknown
        valveClosed,        // water is NOT being directed to the solar panels
        valveOpening,       // valve is moving from closed towards open
        valveOpened,        // water IS being directed to the solar panels
        valveClosing,       // valve is moving from open towards closed
    };
    
    bool valvePowerRelayOn = false;        // relay enables power to valve actuator
    bool valveDirectionRelayOn = false;    // relay controls valve turning direction
    
    ValveStates valveState = valveUnknown;      // current valve position state
    MilliSec valveStateTime = 0;                // time of last valveState update
    
    void setup() override {
    }
    
    void update() override {
    }
    
    //
    // setValveState
    //
    void setValveState(ValveStates newState) {
        if (newState != valveState) {
            // state changed
            valveStateTime = millis();      // record state change time
            valveState = newState;          // update state
            publish_state(valveStateText(newState));
        }
    }
    
    const char* const valveStateText(ValveStates state) const {
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
    void setValvePowerRelayOn(bool relayOn) {
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
    void setValveDirectionRelayOn(bool relayOn) {
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
    void setCurrent(float amps) {
        MilliSec msInState = millis() - valveStateTime;
        float secInState = msInState / 1000.0;

        if (amps > 0) {
            // update peak current tracking
            PeakCurrent::instance->setCurrent(amps);
            
            // we have current, actuator is moving, update state
            ESP_LOGD("custom","----- actuator current %0.3f, valve is %s, elapsed: %0.1fs", amps,
                     valveDirectionRelayOn ? "OPENING" : "CLOSING", secInState);
            ActuationTime::instance->setActuationTime(secInState);
            setValveState(valveDirectionRelayOn ? valveOpening : valveClosing);
            
        } else {
            // no actuator current, actuator stopped by its limit switch (or power relay off)
            if (valveState == valveOpening) {
                ESP_LOGD("custom","----- actuator current 0 while OPENING, valve OPENED, elapsed: %0.1fs", secInState);
                ActuationTime::instance->setActuationTime(secInState);
                setValveState( valveOpened );
                
            } else if (instance->valveState == valveClosing) {
                ESP_LOGD("custom","----- actuator current 0 while CLOSING, valve CLOSED, elapsed: %0.1fs", secInState);
                ActuationTime::instance->setActuationTime(secInState);
                setValveState( valveClosed );
            }
        }
    }
};

ActuationTime* ActuationTime::instance = 0;
PeakCurrent* PeakCurrent::instance = 0;
ValvePosition* ValvePosition::instance = 0;
