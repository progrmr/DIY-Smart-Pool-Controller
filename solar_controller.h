#include "esphome.h"

class SolarController : public PollingComponent, public BinarySensor {
  public:
    static SolarController* instance;     // singleton instance

    // constructor
    SolarController() : PollingComponent(30000) {}      // poll every 30 sec
    
    unsigned long msValvePositionChanged = 0;   // millis() time of last valve position change
    std::string valvePosition;

    enum SolarHeatStates {
        unknown,
        solarDisabled,
        solarEnabled,
    };
    
    SolarHeatStates solarHeatState = unknown;
    
    void setup() override {
    }
    
    void update() override {
        // evaluate whether we should enable solar heat
        SolarHeatStates newState = evaluateConditions();
        
        if (newState != solarHeatState) {
            // state changed, publish new state when it changes
            solarHeatState = newState;
            publish_state(newState == solarEnabled);
        }
    }
    
    SolarHeatStates evaluateConditions() {
        // See how long it has been since we turned on/off the solar valve,
        // we limit how often it can change so we don't keep toggling back and
        // forth and burn it out.  If it has been at least 5 minutes then we
        // can consider changing it...
        unsigned long elapsed = millis() - msValvePositionChanged;
        if (elapsed < 5*60*1000) {
            // it has not been long enough, no change allowed
            ESP_LOGD("custom","----- SOLAR: NO CHANGE (only %0.0fs elapsed)", elapsed/1000.0);
            return solarHeatState;        // keep current state
        }
        
        // Check to see if the pump is running.  For now, we will not try to start
        // the pump here.  We only will enable solar heat if the pump is already
        // running and all the other conditions are met.
        float pumpRPM = id(pump_rpm_sensor).state;
        if (pumpRPM < 100.0) {
            ESP_LOGD("custom","----- SOLAR: OFF (pump is off, RPM %0.0f)", pumpRPM);
            return solarDisabled;
        }

        // Check the water temperature.  If it is already above the target temperature then
        // we don't want solar heat on.
        float targetTemp = id(pool_target_temp).state;
        float waterTemp = CtoF(id(water_temperature).state);
        if (waterTemp > targetTemp) {
            ESP_LOGD("custom","----- SOLAR: OFF (water %0.1f, meets target %0.1f)", waterTemp, targetTemp);
            return solarDisabled;
        }

        // Check the solar panel temperature.  If it isn't hot enough then we don't have solar heat
        // available.  If it is hot enough, then start solar
        float panelTemp = CtoF(id(panel_temperature).state);
        if (panelTemp >= (waterTemp + 4.0)) {
            ESP_LOGD("custom","----- SOLAR: ON (water %0.1f, panel %0.1f)", waterTemp, panelTemp);
            return solarEnabled;
        } else {
            ESP_LOGD("custom","----- SOLAR: OFF (water %0.1f, panel %0.1f, not hot enough)", waterTemp, panelTemp);
            return solarDisabled;
        }
    }
    
    void setValvePosition(std::string newPosition) {
        if (valvePosition != newPosition) {
            ESP_LOGD("custom","----- valve position changed to: %s (was %s)", newPosition.c_str(), valvePosition.c_str());
            valvePosition = newPosition;
            msValvePositionChanged = millis();
        }
    }
    
    float CtoF(float centigrade) {
        return (centigrade * 1.8) + 32.0;
    }
};

SolarController* SolarController::instance = 0;
