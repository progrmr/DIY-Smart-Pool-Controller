#include "esphome.h"

#define TARGET_TEMP_TOLERANCE (0.3)     // degrees F, how much to overshoot/undershoot target pool temp
#define POLLING_INTERVAL (30)           // seconds, how often to evaluate 
#define MINIMUM_CHANGE_INTERVAL (5*60)  // seconds, don't change valve unless this time has passed
#define PANEL_TEMP_OFFSET (4)           // degrees F, how many degrees panel must be above water temp to be able to heat it

class SolarController : public PollingComponent, public BinarySensor {
  public:
    static SolarController* instance;     // singleton instance

    // constructor
    SolarController() : PollingComponent(POLLING_INTERVAL * 1000) {} 
    
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
        if (elapsed < MINIMUM_CHANGE_INTERVAL*1000) {
            // it has not been long enough, no change allowed
            ESP_LOGD("custom","----- SOLAR: NO CHANGE (%0.0f sec elapsed, need %0.0f)", 
                     elapsed/1000.0, MINIMUM_CHANGE_INTERVAL);
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
        float waterTemp = CtoF(id(water_temperature).state);
        float targetTemp = id(pool_target_temp).state;

        switch (solarHeatState) {
        case solarDisabled:
            // if solar is off, we don't turn it on unless we are below the
            // target temperature by the amount of the tolerance
            targetTemp = id(pool_target_temp).state - TARGET_TEMP_TOLERANCE;
            break;
        case solarEnabled:
            // if solar is already on, we overshoot by the tolerance amount, 
            // that is: keep it on until we are slightly above target
            // otherwise it will oscillate on/off as soon as it drops by 0.01 degree 
            targetTemp = id(pool_target_temp).state + TARGET_TEMP_TOLERANCE;
            break;
        default:
            break;
        }
        if (waterTemp >= targetTemp) {
            ESP_LOGD("custom","----- SOLAR: OFF (water %0.1f, meets target %0.1f)", waterTemp, targetTemp);
            return solarDisabled;
        }
        
        // Check the solar panel temperature.  If it isn't hot enough then we don't have solar heat
        // available.  If it is hot enough, then start solar
        float panelTemp = CtoF(id(panel_temperature).state);
        // panels must be 4º hotter than the water or we don't run the solar,
        // lower thresholds don't work well, because as soon as we send water to the
        // panels they will cool off a little and then be under threshold, 
        // causing on/off oscillation
        float minPanelTemp = waterTemp + PANEL_TEMP_OFFSET;      // panel must 4º hotter than water
        
        if (panelTemp >= minPanelTemp) {
            ESP_LOGD("custom","----- SOLAR: ON (water %0.1f, panel %0.1f (need %0.1f)", waterTemp, panelTemp, minPanelTemp);
            return solarEnabled;
        } else {
            ESP_LOGD("custom","----- SOLAR: OFF (water %0.1f, panel %0.1f (need %0.1f)", waterTemp, panelTemp, minPanelTemp);
            return solarDisabled;
        }
    }
    
    void setValvePosition(std::string newPosition) {
        if (valvePosition != newPosition) {
            ESP_LOGD("custom","----- SOLAR: valve changed to: %s (was %s)", newPosition.c_str(), valvePosition.c_str());
            valvePosition = newPosition;
            msValvePositionChanged = millis();
        }
    }
    
    float CtoF(float centigrade) {
        return (centigrade * 1.8) + 32.0;
    }
};

SolarController* SolarController::instance = 0;
