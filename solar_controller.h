#include "esphome.h"

#define SPA_TARGET_TOLERANCE (0.6)      // degrees F, how much to overshoot/undershoot target pool temp
#define POOL_TARGET_TOLERANCE (0.3)     // degrees F, how much to overshoot/undershoot target pool temp
#define PANEL_START_OFFSET (4)          // degrees F, how many degrees panel must be above water temp to start solar
#define PANEL_STOP_OFFSET (2)           // degrees F, how many degrees panels must be above water temp to stop solar

#define POLLING_INTERVAL (30)           // seconds, how often to evaluate 
#define MINIMUM_CHANGE_INTERVAL (5*60)  // seconds, don't change valve unless this time has passed
#define TIMEOUT_INTERVAL (30)           // seconds, timeout waiting for data

class SolarController : public PollingComponent, public BinarySensor {
  public:
    static SolarController* instance;     // singleton instance

    // constructor
    SolarController() : PollingComponent(POLLING_INTERVAL * 1000) {} 
    
    typedef unsigned long MilliSec;
    
    enum SolarHeatStates {
        unknown,
        solarDisabled,
        solarEnabled,
    };
    
    const float nan = std::nan("");
    
    // track solar heat state
    //
    MilliSec msSolarHeatStateChanged = 0;       // millis() time of last state change
    SolarHeatStates solarHeatState = unknown;
    
    // track valve position
    //
    MilliSec msValvePositionChanged = 0;        // millis() time of last valve position change
    std::string valvePosition;
    
    // track current switch and sensor readings
    //
    MilliSec msMissingDataStarted = 0;          // millis() time of first missing data event
    bool spaMode = false;
    float pumpRPM     = std::nan("");
    float waterTempF  = std::nan("");
    float targetTempF = std::nan("");
    float panelTempF  = std::nan("");
    
    void setup() override {
        const MilliSec now = millis();
        msSolarHeatStateChanged = now;
        msValvePositionChanged = now;
    }
    
    void update() override {
        //const bool rpmHasState = id(pump_rpm_sensor).has_state();
        
        // gather all the data we need to decide whether to enable or disable solar
        //
        spaMode = id(spa_mode).state;
        if (spaMode) {
            targetTempF = id(spa_target_temp).has_state() ? id(spa_target_temp).state : nan;
        } else {
            targetTempF = id(pool_target_temp).has_state() ? id(pool_target_temp).state : nan;
        }
        pumpRPM = id(pump_rpm_sensor).has_state() ? id(pump_rpm_sensor).state : nan;
        waterTempF = id(water_temperature).has_state() ? CtoF( id(water_temperature).state ) : nan;
        panelTempF = id(panel_temperature).has_state() ? CtoF(id(panel_temperature).state) : nan;

        // check for missing data
        //
        std::string missingStr = "";
        
        if (std::isnan(pumpRPM)) {
            missingStr += " pumpRPM";
        }
        if (std::isnan(waterTempF)) {
            missingStr += " waterTemp";
        }
        if (std::isnan(targetTempF)) {
            missingStr += " targetTemp";
        }
        if (std::isnan(panelTempF)) {
            missingStr += " panelTemp";
        }
        if (missingStr.length() > 0) {
            if (msMissingDataStarted == 0) {
                msMissingDataStarted == millis();       // start timeout clock
            }
            ESP_LOGD("custom","***** WARNING: data missing %s", missingStr.c_str());
        } else {
            // nothing missing, reset timeout clock
            msMissingDataStarted = 0;       
        }
        
        // See how long it has been since we turned on/off the solar valve,
        // we limit how often it can change so we don't keep toggling back and
        // forth and burn it out.  If it has been at least 5 minutes then we
        // can consider changing it...
        MilliSec elapsed = millis() - msValvePositionChanged;
        if (elapsed < MINIMUM_CHANGE_INTERVAL*1000) {
            // it has not been long enough, no change allowed
            ESP_LOGD("custom","----- SOLAR: NO CHANGE (%0.0f sec elapsed, need %0.0f)", 
                     elapsed/1000.0, float(MINIMUM_CHANGE_INTERVAL));
            return;
        }

        if (missingStr.length() > 0) {
            MilliSec elapsedMissing = millis() - msMissingDataStarted;
            if (elapsedMissing >= TIMEOUT_INTERVAL*1000 && solarHeatState != solarDisabled) { 
                ESP_LOGD("custom","***** ERROR: timed out, still missing data, solar disabled");
                setSolarHeatState(solarDisabled);   // missing data for long time, disable solara
            }
            return;
        }

        // evaluate whether we should enable solar heat
        //
        setSolarHeatState( evaluateConditions() );
    }
    
    void setSolarHeatState(SolarHeatStates newState) {
        if (newState != solarHeatState) {
            // state changed, publish new state when it changes
            msSolarHeatStateChanged = millis();
            solarHeatState = newState;
            publish_state(newState == solarEnabled);
        }
    }
    
    SolarHeatStates evaluateConditions() {
        // Check to see if the pump is running.  For now, we will not try to start
        // the pump here.  We only will enable solar heat if the pump is already
        // running and all the other conditions are met.
        if (!spaMode) {
            if (pumpRPM < 100.0) {
                ESP_LOGD("custom","----- SOLAR: OFF (pump is off, RPM %0.0f)", pumpRPM);
                return solarDisabled;
            }
        }

        // Check the water temperature.  If it is already above the target 
        // temperature then we don't want solar heat on.
        float tolerance = spaMode ? SPA_TARGET_TOLERANCE : POOL_TARGET_TOLERANCE;
        float targetWaterTemp = targetTempF;
        
        switch (solarHeatState) {
            case solarDisabled:
                // if solar is off, we don't turn it on again until water is below the
                // target temperature by the amount of the tolerance
                targetWaterTemp -= tolerance;
                break;
            case solarEnabled:
                // if solar is on, we keep it on until water is over the target temp
                // by the tolerance amount, 
                // that is: keep it on until we are slightly above target
                // otherwise it will oscillate on/off as soon as it drops by 0.01 degree 
                targetWaterTemp += tolerance;
                break;
            default:
                break;
        }
        
        if (waterTempF >= targetWaterTemp) {
            ESP_LOGD("custom","----- SOLAR: OFF (water %0.1f, meets target %0.1f)", waterTempF, targetWaterTemp);
            return solarDisabled;
        } else {
            ESP_LOGD("custom","----- SOLAR: NEEDED (water %0.1f, target %0.1f)", waterTempF, targetWaterTemp);
        }
        
        // Check the solar panel temperature.  If it isn't hot enough then we don't have solar heat
        // available.  If it is hot enough, then start solar
        //
        // panels must be hotter than the water or we don't run the solar,
        // lower thresholds don't work well, because as soon as we send water to the
        // panels they will cool off a little and then be under threshold, 
        // causing on/off oscillation
        float minPanelTemp = waterTempF;
        switch (solarHeatState) {
            case solarEnabled:
                // if solar is already on, we keep it on until the panels drop below
                // the water temp with a lesser offset
                minPanelTemp += PANEL_STOP_OFFSET;
                break;
            case solarDisabled:
            default:
                // if solar is off, we require the panels to be above the water
                // temp by the offset amount in order to start solar
                minPanelTemp += PANEL_START_OFFSET;
                break;
        }
        
        if (panelTempF >= minPanelTemp) {
            ESP_LOGD("custom","----- SOLAR: ON ^^^^^ water %0.1f, panel %0.1f (need %0.1f), target %0.1f", 
                     waterTempF, panelTempF, minPanelTemp, targetWaterTemp);
            return solarEnabled;
        } else {
            ESP_LOGD("custom","----- SOLAR: OFF vvvvv water %0.1f, panel %0.1f (need %0.1f), target %0.1f", 
                     waterTempF, panelTempF, minPanelTemp, targetWaterTemp);
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
