#include "esphome.h"

#define SPA_TARGET_TOLERANCE (0.5)  // in ºF, how much to overshoot/undershoot target pool temp
#define POOL_TARGET_TOLERANCE (0.3) // in ºF, how much to overshoot/undershoot target pool temp

// PANEL_START_OFFSET - in ºF, how many degrees panels must be > water 
// temp in order to turn on solar
#define PANEL_START_OFFSET (6)

// PANEL_STOP_OFFSET - in ºF, how many degrees panel must be > water temp
// in order to keep solar on, if panels drop below this temp, solar turns off
#define PANEL_STOP_OFFSET (1)       

#define SolarControllerPollIntervalSecs (30)    // seconds, how often to evaluate
#define DataMissingTimeOutSecs (60)             // seconds, timeout waiting for data

#define MinimumValveChangeIntervalSecs (5*60)  // seconds, don't change valve unless this time has passed
#define MinimumDesiredChangeIntervalSecs (5*60)  // seconds, don't change states unless the desired state has been constant this long

class SolarController : public PollingComponent, public BinarySensor {
  public:
    static SolarController* instance;     // singleton instance

    // constructor
    SolarController() : PollingComponent(SolarControllerPollIntervalSecs * 1000) {}

    typedef unsigned long MilliSec;
    
    enum SolarHeatStates {
        unknown,
        solarDisabled,
        solarEnabled,
    };
    
    // track desired solar heat state
    //
    MilliSec msSolarDesiredStateChanged = 0;
    SolarHeatStates desiredSolarState = unknown;
    
    // track solar heat state
    //
    MilliSec msSolarHeatStateChanged = 0;       // millis() time of last state change
    SolarHeatStates solarHeatState = unknown;
    
    // track valve position
    //
    MilliSec msValvePositionChanged = 0;        // millis() time of last valve position change
    std::string valvePosition = "unknown";
    
    // track current switch and sensor readings
    //
    MilliSec msMissingDataStarted = 0;          // millis() time of first missing data event

    void setup() override {
        const MilliSec now = millis();
        msSolarDesiredStateChanged = now;
        msSolarHeatStateChanged = now;
        msValvePositionChanged = now;
    }
    
    void update() override {
        const bool spaMode = id(spa_mode).state;
        
        // Check to see if the pump is running.  For now, we will not try to start
        // the pump here.  We only will enable solar heat if the pump is already
        // running and all the other conditions are met.
        auto pumpRPMSensor = id(pump_rpm_sensor);
        const float pumpRPM = pumpRPMSensor.has_state() ? pumpRPMSensor.state : NAN;
        if (std::isnan(pumpRPM)) {
            // pump RPM not available, leave solar state unchanged
            return;
        }
        if (pumpRPM < 100.0) {
            ESP_LOGD("custom","----- SOLAR: OFF (pump is off, RPM %0.0f)", pumpRPM);
            // no point to running solar if the pump is turned off
            setSolarHeatState(solarDisabled);
            return;
        }

        // Time Check: we don't want to run the pump from 1600-2100 local,
        // because that's SDGE peak rates (exception: allow if in Spa mode)
        //
        auto sntp = id(local_sntp_time);
        const ESPTime timeNow = sntp.now(); 
        const int hour = timeNow.hour;

        if (!spaMode) {
            if (hour >= 16 && hour < 21) {
                ESP_LOGD("custom","----- SOLAR: OFF (%02d:%02d is peak rates)",
                         hour, timeNow.minute);
                setSolarHeatState(solarDisabled);
                return;
            }
        }
        
        // get temperatures we need to decide whether to enable or disable solar
        auto waterTempC = id(estimated_water_temp);
        auto panelTempC = id(panel_temperature);

        float waterTempF = waterTempC.has_state() ? CtoF(waterTempC.state) : NAN;
        float panelTempF = panelTempC.has_state() ? CtoF(panelTempC.state) : NAN;
        float targetHeatTempF = NAN;
        float targetCoolTempF = NAN;

        if (spaMode) {
            auto spaTargetF = id(spa_target_temp);
            targetHeatTempF = spaTargetF.has_state() ? spaTargetF.state : NAN;
        } else {
            auto poolHeatTargetF = id(pool_target_temp);
            targetHeatTempF = poolHeatTargetF.has_state() ? poolHeatTargetF.state : NAN;
            auto poolTargetCoolF = id(pool_cooling_target);
            targetCoolTempF = poolTargetCoolF.has_state() ? poolTargetCoolF.state : NAN;
        }
        
        // check for missing data
        //
        std::string missingStr = "";
        
        if (std::isnan(waterTempF)) {
            missingStr += " water temp";
        }
        if (std::isnan(targetHeatTempF)) {
            missingStr += " target heat temp";
        }
        if (std::isnan(panelTempF)) {
            missingStr += " panel temp";
        }
        if (missingStr.length() > 0) {
            if (msMissingDataStarted == 0) {
                msMissingDataStarted == millis();       // start timeout clock
            }
            ESP_LOGD("custom","***** WARNING: solar missing: %s", missingStr.c_str());
        } else {
            // nothing missing, reset timeout clock
            msMissingDataStarted = 0;       
        }
        
        if (missingStr.length() > 0) {
            const MilliSec msElapsedMissing = millis() - msMissingDataStarted;
            const float secElapsedMissing = msElapsedMissing / 1000.0;
            if (secElapsedMissing >= DataMissingTimeOutSecs && solarHeatState != solarDisabled) {
                ESP_LOGD("custom","***** ERROR: timed out, missing data for %0.1fs, solar disabled",
                         secElapsedMissing);
                // missing data for long time, disable solar
                setSolarHeatState(solarDisabled);   
                return;
            } else {
                return;     // leave solar state unchanged (until timeout above)
            }
        }

        // evaluate whether we should enable solar heat
        //
        SolarHeatStates newDesiredSolarState = unknown;
        
        if (spaMode || (hour >= 8 && hour <= 19)) {
            // evaluate whether we should enable solar heating
            newDesiredSolarState = evaluateHeat(spaMode, targetHeatTempF, waterTempF, panelTempF);
        } else {
            // evaluate whether we should enable solar cooling
            newDesiredSolarState = evaluateCooling(spaMode, targetCoolTempF, waterTempF, panelTempF);
        }
        
        if (desiredSolarState != newDesiredSolarState) {
            // update time when desired state changed
            msSolarDesiredStateChanged = millis();      // track when desired state changes
            desiredSolarState = newDesiredSolarState;   // track desired state
        }
        
        // check to see how recently the desired state changed
        const MilliSec msElapsedDesired = millis() - msSolarDesiredStateChanged;
        const float secElapsedDesired = msElapsedDesired / 1000.0;
        if (secElapsedDesired >= MinimumDesiredChangeIntervalSecs) {
            // the desire has been the same for long enough, allow the change
            setSolarHeatState( newDesiredSolarState );
        }
    }
    
    void setSolarHeatState(SolarHeatStates newState) {
        if (newState != solarHeatState) {
            // See how long it has been since we turned on/off the solar valve,
            // we limit how often it can change so we don't keep toggling back and
            // forth and wear out the seals.  If it has been long enough then we
            // can consider changing it...
            // (exceptions above: if pump off or if peak electric hours)
            //
            MilliSec msElapsed = millis() - msValvePositionChanged;
            if (msElapsed < MinimumValveChangeIntervalSecs*1000) {
                // it has not been long enough, no change allowed
                ESP_LOGD("custom","***** SOLAR: too soon to change (only %0.0fs, wait %0.0fs)",
                         msElapsed/1000.0, float(MinimumValveChangeIntervalSecs));
                // don't disable or enable solar, leave it unchanged,
                // we turned the valve recently, too soon to change it again
                return;
            }

            // state changed, publish new state when it changes
            ESP_LOGD("custom","----- SOLAR: state changed to: %s", 
                     newState==solarEnabled ? "ON" : "OFF");
            msSolarHeatStateChanged = millis();
            solarHeatState = newState;
            publish_state(newState == solarEnabled);
        }
    }
    
    SolarHeatStates evaluateCooling(bool spaMode, float targetTempF, float waterTempF, float panelTempF) {
        float targetWaterTempF = targetTempF;
        if (waterTempF <= targetWaterTempF) {
            ESP_LOGD("custom","----- SOLAR: NO COOLING NEED (water %0.1f <= target %0.1f)",
                     waterTempF, targetWaterTempF);
            return solarDisabled;		// water too cold, no cooling needed
        }
        ESP_LOGD("custom","----- SOLAR: NEED COOLING (water %0.1f <= target %0.1f)", waterTempF, targetWaterTempF);
        
        // Check the solar panel temperature.  If it isn't cool enough then we don't have solar cooling
        // available.  If it is cool enough, then start solar
        //
        float maxPanelTempF = waterTempF;
        switch (solarHeatState) {
            case solarEnabled:
                // if solar is already on, we keep it on as long as the panels
                // are below the water temp by the stop offset amount
                maxPanelTempF -= PANEL_STOP_OFFSET;
                break;
            case solarDisabled:
            default:
                // if solar is off, we require the panels to be below the water
                // temp by the start offset amount in order to start solar
                maxPanelTempF -= PANEL_START_OFFSET;
                break;
        }

        if (panelTempF <= maxPanelTempF) {
            ESP_LOGD("custom",
                     "----- SOLAR: GO, PANELS COOLER (water %0.1f, panel %0.1f <= max %0.1f, target %0.1f)",
                     waterTempF, panelTempF, maxPanelTempF, targetWaterTempF);
            return solarEnabled;
        } else {
            ESP_LOGD("custom",
                     "----- SOLAR: NO GO, PANELS WARMER (water %0.1f, panel %0.1f > max %0.1f, target %0.1f)",
                     waterTempF, panelTempF, maxPanelTempF, targetWaterTempF);
            return solarDisabled;
        }
    }

    SolarHeatStates evaluateHeat(bool spaMode, float targetTempF, float waterTempF, float panelTempF) {
        // Check the water temperature.  If it is already above the target 
        // temperature then we don't want solar heat on.
        float tolerance = spaMode ? SPA_TARGET_TOLERANCE : POOL_TARGET_TOLERANCE;
        float targetWaterTempF = targetTempF;

        switch (solarHeatState) {
            case solarDisabled:
                // if solar is off, we don't turn it on again until water is below the
                // target temperature by the amount of the tolerance
                targetWaterTempF -= tolerance;
                break;
            case solarEnabled:
                // if solar is on, we keep it on until water is over the target temp
                // by the tolerance amount, 
                // that is: keep it on until we are slightly above target
                // otherwise it will oscillate on/off as soon as it drops by 0.01 degree 
                targetWaterTempF += tolerance;
                break;
            default:
                break;
        }
        
        if (waterTempF >= targetWaterTempF) {
            ESP_LOGD("custom","----- SOLAR: NO HEAT NEED (water %0.1f >= target %0.1f)", 
                     waterTempF, targetWaterTempF);
            return solarDisabled;
        }

        ESP_LOGD("custom","----- SOLAR: NEED HEAT (water %0.1f < target %0.1f)",
                 waterTempF, targetWaterTempF);

        // Check the solar panel temperature.  If it isn't hot enough then we don't have solar heat
        // available.  If it is hot enough, then start solar
        //
        // panels must be hotter than the water or we don't run the solar,
        // lower thresholds don't work well, because as soon as we send water to the
        // panels they will cool off a little and then be under threshold, 
        // causing on/off oscillation
        float minPanelTempF = waterTempF;
        switch (solarHeatState) {
            case solarEnabled:
                // if solar is already on, we keep it on as long as the panels
                // are above the water temp by the stop offset amount
                minPanelTempF += PANEL_STOP_OFFSET;
                break;
            case solarDisabled:
            default:
                // if solar is off, we require the panels to be above the water
                // temp by the start offset amount in order to start solar
                minPanelTempF += PANEL_START_OFFSET;
                break;
        }
        
        if (panelTempF >= minPanelTempF) {
            ESP_LOGD("custom",
                     "----- SOLAR: GO, PANELS HOT (panel %0.1f >= min %0.1f)",
                     panelTempF, minPanelTempF);
            return solarEnabled;
        } else {
            ESP_LOGD("custom",
                     "----- SOLAR: NO GO, PANELS COOLER (panel %0.1f < min %0.1f)",
		     panelTempF, minPanelTempF);
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

#define MaxWaterTempAgeSeconds (300)  
#define TemperatureUpdateIntervalSeconds (60)
#define StdTempDropFPH (1.5/6.5)        // w 20ºF diff, pool lost 1.5º in 6.5 hrs
#define StdTempDropFPH2 (4.2/13.0)      // w 24ºF diff, pool lost 4.2º in 13 hrs
#define TempEstimatePollingInterval (60)

class TemperatureEstimater : public PollingComponent, public Sensor {
public:
    static TemperatureEstimater* instance;      // singleton
    
    // constructor
    TemperatureEstimater() : PollingComponent(TempEstimatePollingInterval * 1000) {} 

    typedef unsigned long MilliSec;

    float panelTempC = NAN;             // solar panels temp
    
    float lastWaterTempC = NAN;         // last valid water temp reading
    MilliSec msLastWaterTemp = 0;       // time of last valid water temp

    float lastEstimateC = NAN;          // last estimated water temp
    MilliSec msLastEstimate = 0;        // time of last estimated water temp
    
    MilliSec msLastStateUpdate = 0;
    
    void setup() override {
        msLastStateUpdate = millis();
    }
    
    void update() override {
        const MilliSec msNow = millis();
        const float secSinceStateUpdate = secElapsed(msNow, msLastStateUpdate);
        
        if (secSinceStateUpdate < TemperatureUpdateIntervalSeconds) {
            return;     // only do state updates few minutes
        }

        if (std::isnan(lastWaterTempC) || std::isnan(panelTempC)) {
            // we haven't gotten a water or panel temp yet, can't estimate
            return;
        }
        
        if (msLastWaterTemp != 0) {
            // if we haven't gotten a valid water temp in the past
            // minutes then estimate the water temp
            float secElapsedSinceValid = secElapsed(msNow, msLastWaterTemp);
            
            if (secElapsedSinceValid >= MaxWaterTempAgeSeconds) {
                // we haven't received a valid water temp for some time,
                // so we should estimate water temp now
                MilliSec msLastTemp = msLastWaterTemp;
                float lastTempC = lastWaterTempC;
                
                if (!std::isnan(lastEstimateC)) {
                    // we have a previous estimate, use it
                    lastTempC = lastEstimateC;
                    msLastTemp = msLastEstimate;
                }
                
                lastEstimateC = estimatedTempC(lastTempC, msLastTemp, panelTempC);
                msLastEstimate = msNow;
                publish_state(lastEstimateC);
            }
        }
    }
    
    float estimatedTempC(float lastTempC, MilliSec msLastTemp, float panelTempC) {
        const float lastEstimateF = CtoF(lastTempC);
        const float panelTempF = CtoF(panelTempC);
        
        // water temp drops 1.5ºF per 6.5h when panel is 10-20º below water temp
        const float difference = panelTempF - lastEstimateF;    // - colder, + warmer
        float dropRateFPH = 0;
        
        if (difference < -22) {
            dropRateFPH = StdTempDropFPH;
        } else if (difference < -12) {
            // it's colder out, use the drop rate
            dropRateFPH = StdTempDropFPH * 0.90;
        } else if (difference < -5) {
            // a little colder, use reduced drop rate
            dropRateFPH = StdTempDropFPH * 0.67;
        } else if (difference < 0) {
            // slightly colder, use reduced drop rate
            dropRateFPH = StdTempDropFPH * 0.33;
        } else if (difference < 10) {
            // slightly warmer, use reduced climb rate
            dropRateFPH = -StdTempDropFPH * 0.33;
        } else {
            // warmer, use climb rate
            dropRateFPH = -StdTempDropFPH * 0.67;
        } 
        const float dropRateFPM = dropRateFPH / 60.0;       // convert to per minute rate
        const float secElapsedSinceLast = secElapsed(millis(), msLastTemp);
        const float minElapsedSinceLast = secElapsedSinceLast / 60.0;
        const float dropF = dropRateFPM * minElapsedSinceLast;
        const float newEstimateF = lastEstimateF - dropF;
        ESP_LOGD("custom","--- ESTIMATE: %0.3fºF := %0.3f - %0.3f (in %0.0f min)", 
                 newEstimateF, lastEstimateF, dropF, minElapsedSinceLast);
        return FtoC(newEstimateF);
    }
    
    void setPanelTempC(float newPanelTempC) {
        panelTempC = newPanelTempC;
    }
    
    void setWaterTempC(float newWaterTempC) {
        if (!std::isnan(newWaterTempC)) {
            // use the sensor's water temperature
            const MilliSec msNow = millis();
            lastWaterTempC = newWaterTempC;
            msLastWaterTemp = msNow;

            lastEstimateC = NAN;            // remove old estimate
            msLastEstimate = 0;             // no estimate
            
            publish_state(newWaterTempC);
            msLastStateUpdate = msNow;
        }
    }
    
    float secElapsed(MilliSec now, MilliSec previous) {
        const MilliSec msElapsed = now - previous;
        return msElapsed / 1000.0;
    }
    
    float CtoF(float centigrade) {
        return (centigrade * 1.8) + 32.0;
    }

    float FtoC(float fahrenheit) {
        return (fahrenheit - 32.0) / 1.8;
    }
    
};

SolarController* SolarController::instance = 0;
TemperatureEstimater* TemperatureEstimater::instance = 0;
