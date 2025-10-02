//
//  solar_controller.h
//  Pool Controller
//
//  Created by Gary Morris on 2025-09-26.
//

#include "solar_controller.h"
#include "../pool_pump_rs485/pool_pump_rs485.h"
#include "esphome/core/preferences.h"
#include "esphome/core/helpers.h"
#include "esphome/components/time/real_time_clock.h"
#include <string>

static const char* const PREF_KEY = "solar_flow_state";
static const char *const TAG = "solar_controller";

SolarController* SolarController::getInstance() {
  return instance_;
}

// constructor
SolarController::SolarController() : esphome::PollingComponent(SolarControllerPollIntervalSecs * 1000) {
  instance_ = this;
}

void SolarController::setup() {
    // init times to now
    const MilliSec now = millis();
    msDesiredFlowStateChanged = now;
    msCurrentFlowStateChanged = now;

    // Create a preferences object
//    auto prefs = esphome::global_preferences->make_preference<uint32_t>(PREF_KEY);
    auto prefs = esphome::global_preferences->make_preference<uint32_t>(sizeof(uint32_t), esphome::fnv1_hash(PREF_KEY));
    uint32_t saved_state;

    // Attempt to load the saved value from flash
    if (prefs.load(&saved_state)) {
        // If successful, cast the integer back to bool and set the state
        bool wasFlowing = static_cast<bool>(saved_state);
        setSolarFlowState(wasFlowing ? FlowStates::flowing : FlowStates::idle);
        ESP_LOGD(TAG, "Restored solar flow state from flash: %s", wasFlowing ? "ON" : "OFF");
    }
}

void SolarController::setSolarFlowState(FlowStates newState) {
    if (newState != currentFlowState) {
        // state changed, publish new state when it changes
        const bool flowOn = newState == FlowStates::flowing;
        ESP_LOGD("custom","----- SOLAR: state changed to: %s", flowOn ? "ON" : "OFF");
        msCurrentFlowStateChanged = millis();
        currentFlowState = newState;

        if (solarFlowSensor_ != nullptr) {
            solarFlowSensor_->publish_state(flowOn);
        }

        if (newState == FlowStates::flowing || newState == FlowStates::idle) {
            // Create a preferences object and save the new state to flash
//            auto prefs = esphome::global_preferences->make_preference<uint32_t>(PREF_KEY);
            auto prefs = esphome::global_preferences->make_preference<uint32_t>(sizeof(uint32_t), esphome::fnv1_hash(PREF_KEY));
            // Cast the enum to an integer for saving
            const uint32_t saved_state = static_cast<uint32_t>(flowOn);
            prefs.save(&saved_state);
        }
    }
}

void SolarController::update() {
    // Check to see if the pump is running.  For now, we will not try to start
    // the pump here.  We only will enable solar heat if the pump is already
    // running and all the other conditions are met.
    auto pumpRPMSensor = esphome::pool_pump_rs485::PoolPumpRS485::getInstance()->get_rpm_sensor();
    const float pumpRPM = pumpRPMSensor->has_state() ? pumpRPMSensor->state : NAN;
    if (std::isnan(pumpRPM)) {
        // pump RPM not available, leave solar state unchanged
        return;
    }
    if (pumpRPM < 100.0) {
        ESP_LOGD("custom","----- SOLAR: OFF (pump is off, RPM %0.0f)", pumpRPM);
        // no point to running solar if the pump is turned off
        setSolarFlowState(FlowStates::idle);
        return;
    }


    // --- Time Check using localtime() with a safety check ---
    // This is the Unix timestamp for January 1, 2021.
    // We use it to check if the current time is a modern date.
    const time_t MIN_VALID_TIME = 1609459200;
    time_t now_ts = time(nullptr);

    // This is the crucial safety check!
    if (now_ts < MIN_VALID_TIME) {
        // The clock is not synced yet. Exit and try again on the next update.
        ESP_LOGD("custom", "Time not synced, skipping solar logic.");
        return;
    }

    // Time Check: we don't want to run the pump from 1600-2100 local,
    // because that's SDGE peak rates (exception: allow if in Spa mode)
    //
    struct tm *timeinfo = localtime(&now_ts);
    const int hour = timeinfo->tm_hour;

    if (!spa_mode_) {
        if (hour >= 16 && hour < 21) {
            ESP_LOGD("custom","----- SOLAR: OFF (%02d hours is peak rates)", hour);
            setSolarFlowState(FlowStates::idle);
            return;
        }
    }

    // get temperatures we need to decide whether to enable or disable solar
    float waterTempF = std::isnan(estimated_water_temp_) ? NAN : CtoF(estimated_water_temp_);
    float panelTempF = std::isnan(panel_temperature_) ? NAN : CtoF(panel_temperature_);
    float targetHeatTempF = NAN;
    float targetCoolTempF = NAN;

    if (spa_mode_) {
      targetHeatTempF = spa_target_temp_;
    } else {
        targetHeatTempF = pool_target_temp_;
        targetCoolTempF = pool_cooling_target_;
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
            msMissingDataStarted = millis();       // start timeout clock
        }
        ESP_LOGD("custom","***** WARNING: solar missing: %s", missingStr.c_str());
    } else {
        // nothing missing, reset timeout clock
        msMissingDataStarted = 0;
    }

    if (missingStr.length() > 0) {
        const MilliSec msElapsedMissing = millis() - msMissingDataStarted;
        const float secElapsedMissing = msElapsedMissing / 1000.0;
        if (secElapsedMissing >= DataMissingTimeOutSecs && currentFlowState == FlowStates::flowing) {
            ESP_LOGD("custom","***** ERROR: timed out, missing data for %0.1fs, solar disabled",
                     secElapsedMissing);
            // missing data for long time, disable solar
            setSolarFlowState(FlowStates::idle);
        }
    }

    // evaluate whether we should enable solar heat or night cooling
    //
    FlowStates newDesiredFlowState{FlowStates::unknown};

    if (spa_mode_ || (hour >= 8 && hour < 20)) {
        // evaluate whether we should enable solar heating
        newDesiredFlowState = evaluateHeat(targetHeatTempF, waterTempF, panelTempF);
    } else {
        // evaluate whether we should enable solar cooling
        newDesiredFlowState = evaluateCooling(targetCoolTempF, waterTempF, panelTempF);
    }

    if (desiredFlowState != newDesiredFlowState) {
        // update time when desired state changed
        msDesiredFlowStateChanged = millis();     // track when desired state changed
        desiredFlowState = newDesiredFlowState;   // track desired state
    }

    // check to see how recently the desired state changed
    const MilliSec msElapsedDesired = millis() - msDesiredFlowStateChanged;
    const float secElapsedDesired = msElapsedDesired / 1000.0;
    if (secElapsedDesired >= MinimumDesiredChangeIntervalSecs) {
        // the desire has been the same for long enough, allow the change
        setSolarFlowState( newDesiredFlowState );
    }
}

SolarController::FlowStates SolarController::evaluateCooling(float targetTempF,
                                                             float waterTempF,
                                                             float panelTempF) const {
    float targetWaterTempF = targetTempF;
    if (waterTempF <= targetWaterTempF) {
        ESP_LOGD("custom","----- SOLAR: NO COOLING NEED (water %0.1f <= target %0.1f)",
                 waterTempF, targetWaterTempF);
        return FlowStates::idle;            // water too cold, no cooling needed
    }
    ESP_LOGD("custom","----- SOLAR: NEED COOLING (water %0.1f <= target %0.1f)", waterTempF, targetWaterTempF);

    // Check the solar panel temperature.  If it isn't cool enough then we don't have solar cooling
    // available.  If it is cool enough, then start solar
    //
    float maxPanelTempF = waterTempF;
    switch (currentFlowState) {
        case FlowStates::flowing:
            // if solar is already on, we keep it on as long as the panels
            // are below the water temp by the stop offset amount
            maxPanelTempF -= PANEL_STOP_OFFSET;
            break;
        case FlowStates::idle:
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
        return FlowStates::flowing;
    } else {
        ESP_LOGD("custom",
                 "----- SOLAR: NO GO, PANELS WARMER (water %0.1f, panel %0.1f > max %0.1f, target %0.1f)",
                 waterTempF, panelTempF, maxPanelTempF, targetWaterTempF);
        return FlowStates::idle;
    }
}

SolarController::FlowStates SolarController::evaluateHeat(float targetTempF,
                                                          float waterTempF,
                                                          float panelTempF) const {
    // Check the water temperature.  If it is already above the target
    // temperature then we don't want solar heat on.
    float tolerance = spa_mode_ ? SPA_TARGET_TOLERANCE : POOL_TARGET_TOLERANCE;
    float targetWaterTempF = targetTempF;

    switch (currentFlowState) {
        case FlowStates::idle:
            // if solar is off, we don't turn it on again until water is below the
            // target temperature by the amount of the tolerance
            targetWaterTempF -= tolerance;
            break;
        case FlowStates::flowing:
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
        return FlowStates::idle;
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
    switch (currentFlowState) {
        case FlowStates::flowing:
            // if solar is already on, we keep it on as long as the panels
            // are above the water temp by the stop offset amount
            minPanelTempF += PANEL_STOP_OFFSET;
            break;
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
        return FlowStates::flowing;
    } else {
        ESP_LOGD("custom",
                 "----- SOLAR: NO GO, PANELS COOLER (panel %0.1f < min %0.1f)",
                 panelTempF, minPanelTempF);
        return FlowStates::idle;
    }
}

