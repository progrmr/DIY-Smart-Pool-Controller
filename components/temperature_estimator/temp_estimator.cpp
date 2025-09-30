//
//  temp_estimator.cpp
//  Pool Controller
//
//  Created by Gary Morris on 2025-09-28.
//

#include "temp_estimator.h"
#include "esphome/core/log.h"

TemperatureEstimator* TemperatureEstimator::getInstance() {
    static TemperatureEstimator instance;
    return &instance;
}

// constructor
TemperatureEstimator::TemperatureEstimator() : PollingComponent() {}

void TemperatureEstimator::update() override {
    if (std::isnan(lastWaterTempC) || std::isnan(panelTempC)) {
        // we haven't gotten a water or panel temp yet, can't estimate
        return;
    }

    if (msLastWaterTemp != 0) {
        // if we haven't gotten a valid water temp in the past
        // minutes then estimate the water temp
        const MilliSec msNow = millis();
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

            if (estimatedTempSensor_) {
                estimatedTempSensor_->publish_state(lastEstimateC);
            }
        }
    }
}

float TemperatureEstimator::estimatedTempC(float lastTempC, MilliSec msLastTemp, float panelTempC) const {
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

void TemperatureEstimator::setPanelTempC(float newPanelTempC) {
    panelTempC = newPanelTempC;
}

void TemperatureEstimator::setWaterTempC(float newWaterTempC) {
    if (!std::isnan(newWaterTempC)) {
        // use the sensor's water temperature
        lastWaterTempC = newWaterTempC;
        msLastWaterTemp = millis();

        lastEstimateC = NAN;            // remove old estimate
        msLastEstimate = 0;             // no estimate

        if (estimatedTempSensor_) {
            estimatedTempSensor_->publish_state(newWaterTempC);
        }
    }
}

float TemperatureEstimator::secElapsed(MilliSec now, MilliSec previous) const {
    const MilliSec msElapsed = now - previous;
    return msElapsed / 1000.0;
}
