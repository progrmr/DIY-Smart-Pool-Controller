//
//  pool_temp_estimator.h
//  Pool Controller
//
//  Created by Gary Morris on 2025-09-28.
//

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "common_types.h"

// This component estimates water temperature when direct readings are unavailable.
class PoolTempEstimator : public esphome::PollingComponent {
public:
    // singleton access
    static PoolTempEstimator* getInstance();

    // --- PUBLIC GETTERS (Read-Only Access) ---
    // Anyone can call these methods to get the current sensor pointers.
    esphome::sensor::Sensor* get_estimatedTempSensor() const { return estimatedTempSensor_; }

    // --- PUBLIC SETTERS (Needed for ESPHome Setup) ---
    // These allow the ESPHome framework to link the sensors during startup.
    void set_estimatedTempSensor(esphome::sensor::Sensor *s) { estimatedTempSensor_ = s; }

    // Method declarations
    void update() override;

    void setPanelTempC(float newPanelTempC);
    void setWaterTempC(float newWaterTempC);

private:
    // --- SENSOR MEMBERS ---
    // Public pointers to the sensor objects that we will manage.
    esphome::sensor::Sensor *estimatedTempSensor_{nullptr};

    // Constructor
    PoolTempEstimator();     // singleton constructor

    // State tracking variables
    float panelTempC = NAN;

    float lastWaterTempC = NAN;
    MilliSec msLastWaterTemp = 0;

    float lastEstimateC = NAN;
    MilliSec msLastEstimate = 0;

    // constants
    static constexpr int MaxWaterTempAgeSeconds{300};
    static constexpr float StdTempDropFPH{1.5/6.5};     // w 20ºF diff, pool lost 1.5º in 6.5 hrs
    static constexpr float StdTempDropFPH2{4.2/13.0};   // w 24ºF diff, pool lost 4.2º in 13 hrs

    // private method declarations
    float estimatedTempC(float lastTempC, MilliSec msLastTemp, float panelTempC) const;
    float secElapsed(MilliSec now, MilliSec previous) const;
};
