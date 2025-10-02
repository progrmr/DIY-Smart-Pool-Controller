//
//  solar_controller.h
//  Pool Controller
//
//  Created by Gary Morris on 2025-09-26.
//  Refactored by Gemini AI 2025-09-30
//

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "common_types.h"

// This component controls the logic for solar heating and cooling.
class SolarController : public esphome::PollingComponent {
public:
    // singleton access
    static SolarController* getInstance();
    // Constructor
    SolarController();      // singleton constructor

    // --- PUBLIC GETTERS (Read-Only Access) ---
    // Anyone can call these methods to get the current sensor pointers.
    esphome::binary_sensor::BinarySensor* get_solarFlowSensor() const { return solarFlowSensor_; }

    // --- PUBLIC SETTERS (Needed for ESPHome Setup) ---
    // These allow the ESPHome framework to link the sensors during startup.
    void set_solarFlowSensor(esphome::binary_sensor::BinarySensor *s) { solarFlowSensor_ = s; }

    // Method declarations
    void setup() override;
    void update() override;

    // this component needs to know if the spa is in use to manage the solar
    void set_spa_mode(bool spa_mode) { spa_mode_ = spa_mode; }
    void set_estimated_water_temp(float temp) { estimated_water_temp_ = temp; };
    void set_panel_temperature(float temp) { panel_temperature_ = temp; };
    void set_spa_target_temp(float temp) { spa_target_temp_ = temp; };
    void set_pool_target_temp(float temp) { pool_target_temp_ = temp; };
    void set_pool_cooling_target(float temp) { pool_cooling_target_ = temp; };

private:
    static inline SolarController* instance_{nullptr};  // pointer to instance

    // --- SENSOR MEMBERS ---
    // Public pointers to the sensor objects that we will manage.
    // solarFlowSensor indicates if water should be circulating up to the solar panels
    esphome::binary_sensor::BinarySensor* solarFlowSensor_{nullptr};

    // Type declarations
    enum class FlowStates { unknown, idle, flowing };

    // track desired solar heat state
    //
    MilliSec msDesiredFlowStateChanged{0};       // millis() time at desired flow change
    FlowStates desiredFlowState{FlowStates::unknown};

    // track solar heat state
    //
    MilliSec msCurrentFlowStateChanged{0};       // millis() time of last state change
    FlowStates currentFlowState{FlowStates::unknown};

    // track current switch and sensor readings
    //
    MilliSec msMissingDataStarted{0};            // millis() time of first missing data event

    // track current values from other components
    bool spa_mode_{false};
    float estimated_water_temp_{NAN};
    float panel_temperature_{NAN};
    float spa_target_temp_{NAN};
    float pool_target_temp_{NAN};
    float pool_cooling_target_{NAN};

    // constants
    //
    static constexpr float SPA_TARGET_TOLERANCE{0.5};  // in ºF, how much to over/undershoot target
    static constexpr float POOL_TARGET_TOLERANCE{0.3}; // in ºF, how much to over/undershoot target

    // PANEL_START_OFFSET - in ºF, how many degrees panels must be > water
    // temp in order to turn on solar
    static constexpr float PANEL_START_OFFSET{6.0};

    // PANEL_STOP_OFFSET - in ºF, how many degrees panel must be > water temp
    // in order to keep solar on, if panels drop below this temp, solar turns off
    static constexpr float PANEL_STOP_OFFSET{1.0};

    static constexpr int SolarControllerPollIntervalSecs{30}; // seconds, how often to evaluate
    static constexpr int DataMissingTimeOutSecs{60};          // seconds, timeout waiting for data
    static constexpr int MinimumDesiredChangeIntervalSecs{5*60}; // seconds, don't change solar more often, to avoid flipping the diverter valve back and forth (it takes 24s to change)

    // private method declarations
    void setSolarFlowState(FlowStates newState);
    FlowStates evaluateCooling(float targetTempF, float waterTempF, float panelTempF) const;
    FlowStates evaluateHeat(float targetTempF, float waterTempF, float panelTempF) const;
};

