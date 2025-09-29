//
//  solar_controller.h
//  Pool Controller
//
//  Created by Gary Morris on 2025-09-26.
//  Refactored by Gemini AI 2025-09-26
//

#pragma once

#include "esphome.h"
#include "common_types.h"

// This component controls the logic for solar heating and cooling.
class SolarController : public PollingComponent {
public:
    // singleton access
    static SolarController* getInstance();

    // --- SENSOR MEMBERS ---
    // Public pointers to the sensor objects that we will manage.
    // solarFlowSensor indicates if water should be circulating up to the solar panels
    BinarySensor* solarFlowSensor{nullptr};

    // Method declarations
    void setup() override;
    void update() override;

private:
    // Constructor
    SolarController();      // singleton constructor
    static SolarController* instance{nullptr};

    // Type declarations
    enum class SolarFlowStates { unknown, idle, flowing };

    // track desired solar heat state
    //
    MilliSec msDesiredFlowStateChanged = 0;
    SolarFlowStates desiredFlowState = unknown;

    // track solar heat state
    //
    MilliSec msCurrentFlowStateChanged = 0;       // millis() time of last state change
    SolarFlowStates currentFlowState = unknown;

    // track current switch and sensor readings
    //
    MilliSec msMissingDataStarted = 0;          // millis() time of first missing data event

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
    void setSolarFlowState(SolarFlowStates newState);
    SolarFlowStates evaluateCooling(bool spaMode, float targetTempF,
                                    float waterTempF, float panelTempF) const;
    SolarFlowStates evaluateHeat(bool spaMode, float targetTempF,
                                 float waterTempF, float panelTempF) const;
};

