//
//  solar_controller.h
//  Pool Controller
//
//  Created by Gary Morris on 2025-09-26.
//  Refactored by Gemini AI 2025-09-26
//

#pragma once

#include "esphome.h"
#include <string>

// --- Constants ---
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
#define MaxWaterTempAgeSeconds (300)
#define TemperatureUpdateIntervalSeconds (60)
#define StdTempDropFPH (1.5 / 6.5)
#define StdTempDropFPH2 (4.2 / 13.0)
#define TempEstimatePollingInterval (60)

#define MinimumValveChangeIntervalSecs (5*60)  // seconds, don't change valve unless this time has passed
#define MinimumDesiredChangeIntervalSecs (5*60)  // seconds, don't change states unless the desired state has been constant this long

// --- Type Definitions ---
using MilliSec = unsigned long;

// --- Class Declarations ---

// This component controls the logic for solar heating and cooling.
class SolarController : public PollingComponent, public BinarySensor {
public:
    static SolarController *instance;

    // An enumeration to represent the state of solar heating.
    enum SolarHeatStates { unknown, solarDisabled, solarEnabled };

    // State tracking variables

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

    // Constructor
    SolarController();

    // Method declarations
    void setup() override;
    void update() override;
    void setSolarHeatState(SolarHeatStates newState);
    SolarHeatStates evaluateCooling(bool spaMode, float targetTempF, float waterTempF, float panelTempF);
    SolarHeatStates evaluateHeat(bool spaMode, float targetTempF, float waterTempF, float panelTempF);
    void setValvePosition(std::string newPosition);
    float CtoF(float centigrade);
};

// This component estimates water temperature when direct readings are unavailable.
class TemperatureEstimater : public PollingComponent, public Sensor {
public:
    static TemperatureEstimater *instance;

    // State tracking variables
    float panelTempC = NAN;
    float lastWaterTempC = NAN;
    MilliSec msLastWaterTemp = 0;
    float lastEstimateC = NAN;
    MilliSec msLastEstimate = 0;
    MilliSec msLastStateUpdate = 0;

    // Constructor
    TemperatureEstimater();

    // Method declarations
    void setup() override;
    void update() override;
    float estimatedTempC(float lastTempC, MilliSec msLastTemp, float panelTempC);
    void setPanelTempC(float newPanelTempC);
    void setWaterTempC(float newWaterTempC);
    float secElapsed(MilliSec now, MilliSec previous);
    float CtoF(float centigrade);
    float FtoC(float fahrenheit);
};
