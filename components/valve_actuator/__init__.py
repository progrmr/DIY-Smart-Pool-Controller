# components/valve_actuator/__init__.py

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, text_sensor
from esphome.const import (
    CONF_ID,
    STATE_CLASS_MEASUREMENT,
    UNIT_AMPERE,
    UNIT_SECOND,
    ICON_CURRENT_AC,
    ICON_TIMER,
)

# It's better practice to declare the C++ class from the global namespace
# if that's where it truly lives in your .h/.cpp files.
ValveActuator = cg.global_ns.class_("ValveActuator", cg.Component)

CONF_PEAK_CURRENT = "peak_current"
CONF_ACTUATION_TIME = "actuation_time"
CONF_VALVE_POSITION = "valve_position"

CONFIG_SCHEMA = cv.Schema({
    # We can remove cv.GenerateID() as add_global will manage the component's ID.
    cv.Optional(CONF_PEAK_CURRENT): sensor.sensor_schema(
        unit_of_measurement=UNIT_AMPERE,
        icon=ICON_CURRENT_AC,
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_ACTUATION_TIME): sensor.sensor_schema(
        unit_of_measurement=UNIT_SECOND,
        icon=ICON_TIMER,
        accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    cv.Optional(CONF_VALVE_POSITION): text_sensor.text_sensor_schema(
        icon="mdi:valve",
    ),
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    """Generate the C++ code for this component."""
    # Use the modern, high-level helper for singletons.
    # This automatically handles includes and variable declaration.
    var = cg.add_global(ValveActuator.getInstance())
    
    await cg.register_component(var, config)

    if CONF_PEAK_CURRENT in config:
        sens = await sensor.new_sensor(config[CONF_PEAK_CURRENT])
        cg.add(var.set_peakCurrentSensor(sens))

    if CONF_ACTUATION_TIME in config:
        sens = await sensor.new_sensor(config[CONF_ACTUATION_TIME])
        cg.add(var.set_actuationTimeSensor(sens))

    if CONF_VALVE_POSITION in config:
        sens = await text_sensor.new_text_sensor(config[CONF_VALVE_POSITION])
        cg.add(var.set_valvePositionSensor(sens))
