# components/valve_actuator/__init__.py

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, text_sensor
from esphome.const import CONF_ID

# Define these as custom keys instead of importing them
CONF_PEAK_CURRENT = "peak_current"
CONF_ACTUATION_TIME = "actuation_time"
CONF_VALVE_POSITION = "valve_position"

ValveActuator = cg.global_ns.class_("ValveActuator", cg.Component)

# Use a more specific key to avoid conflicts with other components
CONF_VALVE_POSITION = "valve_position"

# The schema now requires the IDs of sensors that have already been defined elsewhere.
# This completely decouples the validation logic.
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(ValveActuator),
    cv.Required(CONF_PEAK_CURRENT): cv.use_id(sensor.Sensor),
    cv.Required(CONF_ACTUATION_TIME): cv.use_id(sensor.Sensor),
    cv.Required(CONF_VALVE_POSITION): cv.use_id(text_sensor.TextSensor),
})

async def to_code(config):
    """Generate the C++ code for this component."""
    # var = cg.add_global(ValveActuator.getInstance())
    # var = cg.add_global(cg.RawExpression(f"{ValveActuator}::getInstance()"))
    # var = cg.RawExpression(f"{ValveActuator}::getInstance()")
    # var = cg.add_global(ValveActuator.operator("ptr"), cg.RawExpression(f"{ValveActuator}::getInstance()"))
    # if var is None:
    #    return
    
    # Use the standard ESPHome way to create the object
    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(var, config)

    peak_current_sens = await cg.get_variable(config[CONF_PEAK_CURRENT])
    cg.add(var.set_peakCurrentSensor(peak_current_sens))

    actuation_time_sens = await cg.get_variable(config[CONF_ACTUATION_TIME])
    cg.add(var.set_actuationTimeSensor(actuation_time_sens))

    valve_pos_sens = await cg.get_variable(config[CONF_VALVE_POSITION])
    cg.add(var.set_valvePositionSensor(valve_pos_sens))
