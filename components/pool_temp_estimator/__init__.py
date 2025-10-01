"""
ESPHome component for the PoolTempEstimator custom C++ class.

This component integrates the PoolTempEstimator, which estimates pool water
temperature when direct readings are unavailable. It exposes the estimated
temperature as a sensor and provides actions to update it with real-time
panel and water temperature data from other sensors.
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
# **** FIX: Import the generic Action class ****
from esphome.automation import Action, build_action, register_action
from esphome.const import (
    CONF_ID,
    CONF_TEMPERATURE,
)

# This component relies on the sensor component.
DEPENDENCIES = ["sensor"]

# Create a namespace for the C++ code.
pool_temp_estimator_ns = cg.esphome_ns.namespace("esphome")

# Get a reference to the C++ class defined in your .h file.
PoolTempEstimator = pool_temp_estimator_ns.class_("PoolTempEstimator", cg.PollingComponent)

# --- Configuration Keys ---
CONF_ESTIMATED_TEMPERATURE = "estimated_temperature"

# --- Component Configuration Schema ---
CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(PoolTempEstimator),
            cv.Required(CONF_ESTIMATED_TEMPERATURE): sensor.sensor_schema(
                icon="mdi:thermometer-water",
                accuracy_decimals=1,
                device_class="temperature",
                state_class="measurement",
            ),
        }
    ).extend(cv.polling_component_schema("60s"))
)

# --- Code Generation ---
async def to_code(config):
    """Generate the C++ code for the PoolTempEstimator component."""
    var = cg.add_global(PoolTempEstimator.getInstance())
    if var is None:
        return  # <-- This is the correct check

    await cg.register_component(var, config)

    if conf_sensor := config.get(CONF_ESTIMATED_TEMPERATURE):
        s = await sensor.new_sensor(conf_sensor)
        cg.add(var.set_estimatedTempSensor(s))


# --- Actions ---

# Define a common schema for both actions to avoid repetition.
SET_TEMPERATURE_ACTION_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_ID): cv.use_id(PoolTempEstimator),
        cv.Required(CONF_TEMPERATURE): cv.templatable(cv.float_),
    }
)

# Action to set the Panel Temperature
@register_action(
    "pool_temp_estimator.set_panel_temp",
    Action,  # **** FIX: Provide the generic Action class ****
    SET_TEMPERATURE_ACTION_SCHEMA,
)
async def set_panel_temp_action_to_code(config, action_id, template_arg, args):
    """Generate C++ code for the set_panel_temp action."""
    paren = await cg.get_variable(config[CONF_ID])
    action = build_action(action_id, template_arg, args)
    temp_val = await cg.templatable(config[CONF_TEMPERATURE], args, float)
    cg.add(action.add_body(f"{paren}->setPanelTempC({temp_val});"))
    return action


# Action to set the Water Temperature
@register_action(
    "pool_temp_estimator.set_water_temp",
    Action,  # **** FIX: Provide the generic Action class ****
    SET_TEMPERATURE_ACTION_SCHEMA,
)
async def set_water_temp_action_to_code(config, action_id, template_arg, args):
    """Generate C++ code for the set_water_temp action."""
    paren = await cg.get_variable(config[CONF_ID])
    action = build_action(action_id, template_arg, args)
    temp_val = await cg.templatable(config[CONF_TEMPERATURE], args, float)
    cg.add(action.add_body(f"{paren}->setWaterTempC({temp_val});"))
    return action
