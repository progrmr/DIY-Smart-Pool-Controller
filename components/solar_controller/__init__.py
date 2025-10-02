# components/solar_controller/__init__.py

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ID

# Define a custom key for our component
CONF_SOLAR_FLOW_SENSOR = "solar_flow_sensor"

SolarController = cg.global_ns.class_('SolarController', cg.PollingComponent)

# The schema now requires the ID of a binary_sensor that has been defined elsewhere.
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(SolarController),
    cv.Required(CONF_SOLAR_FLOW_SENSOR): cv.use_id(binary_sensor.BinarySensor),
}).extend(cv.polling_component_schema('30s'))


async def to_code(config):
    """Generate the C++ code for this component."""
    # var = cg.add_global(SolarController.getInstance())
    # var = cg.add_global(cg.RawExpression(f"{SolarController}::getInstance()"))
    # var = cg.RawExpression(f"{SolarController}::getInstance()")
    # var = cg.add_global(SolarController.operator("ptr"), cg.RawExpression(f"{SolarController}::getInstance()"))
    # if var is None:
    #    return

    # Use the standard ESPHome way to create the object
    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(var, config)

    # Get the pre-existing sensor and link it
    sens = await cg.get_variable(config[CONF_SOLAR_FLOW_SENSOR])
    cg.add(var.set_solarFlowSensor(sens))
