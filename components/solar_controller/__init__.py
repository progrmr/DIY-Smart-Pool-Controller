# components/solar_controller/__init__.py

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ID

DEPENDENCIES = ['time']

# The C++ class is in the global namespace
SolarController = cg.global_ns.class_('SolarController', cg.PollingComponent)

CONF_SOLAR_FLOW = 'solar_flow'

CONFIG_SCHEMA = cv.Schema({
    # We no longer need a separate GenerateID here, as add_global handles it.
    cv.Required(CONF_SOLAR_FLOW): binary_sensor.binary_sensor_schema(),
}).extend(cv.polling_component_schema('30s'))


async def to_code(config):
    """Generate the C++ code for this component."""
    # This single line replaces the manual variable creation and include.
    # It gets the singleton instance and registers it as a global variable.
    var = cg.add_global(SolarController.getInstance())
    if var is None:
        return  # <-- This is the correct check

    # Register it as a component so its setup() and loop() are called.
    await cg.register_component(var, config)

    # The sensor linking logic remains the same.
    sens = await binary_sensor.new_binary_sensor(config[CONF_SOLAR_FLOW])
    cg.add(var.set_solarFlowSensor(sens))
