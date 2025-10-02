import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor

# Define a custom key
CONF_ESTIMATED_TEMPERATURE_SENSOR = "estimated_temperature_sensor"

PoolTempEstimator = cg.global_ns.class_("PoolTempEstimator", cg.PollingComponent)

# The schema now requires the ID of a sensor that has been defined elsewhere.
CONFIG_SCHEMA = cv.Schema({
    cv.Required(CONF_ESTIMATED_TEMPERATURE_SENSOR): cv.use_id(sensor.Sensor),
}).extend(cv.polling_component_schema("60s"))


async def to_code(config):
    """Generate the C++ code for the PoolTempEstimator component."""
    var = cg.add_global(PoolTempEstimator.getInstance())
    if var is None:
        return

    await cg.register_component(var, config)

    # Get the pre-existing sensor and link it
    s = await cg.get_variable(config[CONF_ESTIMATED_TEMPERATURE_SENSOR])
    cg.add(var.set_estimatedTempSensor(s))
    
