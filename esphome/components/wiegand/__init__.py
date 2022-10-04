import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome import pins
from esphome.const import CONF_ID, CONF_ON_TAG, CONF_TRIGGER_ID

CODEOWNERS = ["@stefan-golinschi"]

CONF_DATA_BITS = "data_bits"
CONF_D0_PIN = "d0_pin"
CONF_D1_PIN = "d1_pin"
CONF_WIEGAND_ID = "wiegand_id"

wiegand_ns = cg.esphome_ns.namespace("wiegand")
Wiegand = wiegand_ns.class_("Wiegand", cg.PollingComponent)
WiegandTrigger = wiegand_ns.class_(
    "WiegandTrigger", automation.Trigger.template(cg.uint32)
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Wiegand),
            cv.Required(CONF_D0_PIN): pins.internal_gpio_input_pullup_pin_schema,
            cv.Required(CONF_D1_PIN): pins.internal_gpio_input_pullup_pin_schema,
            cv.Optional(CONF_DATA_BITS, default=24): cv.int_range(min=8, max=34),
            cv.Optional(CONF_ON_TAG): automation.validate_automation(
                {
                    cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(WiegandTrigger),
                }
            ),
        }
    ).extend(cv.polling_component_schema("1s"))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    pin_d0 = await cg.gpio_pin_expression(config[CONF_D0_PIN])
    cg.add(var.set_pin_d0(pin_d0))
    pin_d1 = await cg.gpio_pin_expression(config[CONF_D1_PIN])
    cg.add(var.set_pin_d1(pin_d1))

    for conf in config.get(CONF_ON_TAG, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID])
        cg.add(var.register_trigger(trigger))
        await automation.build_automation(trigger, [(cg.std_string, "x")], conf)
