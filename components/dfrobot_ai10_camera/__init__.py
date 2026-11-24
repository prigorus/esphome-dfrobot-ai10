import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.core.entity_helpers import setup_entity
from esphome.components.dfrobot_ai10 import DFRobotAI10Component

DEPENDENCIES = ["dfrobot_ai10"]
AUTO_LOAD = ["camera"]
CONF_AI10_ID = "ai10_id"
CONF_MAX_IMAGE_BYTES = "max_image_bytes"
ns = cg.esphome_ns.namespace("dfrobot_ai10_camera")
AI10SnapshotCamera = ns.class_("AI10SnapshotCamera", cg.Component, cg.EntityBase)
CONFIG_SCHEMA = cv.ENTITY_BASE_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(AI10SnapshotCamera),
    cv.Required(CONF_AI10_ID): cv.use_id(DFRobotAI10Component),
    cv.Optional(CONF_MAX_IMAGE_BYTES, default=49152): cv.int_range(min=4096, max=65536),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    cg.add_define("USE_CAMERA")
    var = cg.new_Pvariable(config[CONF_ID])
    await setup_entity(var, config, "camera")
    await cg.register_component(var, config)
    parent = await cg.get_variable(config[CONF_AI10_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_max_image_bytes(config[CONF_MAX_IMAGE_BYTES]))
