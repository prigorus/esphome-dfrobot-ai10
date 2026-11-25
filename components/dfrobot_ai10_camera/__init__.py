import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_TRIGGER_ID
from esphome import automation
from esphome.core.entity_helpers import setup_entity
from esphome.components.dfrobot_ai10 import DFRobotAI10Component

DEPENDENCIES = ["dfrobot_ai10"]
AUTO_LOAD = ["camera"]
CONF_AI10_ID = "ai10_id"
CONF_MAX_IMAGE_BYTES = "max_image_bytes"
ns = cg.esphome_ns.namespace("dfrobot_ai10_camera")
AI10SnapshotCamera = ns.class_("AI10SnapshotCamera", cg.Component, cg.EntityBase)
ArchiveRequestTrigger = ns.class_("ArchiveRequestTrigger", automation.Trigger.template(cg.std_string))
CONFIG_SCHEMA = cv.ENTITY_BASE_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(AI10SnapshotCamera),
    cv.Required(CONF_AI10_ID): cv.use_id(DFRobotAI10Component),
    cv.Optional(CONF_MAX_IMAGE_BYTES, default=49152): cv.int_range(min=4096, max=65536),
    cv.Optional("snapshot_resolution", default="240x320"): cv.one_of("240x320", "480x640", "600x800", "1200x1600", lower=True),
    cv.Optional("failed_capture_threshold", default=3): cv.int_range(min=1, max=20),
    cv.Optional("auto_capture_enabled", default=False): cv.boolean,
    cv.Optional("fallback_snapshot", default=True): cv.boolean,
    cv.Optional("archive_retry_interval", default="60s"): cv.positive_time_period_milliseconds,
    cv.Optional("on_archive_request"): automation.validate_automation({cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(ArchiveRequestTrigger)}),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    cg.add_define("USE_CAMERA")
    var = cg.new_Pvariable(config[CONF_ID])
    await setup_entity(var, config, "camera")
    await cg.register_component(var, config)
    parent = await cg.get_variable(config[CONF_AI10_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_max_image_bytes(config[CONF_MAX_IMAGE_BYTES]))
    cg.add(var.set_snapshot_resolution(["240x320", "480x640", "600x800", "1200x1600"].index(config["snapshot_resolution"])))

    cg.add(var.set_failed_capture_threshold(config["failed_capture_threshold"]))
    cg.add(var.set_auto_capture_enabled(config["auto_capture_enabled"]))
    cg.add(var.set_fallback_snapshot(config["fallback_snapshot"]))
    cg.add(var.set_archive_retry_interval(config["archive_retry_interval"]))
    for conf in config.get("on_archive_request", []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(cg.std_string, "token")], conf)
