import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID, CONF_TRIGGER_ID
from esphome import automation

DEPENDENCIES = ["uart"]
AUTO_LOAD = []

CONF_ON_RECOGNIZED = "on_recognized"
CONF_ON_QR_SCANNED = "on_qr_scanned"  # NEU

dfrobot_ai10_ns = cg.esphome_ns.namespace("dfrobot_ai10")
DFRobotAI10Component = dfrobot_ai10_ns.class_(
    "DFRobotAI10Component", cg.Component, uart.UARTDevice
)

# Auth Trigger: Liefert (name, uid)
RecognizedTrigger = dfrobot_ai10_ns.class_(
    "RecognizedTrigger", automation.Trigger.template(cg.std_string, cg.uint16)
)

# NEU: QR Trigger: Liefert (data)
QRScannedTrigger = dfrobot_ai10_ns.class_(
    "QRScannedTrigger", automation.Trigger.template(cg.std_string)
)

DirectionalEnrollmentTrigger = dfrobot_ai10_ns.class_("DirectionalEnrollmentTrigger", automation.Trigger.template(cg.uint8,cg.uint8,cg.uint16,cg.uint8))
EnrollmentSuccessTrigger = dfrobot_ai10_ns.class_("EnrollmentSuccessTrigger", automation.Trigger.template(cg.uint16))
EnrollmentFailedTrigger = dfrobot_ai10_ns.class_("EnrollmentFailedTrigger", automation.Trigger.template(cg.uint8))
RecognitionFailedTrigger = dfrobot_ai10_ns.class_("RecognitionFailedTrigger", automation.Trigger.template(cg.uint8))
CommandResultTrigger = dfrobot_ai10_ns.class_("CommandResultTrigger", automation.Trigger.template(cg.uint8,cg.uint8))
UserInfoTrigger = dfrobot_ai10_ns.class_("UserInfoTrigger", automation.Trigger.template(cg.uint16,cg.std_string,cg.bool_))
UsersReceivedTrigger = dfrobot_ai10_ns.class_("UsersReceivedTrigger", automation.Trigger.template(cg.uint16))

UserDirectoryTrigger = dfrobot_ai10_ns.class_("UserDirectoryTrigger", automation.Trigger.template(cg.uint16))
UserDirectoryFailedTrigger = dfrobot_ai10_ns.class_("UserDirectoryFailedTrigger", automation.Trigger.template(cg.uint8))

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(DFRobotAI10Component),
        cv.Optional("documented_qr", default=False): cv.boolean,
        cv.Optional("recognition_mode", default="eco"): cv.one_of("eco", "full", lower=True),
        cv.Optional("repeat_delay_ms", default=1000): cv.int_range(min=100, max=5000),
        cv.Optional("recognition_recovery", default=False): cv.boolean,
        cv.Optional("fast_capture", default=False): cv.boolean,
        cv.Optional("incremental_directory", default=False): cv.boolean,
        cv.Optional("auto_refresh_users", default=False): cv.boolean,
        cv.Optional("resume_recognition_after_refresh", default=False): cv.boolean,
        cv.Optional("on_user_directory"): automation.validate_automation({cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(UserDirectoryTrigger)}),
        cv.Optional("on_user_directory_failed"): automation.validate_automation({cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(UserDirectoryFailedTrigger)}),
        cv.Optional("allow_sensitive_transfers", default=False): cv.boolean,
        cv.Optional("on_directional_enrollment"): automation.validate_automation({cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(DirectionalEnrollmentTrigger)}),
        cv.Optional("on_enrollment_success"): automation.validate_automation({cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(EnrollmentSuccessTrigger)}),
        cv.Optional("on_enrollment_failed"): automation.validate_automation({cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(EnrollmentFailedTrigger)}),
        cv.Optional("on_recognition_failed"): automation.validate_automation({cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(RecognitionFailedTrigger)}),
        cv.Optional("on_command_result"): automation.validate_automation({cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(CommandResultTrigger)}),
        cv.Optional("on_user_info"): automation.validate_automation({cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(UserInfoTrigger)}),
        cv.Optional("on_users_received"): automation.validate_automation({cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(UsersReceivedTrigger)}),
        
        # Trigger für Gesicht/Hand
        cv.Optional(CONF_ON_RECOGNIZED): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(RecognizedTrigger),
            }
        ),

        # NEU: Trigger für QR-Codes
        cv.Optional(CONF_ON_QR_SCANNED): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(QRScannedTrigger),
            }
        ),
    }
).extend(uart.UART_DEVICE_SCHEMA).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_documented_qr(config['documented_qr']))
    cg.add(var.set_full_scan(config['recognition_mode'] == 'full'))
    cg.add(var.set_repeat_delay(config['repeat_delay_ms']))
    cg.add(var.set_recovery_enabled(config['recognition_recovery']))
    cg.add(var.set_fast_capture(config['fast_capture']))
    cg.add(var.set_incremental_directory(config['incremental_directory']))
    cg.add(var.set_auto_refresh_users(config['auto_refresh_users']))
    cg.add(var.set_resume_recognition_after_refresh(config['resume_recognition_after_refresh']))
    cg.add(var.set_allow_sensitive_transfers(config['allow_sensitive_transfers']))

    for conf in config.get(CONF_ON_RECOGNIZED, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(
            trigger, [(cg.std_string, "name"), (cg.uint16, "uid")], conf
        )

    for conf in config.get(CONF_ON_QR_SCANNED, []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(
            trigger, [(cg.std_string, "qr_data")], conf
        )
    for conf in config.get("on_directional_enrollment", []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(cg.uint8,"step"),(cg.uint8,"mask"),(cg.uint16,"uid"),(cg.uint8,"result")], conf)
    for conf in config.get("on_enrollment_success", []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(cg.uint16, "uid")], conf)
    for conf in config.get("on_enrollment_failed", []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(cg.uint8, "result")], conf)
    for conf in config.get("on_recognition_failed", []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(cg.uint8, "result")], conf)
    for conf in config.get("on_command_result", []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(cg.uint8, "command"), (cg.uint8, "result")], conf)
    for conf in config.get("on_user_info", []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(cg.uint16, "uid"), (cg.std_string, "name"), (cg.bool_, "admin")], conf)
    for conf in config.get("on_users_received", []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(cg.uint16, "count")], conf)

    for conf in config.get("on_user_directory", []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(cg.uint16, "count")], conf)
    for conf in config.get("on_user_directory_failed", []):
        trigger = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
        await automation.build_automation(trigger, [(cg.uint8, "result")], conf)
