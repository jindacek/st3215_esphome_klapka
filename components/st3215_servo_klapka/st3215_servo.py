import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, switch, text_sensor
from esphome.const import CONF_ID, CONF_UART_ID, UNIT_DEGREES, UNIT_PERCENT
from esphome.components import binary_sensor


DEPENDENCIES = ["uart", "sensor", "switch", "binary_sensor", "text_sensor"]
AUTO_LOAD = ["sensor", "switch", "binary_sensor", "text_sensor"]

st3215_ns = cg.esphome_ns.namespace("st3215_servo_klapka")
St3215Servo = st3215_ns.class_("St3215Servo", cg.Component, uart.UARTDevice)
St3215TorqueSwitch = st3215_ns.class_("St3215TorqueSwitch", switch.Switch, cg.Component)
St3215AutoUnlockSwitch = st3215_ns.class_("St3215AutoUnlockSwitch", switch.Switch, cg.Component)

CONF_SERVO_ID = "servo_id"
CONF_TURNS_FULL_OPEN = "turns_full_open"
CONF_MAX_ANGLE = "max_angle"
CONF_TORQUE_SWITCH = "torque_switch"
CONF_AUTO_UNLOCK_SWITCH = "auto_unlock_switch"
CONF_POSITION_SPEED = "position_speed"
CONF_MIN_CALIB_SPAN_TURNS = "min_calib_span_turns"


# 👉 NOVÉ:
CONF_INVERT_DIRECTION = "invert_direction"
CONF_STATE_SENSOR = "state_sensor"
CONF_MAX_RUN_TIME_MS = "max_run_time_ms"
CONF_STALL_TIMEOUT_MS = "stall_timeout_ms"
CONF_STALL_GRACE_MS = "stall_grace_ms"
CONF_STALL_MIN_DELTA_TURNS = "stall_min_delta_turns"

_SERVO_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(St3215Servo),
        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Required(CONF_SERVO_ID): cv.int_range(min=0, max=100),
        cv.Optional(CONF_MAX_ANGLE, default=240.0): cv.float_,
        cv.Optional(CONF_TURNS_FULL_OPEN, default=0.0): cv.float_,

        # 👉 NOVÉ: invertace směru
        cv.Optional(CONF_INVERT_DIRECTION, default=False): cv.boolean,
        cv.Optional("ramp_factor", default=1.0): cv.float_range(min=0.5, max=3.5),
        cv.Optional(CONF_POSITION_SPEED, default=800): cv.int_range(min=100, max=3000),
        cv.Optional(CONF_MIN_CALIB_SPAN_TURNS, default=0.02): cv.float_range(min=0.005, max=1.0),
        cv.Optional(CONF_MAX_RUN_TIME_MS, default=35000): cv.positive_int,
        cv.Optional(CONF_STALL_TIMEOUT_MS, default=1500): cv.positive_int,
        cv.Optional(CONF_STALL_GRACE_MS, default=900): cv.positive_int,
        cv.Optional(CONF_STALL_MIN_DELTA_TURNS, default=0.010): cv.float_range(min=0.001, max=0.200),
        cv.Optional("angle"): sensor.sensor_schema(unit_of_measurement=UNIT_DEGREES),
        cv.Optional("turns"): sensor.sensor_schema(),
        cv.Optional("percent"): sensor.sensor_schema(unit_of_measurement=UNIT_PERCENT),
        cv.Optional("calib_state"): sensor.sensor_schema(),
        cv.Optional(CONF_TORQUE_SWITCH): switch.switch_schema(St3215TorqueSwitch),
        cv.Optional(CONF_AUTO_UNLOCK_SWITCH): switch.switch_schema(St3215AutoUnlockSwitch),
        cv.Optional("torque_state"): binary_sensor.binary_sensor_schema(device_class="lock"),
        cv.Optional(CONF_STATE_SENSOR): text_sensor.text_sensor_schema(),
    }
).extend(uart.UART_DEVICE_SCHEMA)

CONFIG_SCHEMA = cv.All(cv.ensure_list(_SERVO_SCHEMA))


async def to_code(config):
    for conf in config:
        var = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(var, conf)
        await uart.register_uart_device(var, conf)

        cg.add(var.set_servo_id(conf[CONF_SERVO_ID]))
        cg.add(var.set_max_angle(conf[CONF_MAX_ANGLE]))
        cg.add(var.set_turns_full_open(conf[CONF_TURNS_FULL_OPEN]))

        # 👉 NOVÉ:
        cg.add(var.set_invert_direction(conf[CONF_INVERT_DIRECTION]))
        if "ramp_factor" in conf:
            cg.add(var.set_ramp_factor(conf["ramp_factor"]))

        cg.add(var.set_max_run_time_ms(conf[CONF_MAX_RUN_TIME_MS]))
        cg.add(var.set_stall_timeout_ms(conf[CONF_STALL_TIMEOUT_MS]))
        cg.add(var.set_stall_grace_ms(conf[CONF_STALL_GRACE_MS]))
        cg.add(var.set_stall_min_delta_turns(conf[CONF_STALL_MIN_DELTA_TURNS]))
        cg.add(var.set_position_speed(conf[CONF_POSITION_SPEED]))
        cg.add(var.set_min_calib_span_turns(conf[CONF_MIN_CALIB_SPAN_TURNS]))
        
        if "angle" in conf:
            sens = await sensor.new_sensor(conf["angle"])
            cg.add(var.set_angle_sensor(sens))

        if "turns" in conf:
            sens = await sensor.new_sensor(conf["turns"])
            cg.add(var.set_turns_sensor(sens))

        if "percent" in conf:
            sens = await sensor.new_sensor(conf["percent"])
            cg.add(var.set_percent_sensor(sens))

        if "calib_state" in conf:
            sens = await sensor.new_sensor(conf["calib_state"])
            cg.add(var.set_calib_state_sensor(sens))

        if CONF_STATE_SENSOR in conf:
            ts = await text_sensor.new_text_sensor(conf[CONF_STATE_SENSOR])
            cg.add(var.set_state_sensor(ts))        

        if CONF_TORQUE_SWITCH in conf:
            sw = await switch.new_switch(conf[CONF_TORQUE_SWITCH])
            await cg.register_component(sw, conf[CONF_TORQUE_SWITCH])
            cg.add(sw.set_parent(var))
            cg.add(var.set_torque_switch(sw))
        if CONF_AUTO_UNLOCK_SWITCH in conf:
            sw2 = await switch.new_switch(conf[CONF_AUTO_UNLOCK_SWITCH])
            await cg.register_component(sw2, conf[CONF_AUTO_UNLOCK_SWITCH])
            cg.add(sw2.set_parent(var))
            cg.add(var.set_auto_unlock_switch(sw2))
        if "torque_state" in conf:
            sens = await binary_sensor.new_binary_sensor(conf["torque_state"])
            cg.add(var.set_torque_state_sensor(sens))
