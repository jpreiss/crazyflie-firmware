#!/usr/bin/env python

import numpy as np

import cffirmware


def a2v(a):
    v = cffirmware.vec3_s()
    v.x = a[0]
    v.y = a[1]
    v.z = a[2]
    return v


def a2att(a):
    att = cffirmware.attitude_t()
    att.roll = a[0]
    att.pitch = a[1]
    att.yaw = a[2]
    return att


def v2a(v):
    return np.array([v.x, v.y, v.z], dtype=np.float32)


def control_equal(c1, c2):
    if c1.controlMode != c2.controlMode:
        return False
    if c1.controlMode == cffirmware.controlModeLegacy:
        return (
            c1.roll == c2.roll
            and c1.pitch == c2.pitch
            and c1.yaw == c2.yaw
            and c1.thrust == c2.thrust
        )
    raise NotImplementedError


def zero_inputs():
    setpoint = cffirmware.setpoint_t()
    setpoint.mode.x = cffirmware.modeAbs
    setpoint.mode.y = cffirmware.modeAbs
    setpoint.mode.z = cffirmware.modeAbs
    setpoint.mode.yaw = cffirmware.modeAbs
    setpoint.position = a2v(np.zeros(3))
    setpoint.yaw = 0
    setpoint.velocity = a2v(np.zeros(3))
    setpoint.attitudeRate = a2att(np.zeros(3))
    setpoint.attitude = a2att(np.zeros(3))

    state = cffirmware.state_t()
    state.position = a2v(np.zeros(3))
    state.velocity = a2v(np.zeros(3))
    state.attitude = a2att(np.zeros(3))

    sensors = cffirmware.sensorData_t()
    sensors.gyro.x = 0
    sensors.gyro.y = 0
    sensors.gyro.z = 0

    return setpoint, state, sensors


def test_gaps_eta_zero_no_error_noop():

    ctrl = cffirmware.controllerMellinger_t()

    cffirmware.controllerMellingerInit(ctrl)

    setpoint, state, sensors = zero_inputs()
    step = 0

    ctrl.gaps_enable = 0
    control_disabled = cffirmware.control_t()
    cffirmware.controllerMellinger(ctrl, control_disabled, setpoint, sensors, state, step)

    cffirmware.controllerMellingerInit(ctrl)
    ctrl.gaps_enable = 1
    control_enabled = cffirmware.control_t()
    for _ in range(100):
        cffirmware.controllerMellinger(ctrl, control_enabled, setpoint, sensors, state, step)

        assert control_equal(control_disabled, control_enabled)


def test_gaps_eta_zero_error_noop():

    ctrl = cffirmware.controllerMellinger_t()
    ctrl_gaps = cffirmware.controllerMellinger_t()
    for c in [ctrl, ctrl_gaps]:
        cffirmware.controllerMellingerInit(c)
    ctrl_gaps.gaps_enable = 1

    # introduce some error
    setpoint, state, sensors = zero_inputs()
    state.position.x = 0.1
    step = 0

    command = cffirmware.control_t()
    command_gaps = cffirmware.control_t()

    for _ in range(100):
        cffirmware.controllerMellinger(
            ctrl, command, setpoint, sensors, state, step)
        cffirmware.controllerMellinger(
            ctrl_gaps, command_gaps, setpoint, sensors, state, step)
        assert control_equal(command, command_gaps)


def test_gaps_eta_nonzero_error_has_effect():

    ctrl = cffirmware.controllerMellinger_t()
    ctrl_gaps = cffirmware.controllerMellinger_t()
    for c in [ctrl, ctrl_gaps]:
        cffirmware.controllerMellingerInit(c)
    ctrl_gaps.gaps_enable = 1
    ctrl_gaps.gaps_eta = 1

    # introduce some error
    setpoint, state, sensors = zero_inputs()
    state.position.x = 0.1
    step = 0

    command = cffirmware.control_t()
    command_gaps = cffirmware.control_t()

    for i in range(100):
        cffirmware.controllerMellinger(
            ctrl, command, setpoint, sensors, state, step)
        cffirmware.controllerMellinger(
            ctrl_gaps, command_gaps, setpoint, sensors, state, step)
        if i == 0:
            assert control_equal(command, command_gaps)
        else:
            assert not control_equal(command, command_gaps)
