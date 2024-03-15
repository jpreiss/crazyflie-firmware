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


def test_gaps_eta_zero_noop():

    ctrl = cffirmware.controllerMellinger_t()

    cffirmware.controllerMellingerInit(ctrl)

    setpoint = cffirmware.setpoint_t()
    setpoint.mode.x = cffirmware.modeAbs
    setpoint.mode.y = cffirmware.modeAbs
    setpoint.mode.z = cffirmware.modeAbs
    setpoint.mode.yaw = cffirmware.modeAbs
    setpoint.position = a2v(np.zeros(3))
    setpoint.yaw = 0
    setpoint.velocity = a2v(np.zeros(3))
    setpoint.attitudeRate = a2att(np.zeros(3))

    state = cffirmware.state_t()
    state.position = a2v(np.zeros(3))
    state.velocity = a2v(np.zeros(3))
    state.attitude = a2att(np.zeros(3))

    sensors = cffirmware.sensorData_t()
    sensors.gyro.x = 0
    sensors.gyro.y = 0
    sensors.gyro.z = 0

    step = 100

    ctrl.gaps_enable = 0
    control_disabled = cffirmware.control_t()
    cffirmware.controllerMellinger(ctrl, control_disabled, setpoint, sensors, state, step)

    ctrl.gaps_enable = 1
    control_enabled = cffirmware.control_t()
    cffirmware.controllerMellinger(ctrl, control_enabled, setpoint, sensors, state, step)

    assert control_equal(control_disabled, control_enabled)

