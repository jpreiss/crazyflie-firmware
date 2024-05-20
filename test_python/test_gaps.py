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


def control_equal(c1, c2, rtol=0.0, atol=0.0):
    if c1.controlMode != c2.controlMode:
        return False
    if c1.controlMode == cffirmware.controlModeLegacy:
        v1, v2 = [(c.roll, c.pitch, c.yaw, c.thrust) for c in [c1, c2]]
    elif c1.controlMode == cffirmware.controlModeForceTorque:
        v1, v2 = [(c.torqueX, c.torqueY, c.torqueZ, c.thrustSi) for c in [c1, c2]]
    else:
        raise NotImplementedError
    return np.allclose(v1, v2, rtol=rtol, atol=atol)


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


def test_mel_vs_gaps():
    mel = cffirmware.controllerMellinger_t()
    lee = cffirmware.controllerLee_t()

    cffirmware.controllerMellingerInit(mel)
    cffirmware.controllerLeeInit(lee)

    mel.ki_m_xy = 0.0
    mel.ki_m_z = 0.0

    # introduce some error
    setpoint, state, sensors = zero_inputs()
    state.position.x = 0.1
    step = 0

    w_err = cffirmware.mkvec(0.1, 0.2, 0.3)
    lee.prev_w_err = w_err
    mel.prev_omega_roll = w_err.x
    mel.prev_omega_pitch = w_err.y

    ctrl_mel = cffirmware.control_t()
    ctrl_lee = cffirmware.control_t()

    for i in range(100):
        cffirmware.controllerMellinger(
            mel, ctrl_mel, setpoint, sensors, state, step)
        cffirmware.controllerLee(
            lee, ctrl_lee, setpoint, sensors, state, step)
        """ TODO: get these from the codegen version?
        assert np.allclose(
            np.array(mel.z_axis_desired),
            np.array(lee.gaps.debug.z_axis_desired),
            rtol=1e-3
        )
        assert np.allclose(
            np.array(mel.eR),
            np.array(lee.gaps.debug.eR),
            rtol=1e-3
        )
        assert np.allclose(
            np.array(mel.ew),
            np.array(lee.gaps.debug.ew),
            rtol=1e-3
        )
        """
        # because allclose isn't symmetrical
        assert control_equal(ctrl_mel, ctrl_lee, rtol=4e-3)
        assert control_equal(ctrl_lee, ctrl_mel, rtol=4e-3)
