import math

import pytest

from hvac.alarms import Alarm, AlarmMonitor
from hvac.controllers import OnOffController, PIDController
from hvac.model import RoomModel
from hvac.simulate import Sample, Scenario, compute_metrics, simulate


# ---------------- plant model ----------------
def test_room_drifts_toward_outside_temperature_without_heating():
    room = RoomModel(temp=22.0)
    for _ in range(3600 * 3):                       # 3 hours, dt = 1 s
        room.step(0.0, 0.0, 1.0)
    # after ~3 time constants the room has lost about 95% of the difference
    assert 0.5 < room.temp < 2.0


def test_steady_state_command_holds_temperature():
    room = RoomModel(temp=22.0)
    u = room.steady_state_command(22.0, 5.0)
    for _ in range(600):
        room.step(u, 5.0, 1.0)
    assert room.temp == pytest.approx(22.0, abs=1e-6)


def test_actuator_command_is_clamped():
    a, b = RoomModel(temp=20.0), RoomModel(temp=20.0)
    a.step(5.0, 20.0, 10.0)
    b.step(1.0, 20.0, 10.0)
    assert a.temp == b.temp


# ---------------- controllers ----------------
def test_pid_proportional_only_output():
    pid = PIDController(kp=0.2, ki=0.0)
    assert pid.update(22.0, 20.0, 1.0) == pytest.approx(0.4)


def test_pid_output_is_limited():
    pid = PIDController(kp=10.0, ki=0.0)
    assert pid.update(22.0, 0.0, 1.0) == 1.0
    assert pid.update(0.0, 22.0, 1.0) == -1.0


def test_pid_rejects_bad_arguments():
    with pytest.raises(ValueError):
        PIDController(1, 0, out_min=1, out_max=1)
    with pytest.raises(ValueError):
        PIDController(1, 0).update(1, 1, 0)


def test_derivative_acts_on_measurement_not_setpoint_step():
    pid = PIDController(kp=0.0, ki=0.0, kd=1.0)
    pid.update(20.0, 20.0, 1.0)
    assert pid.update(30.0, 20.0, 1.0) == 0.0       # setpoint jump causes no derivative kick


def test_anti_windup_limits_integrator_during_saturation():
    with_aw, without = PIDController(1.0, 0.01), PIDController(1.0, 0.01, anti_windup=False)
    for _ in range(1000):
        with_aw.update(22.0, 0.0, 1.0)
        without.update(22.0, 0.0, 1.0)
    assert with_aw.integral < 1.0
    assert without.integral > 100.0


def test_anti_windup_gives_faster_recovery_after_saturation():
    sc = Scenario(initial_temp=0.0, duration_s=4 * 3600)
    good = compute_metrics(simulate(PIDController(0.5, 0.002), sc), sc.setpoint, sc.initial_temp)
    bad = compute_metrics(simulate(PIDController(0.5, 0.002, anti_windup=False), sc), sc.setpoint, sc.initial_temp)
    assert good.overshoot_pct < bad.overshoot_pct


def test_on_off_controller_hysteresis():
    c = OnOffController(band=0.5)
    assert c.update(22, 21.0, 1) == 1.0             # below the band: on
    assert c.update(22, 22.2, 1) == 1.0             # inside the band: stays on
    assert c.update(22, 22.6, 1) == 0.0             # above the band: off
    assert c.update(22, 21.8, 1) == 0.0             # inside the band: stays off


# ---------------- closed loop ----------------
def test_pid_reaches_setpoint_with_no_steady_state_error_despite_disturbance():
    sc = Scenario()                                  # outside temperature drops at t = 2 h
    m = compute_metrics(simulate(PIDController(0.5, 0.0004), sc), sc.setpoint, sc.initial_temp)
    assert abs(m.steady_state_error) < 0.05
    assert m.overshoot_pct < 5.0
    assert m.settling_time_s is not None and m.settling_time_s < 30 * 60


def test_pid_switches_far_less_than_on_off():
    sc = Scenario(noise_std=0.05)
    pid = compute_metrics(simulate(PIDController(0.5, 0.0004), sc), sc.setpoint, sc.initial_temp)
    onoff = compute_metrics(simulate(OnOffController(0.5), sc), sc.setpoint, sc.initial_temp)
    assert onoff.switches > 20 * max(pid.switches, 1)


def test_simulation_is_reproducible_for_a_given_seed():
    sc = Scenario(noise_std=0.1, seed=7, duration_s=600)
    a = [s.measured for s in simulate(PIDController(0.5, 0.0004), sc)]
    b = [s.measured for s in simulate(PIDController(0.5, 0.0004), sc)]
    assert a == b


# ---------------- alarms ----------------
def test_high_alarm_requires_the_on_delay():
    mon = AlarmMonitor(high_limit=26.0, on_delay_s=60.0)
    for _ in range(5):                               # 50 s above the limit
        assert Alarm.HIGH_TEMP not in mon.update(27.0, 10.0)
    assert Alarm.HIGH_TEMP in mon.update(27.0, 10.0)  # 60 s reached


def test_short_excursion_does_not_alarm():
    mon = AlarmMonitor(high_limit=26.0, on_delay_s=60.0)
    for _ in range(5):
        mon.update(27.0, 10.0)
    mon.update(25.0, 10.0)                           # back below: timer resets
    for _ in range(5):
        assert Alarm.HIGH_TEMP not in mon.update(27.0, 10.0)


def test_alarm_clears_only_after_hysteresis():
    mon = AlarmMonitor(high_limit=26.0, on_delay_s=10.0, hysteresis=0.5)
    mon.update(27.0, 10.0)
    assert Alarm.HIGH_TEMP in mon.active
    assert Alarm.HIGH_TEMP in mon.update(25.8, 10.0)  # below the limit but inside the hysteresis band
    assert Alarm.HIGH_TEMP not in mon.update(25.4, 10.0)


def test_low_alarm_raises_and_clears():
    mon = AlarmMonitor(low_limit=16.0, on_delay_s=10.0, hysteresis=0.5)
    assert Alarm.LOW_TEMP in mon.update(10.0, 10.0)
    assert Alarm.LOW_TEMP not in mon.update(17.0, 10.0)


@pytest.mark.parametrize("bad", [-100.0, 200.0, math.nan])
def test_out_of_range_or_nan_reading_is_a_sensor_fault(bad):
    assert Alarm.SENSOR_FAULT in AlarmMonitor().update(bad, 1.0)


def test_sensor_fault_drives_heater_to_safe_state_and_recovers():
    sc = Scenario(duration_s=1800)
    broken = lambda t, v: -999.0 if 600 <= t < 900 else v      # sensor fails for 5 minutes
    samples = simulate(PIDController(0.5, 0.0004), sc, alarms=AlarmMonitor(), sensor_override=broken)
    during = [s for s in samples if 600 <= s.t < 900]
    after = [s for s in samples if s.t >= 900]
    assert all(s.command == 0.0 and Alarm.SENSOR_FAULT in s.alarms for s in during)
    assert any(s.command > 0.0 for s in after)
    assert all(Alarm.SENSOR_FAULT not in s.alarms for s in after)


# ---------------- metrics ----------------
def _series(values, dt=10.0):
    return [Sample(i * dt, v, v, 0.0, 0.0, frozenset()) for i, v in enumerate(values)]


def test_metrics_on_a_known_series():
    # step 20 -> 22, peaks at 22.4 (20% overshoot of the 2 C step)
    s = _series([20, 21, 21.9, 22.4, 22.1, 22.0, 22.0, 22.0, 22.0, 22.0])
    m = compute_metrics(s, setpoint=22.0, initial=20.0, band=0.25)
    assert m.rise_time_s == 20.0                       # 21.9 >= 21.8 (90% level) at t = 20 s
    assert m.overshoot_pct == pytest.approx(20.0)
    assert m.settling_time_s == 40.0                   # last sample outside the band is 22.4 at t = 30 s
    assert m.steady_state_error == pytest.approx(0.0)


def test_metrics_report_never_settled():
    s = _series([20, 20.5, 21, 21.2, 21.4, 21.5, 21.5, 21.5, 21.5, 21.5])
    assert compute_metrics(s, 22.0, 20.0, band=0.25).settling_time_s is None


def test_metrics_reject_empty_input():
    with pytest.raises(ValueError):
        compute_metrics([], 22.0, 20.0)
