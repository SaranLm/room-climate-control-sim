"""Closed-loop simulation and performance metrics."""
import random
from dataclasses import dataclass
from typing import Callable

from .alarms import Alarm, AlarmMonitor
from .controllers import Controller
from .model import RoomModel


@dataclass
class Sample:
    t: float
    temp: float
    measured: float
    command: float
    t_out: float
    alarms: frozenset


@dataclass
class Scenario:
    setpoint: float = 22.0
    initial_temp: float = 18.0
    duration_s: float = 6 * 3600
    dt: float = 10.0
    noise_std: float = 0.0
    seed: int = 1
    # outside temperature as a function of time (disturbance profile)
    t_out: Callable[[float], float] = lambda t: 5.0 if t < 2 * 3600 else 0.0


def simulate(controller: Controller, scenario: Scenario | None = None,
             alarms: AlarmMonitor | None = None,
             sensor_override: Callable[[float, float], float] | None = None) -> list[Sample]:
    """Run the loop. `sensor_override(t, true_temp)` can inject sensor faults."""
    sc = scenario or Scenario()
    room = RoomModel(temp=sc.initial_temp)
    rng = random.Random(sc.seed)
    controller.reset()
    out: list[Sample] = []
    steps = int(sc.duration_s / sc.dt)
    for i in range(steps):
        t = i * sc.dt
        measured = room.temp + (rng.gauss(0, sc.noise_std) if sc.noise_std else 0.0)
        if sensor_override:
            measured = sensor_override(t, measured)
        active = alarms.update(measured, sc.dt) if alarms else set()
        # fail-safe: with an invalid sensor the heater is switched off rather than driven blindly
        command = 0.0 if Alarm.SENSOR_FAULT in active else controller.update(sc.setpoint, measured, sc.dt)
        t_out = sc.t_out(t)
        out.append(Sample(t, room.temp, measured, command, t_out, frozenset(active)))
        room.step(command, t_out, sc.dt)
    return out


@dataclass
class Metrics:
    rise_time_s: float | None      # time to reach 90% of the initial-to-setpoint change
    overshoot_pct: float
    settling_time_s: float | None  # last time the temperature was outside +/- band
    steady_state_error: float      # mean error over the final 10% of the run
    iae: float                     # integral of absolute error (deg C * s)
    energy: float                  # sum of |command| * dt (full-power seconds)
    switches: int                  # actuator on/off transitions (wear indicator)


def compute_metrics(samples: list[Sample], setpoint: float, initial: float, band: float = 0.5) -> Metrics:
    if not samples:
        raise ValueError("no samples")
    dt = samples[1].t - samples[0].t if len(samples) > 1 else 1.0
    step = setpoint - initial
    target90 = initial + 0.9 * step
    rise = next((s.t for s in samples if (s.temp - target90) * (1 if step > 0 else -1) >= 0), None)

    peak = max(s.temp for s in samples) if step > 0 else min(s.temp for s in samples)
    overshoot = max(0.0, (peak - setpoint) / abs(step) * 100) if step > 0 else max(0.0, (setpoint - peak) / abs(step) * 100)

    outside = [s.t for s in samples if abs(s.temp - setpoint) > band]
    settling = None if outside and outside[-1] == samples[-1].t else (outside[-1] + dt if outside else 0.0)

    tail = samples[int(len(samples) * 0.9):]
    sse = sum(setpoint - s.temp for s in tail) / len(tail)
    iae = sum(abs(setpoint - s.temp) for s in samples) * dt
    energy = sum(abs(s.command) for s in samples) * dt
    switches = sum(1 for a, b in zip(samples, samples[1:]) if (a.command > 0) != (b.command > 0))
    return Metrics(rise, overshoot, settling, sse, iae, energy, switches)
