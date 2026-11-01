"""Alarm logic of the kind used in building automation: on-delay, hysteresis and sensor validation."""
from dataclasses import dataclass, field
from enum import Enum


class Alarm(Enum):
    HIGH_TEMP = "HIGH_TEMP"
    LOW_TEMP = "LOW_TEMP"
    SENSOR_FAULT = "SENSOR_FAULT"


@dataclass
class AlarmMonitor:
    high_limit: float = 26.0
    low_limit: float = 16.0
    on_delay_s: float = 60.0        # condition must persist this long before the alarm is raised
    hysteresis: float = 0.5         # must recover by this much before the alarm clears
    sensor_min: float = -40.0       # readings outside this range mean the sensor is faulty
    sensor_max: float = 85.0
    active: set[Alarm] = field(default_factory=set)
    _high_timer: float = 0.0
    _low_timer: float = 0.0

    def update(self, measurement: float, dt: float) -> set[Alarm]:
        if not (self.sensor_min <= measurement <= self.sensor_max) or measurement != measurement:  # NaN check
            self.active = {Alarm.SENSOR_FAULT}
            self._high_timer = self._low_timer = 0.0
            return self.active
        self.active.discard(Alarm.SENSOR_FAULT)

        self._high_timer = self._high_timer + dt if measurement > self.high_limit else 0.0
        self._low_timer = self._low_timer + dt if measurement < self.low_limit else 0.0

        if self._high_timer >= self.on_delay_s:
            self.active.add(Alarm.HIGH_TEMP)
        elif measurement < self.high_limit - self.hysteresis:
            self.active.discard(Alarm.HIGH_TEMP)

        if self._low_timer >= self.on_delay_s:
            self.active.add(Alarm.LOW_TEMP)
        elif measurement > self.low_limit + self.hysteresis:
            self.active.discard(Alarm.LOW_TEMP)
        return self.active
