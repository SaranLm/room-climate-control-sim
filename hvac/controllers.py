"""Controllers share one interface so they can be swapped and compared (polymorphism)."""
from abc import ABC, abstractmethod


class Controller(ABC):
    @abstractmethod
    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        """Return an actuator command in [-1, 1]."""

    def reset(self) -> None:
        pass


class OnOffController(Controller):
    """Thermostat with hysteresis: heater switches on below (sp - band) and off above (sp + band)."""

    def __init__(self, band: float = 0.5) -> None:
        self.band = band
        self.on = False

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        if measurement < setpoint - self.band:
            self.on = True
        elif measurement > setpoint + self.band:
            self.on = False
        return 1.0 if self.on else 0.0

    def reset(self) -> None:
        self.on = False


class PIDController(Controller):
    """PID with output limits, conditional-integration anti-windup and derivative on measurement.

    - Derivative acts on the measurement, not the error, so a setpoint step does not kick the output.
    - Anti-windup: the integrator only accumulates while the output is not saturated
      (or when the error would pull the output back out of saturation).
    """

    def __init__(self, kp: float, ki: float, kd: float = 0.0,
                 out_min: float = -1.0, out_max: float = 1.0, anti_windup: bool = True) -> None:
        if out_min >= out_max:
            raise ValueError("out_min must be below out_max")
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.anti_windup = anti_windup
        self.integral = 0.0
        self._last_meas: float | None = None

    def update(self, setpoint: float, measurement: float, dt: float) -> float:
        if dt <= 0:
            raise ValueError("dt must be positive")
        error = setpoint - measurement
        derivative = 0.0 if self._last_meas is None else -(measurement - self._last_meas) / dt
        self._last_meas = measurement

        candidate = self.integral + self.ki * error * dt
        unsat = self.kp * error + candidate + self.kd * derivative
        out = max(self.out_min, min(self.out_max, unsat))

        saturated = unsat != out
        pushing_deeper = (unsat > self.out_max and error > 0) or (unsat < self.out_min and error < 0)
        if not self.anti_windup or not (saturated and pushing_deeper):
            self.integral = candidate
        return out

    def reset(self) -> None:
        self.integral = 0.0
        self._last_meas = None
