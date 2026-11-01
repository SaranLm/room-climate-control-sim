"""First-order thermal model of a single room (the plant).

    dT/dt = (T_out - T) / tau + gain * u

T      room temperature (deg C)
T_out  outside temperature (deg C)
tau    thermal time constant of the room (s): how slowly it drifts toward the outside temperature
gain   temperature rise per second at full heating (deg C / s)
u      actuator command in [-1, 1]: +1 full heating, -1 full cooling
"""
from dataclasses import dataclass


@dataclass
class RoomModel:
    temp: float = 18.0
    tau: float = 3600.0
    gain: float = 0.01

    def step(self, u: float, t_out: float, dt: float) -> float:
        u = max(-1.0, min(1.0, u))
        self.temp += ((t_out - self.temp) / self.tau + self.gain * u) * dt
        return self.temp

    def steady_state_command(self, setpoint: float, t_out: float) -> float:
        """Command that holds `setpoint` exactly against heat loss to the outside."""
        return (setpoint - t_out) / (self.tau * self.gain)
