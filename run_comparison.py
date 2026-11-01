"""Compare on/off and PID control of a room and write a CSV + plot to ./output."""
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from hvac.controllers import OnOffController, PIDController  # noqa: E402
from hvac.simulate import Scenario, compute_metrics, simulate  # noqa: E402

OUT = Path("output")


def main() -> None:
    OUT.mkdir(exist_ok=True)
    sc = Scenario(noise_std=0.05)
    runs = {
        "On/off (+/-0.5 C)": simulate(OnOffController(0.5), sc),
        "PID": simulate(PIDController(kp=0.5, ki=0.0004, kd=0.0), sc),
    }

    print(f"{'controller':<20}{'rise (min)':>11}{'overshoot %':>13}{'settle (min)':>14}"
          f"{'SS err (C)':>12}{'IAE':>10}{'energy':>9}{'switches':>10}")
    for name, samples in runs.items():
        m = compute_metrics(samples, sc.setpoint, sc.initial_temp)
        rise = f"{m.rise_time_s / 60:.1f}" if m.rise_time_s is not None else "n/a"
        settle = f"{m.settling_time_s / 60:.1f}" if m.settling_time_s is not None else "n/a"
        print(f"{name:<20}{rise:>11}{m.overshoot_pct:>13.1f}{settle:>14}"
              f"{m.steady_state_error:>12.3f}{m.iae:>10.0f}{m.energy:>9.0f}{m.switches:>10}")

    with open(OUT / "comparison.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["controller", "t_s", "temp_c", "command", "t_out_c"])
        for name, samples in runs.items():
            for s in samples[::6]:  # one row per minute
                w.writerow([name, s.t, round(s.temp, 3), round(s.command, 3), s.t_out])

    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    for name, samples in runs.items():
        a1.plot([s.t / 3600 for s in samples], [s.temp for s in samples], label=name)
        a2.plot([s.t / 3600 for s in samples], [s.command for s in samples], label=name, linewidth=0.8)
    a1.axhline(sc.setpoint, color="grey", linestyle="--", linewidth=0.8)
    a1.set_ylabel("Room temperature (C)")
    a1.legend()
    a2.set_ylabel("Heater command")
    a2.set_xlabel("Time (h)")
    a1.set_title("Room heating: on/off vs PID (outside temperature drops at t = 2 h)")
    fig.tight_layout()
    fig.savefig(OUT / "comparison.png", dpi=120)
    print(f"wrote {OUT / 'comparison.csv'} and {OUT / 'comparison.png'}")


if __name__ == "__main__":
    main()
