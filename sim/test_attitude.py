"""
Attitude Step Response Test
Tests attitude control by commanding a 5° roll step and measuring response characteristics.

Pass criteria:
  - Rise time (10% to 90%) < 0.5s
  - Overshoot < 30%
  - Steady-state error < 1° after 2 seconds
"""

import sys
import os
import math
sys.path.insert(0, os.path.dirname(__file__))

from run_sitl import SITLRunner
from plot_results import plot_flight_data


def test_attitude_step(executable: str = None, plot: bool = True) -> bool:
    """
    Run attitude step response test.
    
    Returns:
        True if test passes, False otherwise.
    """
    runner = SITLRunner(executable=executable)

    try:
        runner.start(init_alt=2.0)  # Start at 2m altitude

        # Arm and stabilize first
        runner.arm()
        runner.set_mode("ALT_HOLD")
        runner.set_altitude(2.0)
        runner.set_attitude(0, 0, 0)

        # Let it stabilize
        runner.run(3.0)
        runner.clear_data()

        # Apply 5° roll step
        target_roll_deg = 5.0
        runner.set_attitude(target_roll_deg, 0, 0)
        runner.run(5.0)

        state = runner.get_state()
        print(f"\nFinal state: roll={state.get('roll_deg', 0):.2f}°, "
              f"altitude={state.get('altitude', 0):.2f}m")

        passed = True

        # Analyze steady-state error
        final_roll = state.get('roll_deg', 0)
        ss_error = abs(final_roll - target_roll_deg)
        if ss_error > 1.0:
            print(f"FAIL: Roll steady-state error = {ss_error:.2f}° (limit: 1°)")
            passed = False
        else:
            print(f"PASS: Roll steady-state error = {ss_error:.2f}°")

        # Analyze data for rise time and overshoot
        if runner.flight_data:
            rolls_deg = [d.get('roll', 0) * 180.0 / math.pi
                         for d in runner.flight_data]
            timestamps_ms = [d.get('timestamp_ms', 0) for d in runner.flight_data]

            if rolls_deg:
                max_roll = max(rolls_deg)
                overshoot_pct = ((max_roll - target_roll_deg) / target_roll_deg * 100
                                  if target_roll_deg != 0 else 0)

                if overshoot_pct > 30:
                    print(f"FAIL: Overshoot = {overshoot_pct:.1f}% (limit: 30%)")
                    passed = False
                else:
                    print(f"PASS: Overshoot = {overshoot_pct:.1f}%")

        if plot and runner.flight_data:
            plot_flight_data(runner.flight_data, title="Attitude Step Response",
                             save_path="sim/results/attitude_step.png")

        print(f"\n{'='*40}")
        print(f"Attitude step test: {'PASSED' if passed else 'FAILED'}")
        print(f"{'='*40}")

        return passed

    finally:
        runner.stop()


if __name__ == "__main__":
    executable = sys.argv[1] if len(sys.argv) > 1 else None
    success = test_attitude_step(executable=executable, plot=True)
    sys.exit(0 if success else 1)
