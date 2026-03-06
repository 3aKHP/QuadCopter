"""
Hover Test Scenario
Tests altitude hold stability by commanding the quadcopter to hover at 2 meters.

Pass criteria:
  - Altitude steady-state error < 0.3m after 5 seconds
  - Roll/Pitch deviation < 3° throughout
"""

import sys
import os
import math
sys.path.insert(0, os.path.dirname(__file__))

from run_sitl import SITLRunner
from plot_results import plot_flight_data


def test_hover(executable: str = None, plot: bool = True) -> bool:
    """
    Run hover stability test.
    
    Returns:
        True if test passes, False otherwise.
    """
    runner = SITLRunner(executable=executable)
    
    try:
        runner.start(init_alt=0.5)  # Start slightly above ground
        
        # Arm and set to altitude hold mode
        runner.arm()
        runner.set_mode("ALT_HOLD")
        runner.set_altitude(2.0)
        runner.set_attitude(0, 0, 0)

        # Run simulation for 10 seconds
        runner.clear_data()
        runner.run(10.0)

        state = runner.get_state()
        print(f"\nFinal state: altitude={state.get('altitude', 0):.2f}m, "
              f"roll={state.get('roll_deg', 0):.2f}°, "
              f"pitch={state.get('pitch_deg', 0):.2f}°")

        # Analyze results
        passed = True
        
        # Check final altitude error
        alt_error = abs(state.get('altitude', 0) - 2.0)
        if alt_error > 0.3:
            print(f"FAIL: Altitude error = {alt_error:.3f}m (limit: 0.3m)")
            passed = False
        else:
            print(f"PASS: Altitude error = {alt_error:.3f}m")

        # Check attitude stability (from flight data)
        if runner.flight_data:
            max_roll = max(abs(d.get('roll', 0)) for d in runner.flight_data)
            max_pitch = max(abs(d.get('pitch', 0)) for d in runner.flight_data)
            roll_deg = max_roll * 180.0 / math.pi
            pitch_deg = max_pitch * 180.0 / math.pi

            if roll_deg > 3.0:
                print(f"FAIL: Max roll = {roll_deg:.2f}° (limit: 3°)")
                passed = False
            else:
                print(f"PASS: Max roll = {roll_deg:.2f}°")

            if pitch_deg > 3.0:
                print(f"FAIL: Max pitch = {pitch_deg:.2f}° (limit: 3°)")
                passed = False
            else:
                print(f"PASS: Max pitch = {pitch_deg:.2f}°")

        if plot and runner.flight_data:
            plot_flight_data(runner.flight_data, title="Hover Test",
                             save_path="sim/results/hover_test.png")

        print(f"\n{'='*40}")
        print(f"Hover test: {'PASSED' if passed else 'FAILED'}")
        print(f"{'='*40}")
        
        return passed

    finally:
        runner.stop()


if __name__ == "__main__":
    executable = sys.argv[1] if len(sys.argv) > 1 else None
    success = test_hover(executable=executable, plot=True)
    sys.exit(0 if success else 1)
