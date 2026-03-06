"""
Flight Data Visualization
Plots flight data collected from SITL simulation.
"""

import os
import math
from typing import List, Optional

try:
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend for CI
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("Warning: matplotlib not available, plotting disabled")


def plot_flight_data(data: List[dict], title: str = "Flight Data",
                     save_path: Optional[str] = None) -> None:
    """
    Plot flight data with attitude, altitude, and motor outputs.
    
    Args:
        data: List of flight data records (dicts)
        title: Plot title
        save_path: Optional file path to save the plot
    """
    if not HAS_MATPLOTLIB:
        print("Plotting skipped - matplotlib not available")
        return

    if not data:
        print("No data to plot")
        return

    # Extract data series
    t = [d.get('timestamp_ms', 0) / 1000.0 for d in data]  # Convert to seconds
    roll = [d.get('roll', 0) * 180.0 / math.pi for d in data]
    pitch = [d.get('pitch', 0) * 180.0 / math.pi for d in data]
    yaw = [d.get('yaw', 0) * 180.0 / math.pi for d in data]
    alt = [d.get('altitude', 0) for d in data]
    vz = [d.get('vz', 0) for d in data]
    m0 = [d.get('m0', 0) for d in data]
    m1 = [d.get('m1', 0) for d in data]
    m2 = [d.get('m2', 0) for d in data]
    m3 = [d.get('m3', 0) for d in data]

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(title, fontsize=14, fontweight='bold')

    # Attitude
    axes[0].plot(t, roll, 'r-', label='Roll', linewidth=0.8)
    axes[0].plot(t, pitch, 'g-', label='Pitch', linewidth=0.8)
    axes[0].plot(t, yaw, 'b-', label='Yaw', linewidth=0.8)
    axes[0].set_ylabel('Angle (°)')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('Attitude')

    # Altitude
    axes[1].plot(t, alt, 'b-', linewidth=0.8)
    axes[1].set_ylabel('Altitude (m)')
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title('Altitude')

    # Vertical velocity
    axes[2].plot(t, vz, 'r-', linewidth=0.8)
    axes[2].set_ylabel('Vz (m/s)')
    axes[2].grid(True, alpha=0.3)
    axes[2].set_title('Vertical Velocity')

    # Motor outputs
    axes[3].plot(t, m0, label='M0', linewidth=0.8)
    axes[3].plot(t, m1, label='M1', linewidth=0.8)
    axes[3].plot(t, m2, label='M2', linewidth=0.8)
    axes[3].plot(t, m3, label='M3', linewidth=0.8)
    axes[3].set_ylabel('Throttle')
    axes[3].set_xlabel('Time (s)')
    axes[3].legend(loc='upper right')
    axes[3].grid(True, alpha=0.3)
    axes[3].set_title('Motor Outputs')
    axes[3].set_ylim(-0.05, 1.05)

    plt.tight_layout()

    if save_path:
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Plot saved to: {save_path}")

    plt.close(fig)


if __name__ == "__main__":
    # Example: read CSV file and plot
    import csv
    import sys

    if len(sys.argv) < 2:
        print("Usage: python plot_results.py <csv_file> [output_png]")
        sys.exit(1)

    csv_file = sys.argv[1]
    save_path = sys.argv[2] if len(sys.argv) > 2 else None

    data = []
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            record = {k: float(v) for k, v in row.items()}
            data.append(record)

    plot_flight_data(data, title=f"Flight Data: {csv_file}", save_path=save_path)
