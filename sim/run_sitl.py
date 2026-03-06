"""
SITL Communication Manager
Launches the SITL executable and provides a Python interface for sending commands
and receiving flight data.
"""

import subprocess
import sys
import os
import time
from pathlib import Path
from typing import Optional, List


class SITLRunner:
    """Manages SITL process lifecycle and communication."""

    def __init__(self, executable: str = None, output_file: str = None):
        """
        Initialize SITL runner.
        
        Args:
            executable: Path to SITL executable. Auto-detected if not specified.
            output_file: Optional CSV file for flight data logging.
        """
        self.executable = executable or self._find_executable()
        self.output_file = output_file
        self.process: Optional[subprocess.Popen] = None
        self.flight_data: List[dict] = []
        self._header: Optional[List[str]] = None

    def _find_executable(self) -> str:
        """Auto-detect SITL executable in build directory."""
        candidates = [
            "build/src/sitl/quadcopter_sitl",
            "build/src/sitl/quadcopter_sitl.exe",
            "build/src/sitl/Debug/quadcopter_sitl.exe",
            "build/src/sitl/Release/quadcopter_sitl.exe",
            "cmake-build-debug/src/sitl/quadcopter_sitl",
            "cmake-build-release/src/sitl/quadcopter_sitl",
        ]
        
        project_root = Path(__file__).parent.parent
        for candidate in candidates:
            path = project_root / candidate
            if path.exists():
                return str(path)
        
        raise FileNotFoundError(
            f"SITL executable not found. Build the project first with CMake.\n"
            f"Searched in: {[str(project_root / c) for c in candidates]}"
        )

    def start(self, init_alt: float = 0.0) -> None:
        """Start the SITL process."""
        cmd = [self.executable]
        if self.output_file:
            cmd.extend(["--output", self.output_file])
        cmd.extend(["--init-alt", str(init_alt)])

        self.process = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1  # Line buffered
        )

        # Wait for READY signal. SITL may emit HEADER/DATA lines before READY.
        timeout_s = 5.0
        deadline = time.time() + timeout_s
        last_line = ""
        while time.time() < deadline:
            line = self.process.stdout.readline()
            if not line:
                if self.process.poll() is not None:
                    raise RuntimeError(
                        f"SITL exited before READY (code={self.process.returncode})")
                continue

            line = line.strip()
            last_line = line
            if line == "READY":
                print("[SITL] Started and ready")
                return
            if line.startswith("HEADER,"):
                self._header = line[7:].split(",")
            elif line.startswith("DATA,"):
                self._parse_data_line(line)
            # Ignore any other stdout lines during startup.

        raise RuntimeError(
            f"SITL did not signal READY within {timeout_s}s, last line: {last_line}")

    def stop(self) -> None:
        """Stop the SITL process."""
        if self.process:
            try:
                if self.process.poll() is None:
                    try:
                        self._send("CMD QUIT")
                    except Exception:
                        # Fallback when process is unhealthy or pipe is broken.
                        self.process.terminate()

                    try:
                        self.process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        self.process.kill()
                        self.process.wait(timeout=2)
            finally:
                self.process = None
                print("[SITL] Stopped")

    def _send(self, command: str) -> str:
        """Send a command and return the response line."""
        if not self.process:
            raise RuntimeError("SITL not started")
        
        self.process.stdin.write(command + "\n")
        self.process.stdin.flush()
        
        # Read response(s) - collect DATA lines and the OK/ERR response
        while True:
            line = self.process.stdout.readline().strip()
            if not line:
                continue
            if line.startswith("HEADER,"):
                self._header = line[7:].split(",")
            elif line.startswith("DATA,"):
                self._parse_data_line(line)
            else:
                return line  # OK or ERR response

    def _parse_data_line(self, line: str) -> None:
        """Parse a DATA line into flight_data list."""
        parts = line[5:].split(",")  # Skip "DATA,"
        if self._header and len(parts) == len(self._header):
            record = {}
            for key, val in zip(self._header, parts):
                try:
                    record[key] = float(val)
                except ValueError:
                    record[key] = val
            self.flight_data.append(record)

    def arm(self) -> str:
        """Arm the flight controller."""
        return self._send("CMD ARM")

    def disarm(self) -> str:
        """Disarm the flight controller."""
        return self._send("CMD DISARM")

    def set_mode(self, mode: str) -> str:
        """Set flight mode (STABILIZE or ALT_HOLD)."""
        return self._send(f"CMD SET_MODE mode={mode}")

    def set_attitude(self, roll: float = 0, pitch: float = 0,
                     yaw_rate: float = 0) -> str:
        """Set attitude target (degrees)."""
        return self._send(
            f"CMD SET_ATTITUDE roll={roll} pitch={pitch} yaw_rate={yaw_rate}")

    def set_altitude(self, alt: float) -> str:
        """Set altitude target (meters)."""
        return self._send(f"CMD SET_ALTITUDE alt={alt}")

    def set_throttle(self, throttle: float) -> str:
        """Set manual throttle (0.0 - 1.0)."""
        return self._send(f"CMD SET_THROTTLE throttle={throttle}")

    def run(self, duration: float) -> str:
        """Run simulation for given duration (seconds)."""
        return self._send(f"CMD RUN duration={duration}")

    def get_state(self) -> dict:
        """Get current vehicle state."""
        response = self._send("CMD STATE")
        if response.startswith("STATE,"):
            parts = response[6:].split(",")
            keys = ["altitude", "roll_deg", "pitch_deg", "yaw_deg", "vz"]
            state = {}
            for key, val in zip(keys, parts):
                state[key] = float(val)
            return state
        return {}

    def clear_data(self) -> None:
        """Clear collected flight data."""
        self.flight_data.clear()


def main():
    """Simple interactive SITL session for manual testing."""
    runner = SITLRunner()
    
    try:
        runner.start(init_alt=0.0)
        
        print("SITL Interactive Mode. Type commands or 'quit' to exit.")
        print("Examples:")
        print("  CMD ARM")
        print("  CMD SET_MODE mode=ALT_HOLD")
        print("  CMD SET_ALTITUDE alt=2.0")
        print("  CMD RUN duration=5.0")
        print("  CMD STATE")
        print("  CMD DISARM")
        print("  CMD QUIT")
        print()

        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break
            
            if line.lower() in ("quit", "exit"):
                break
            if not line:
                continue

            response = runner._send(line)
            print(f"  {response}")

    finally:
        runner.stop()


if __name__ == "__main__":
    main()
