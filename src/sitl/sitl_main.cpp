/// SITL Main Program - Software-In-The-Loop simulation entry point
/// Runs the flight controller with simulated physics model
/// Communicates with external test scripts via stdin/stdout

#include "physics_model.h"
#include "sitl_imu.h"
#include "sitl_barometer.h"
#include "sitl_motor.h"
#include "sitl_timer.h"
#include "sitl_logger.h"
#include "flight_controller.h"

#include <iostream>
#include <sstream>
#include <string>
#include <cmath>

using namespace fc;

/// Parse a command line from stdin
/// Format: CMD <command> [key=value ...]
struct Command {
    std::string name;
    float roll = 0, pitch = 0, yaw_rate = 0;
    float altitude = 0;
    float throttle = 0;
    float duration = 0;
    std::string mode;
};

Command parse_command(const std::string& line) {
    Command cmd;
    std::istringstream iss(line);
    std::string token;

    // Skip "CMD" prefix
    iss >> token;
    if (token != "CMD") {
        cmd.name = "INVALID";
        return cmd;
    }

    iss >> cmd.name;

    // Parse key=value pairs
    while (iss >> token) {
        auto eq = token.find('=');
        if (eq == std::string::npos) continue;
        std::string key = token.substr(0, eq);
        std::string val = token.substr(eq + 1);
        try {
            if (key == "roll") cmd.roll = std::stof(val);
            else if (key == "pitch") cmd.pitch = std::stof(val);
            else if (key == "yaw_rate") cmd.yaw_rate = std::stof(val);
            else if (key == "alt") cmd.altitude = std::stof(val);
            else if (key == "throttle") cmd.throttle = std::stof(val);
            else if (key == "duration") cmd.duration = std::stof(val);
            else if (key == "mode") cmd.mode = val;
        } catch (...) {
            // Ignore parse errors
        }
    }

    return cmd;
}

int main(int argc, char* argv[]) {
    // Parse optional command line arguments
    std::string data_file = "";
    float sim_dt = 0.001f;          // Physics simulation step: 1 ms
    float ctrl_dt = 0.002f;         // Control loop step: 2 ms (500 Hz)
    float initial_alt = 0.0f;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--output" && i + 1 < argc) {
            data_file = argv[++i];
        } else if (arg == "--sim-dt" && i + 1 < argc) {
            sim_dt = std::stof(argv[++i]);
        } else if (arg == "--ctrl-dt" && i + 1 < argc) {
            ctrl_dt = std::stof(argv[++i]);
        } else if (arg == "--init-alt" && i + 1 < argc) {
            initial_alt = std::stof(argv[++i]);
        }
    }

    // Create simulation components
    sitl::PhysicsModel physics;
    physics.reset(initial_alt);

    sitl::SitlIMU imu(physics, 0.005f, 0.05f);       // Low noise for SITL
    sitl::SitlBarometer baro(physics, 0.1f);
    sitl::SitlMotor motor(physics);
    sitl::SitlTimer timer;
    sitl::SitlLogger logger(data_file, true);

    // Create flight controller
    core::FlightController fc(imu, baro, motor, timer, logger);

    if (!fc.init()) {
        std::cerr << "Flight controller init failed!" << std::endl;
        return 1;
    }

    // Write data header
    logger.write_data_header(
        "timestamp_ms,roll,pitch,yaw,altitude,vz,m0,m1,m2,m3");

    // Output ready signal
    std::cout << "READY" << std::endl;

    // Main command loop - read commands from stdin
    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.empty() || line[0] == '#') continue;

        Command cmd = parse_command(line);

        if (cmd.name == "ARM") {
            fc.arm();
            std::cout << "OK ARM" << std::endl;

        } else if (cmd.name == "DISARM") {
            fc.disarm();
            std::cout << "OK DISARM" << std::endl;

        } else if (cmd.name == "SET_MODE") {
            if (cmd.mode == "STABILIZE") {
                fc.set_mode(core::FlightMode::STABILIZE);
            } else if (cmd.mode == "ALT_HOLD") {
                fc.set_mode(core::FlightMode::ALT_HOLD);
            }
            std::cout << "OK SET_MODE " << cmd.mode << std::endl;

        } else if (cmd.name == "SET_ATTITUDE") {
            // Convert degrees to radians for user convenience
            float roll_rad = cmd.roll * static_cast<float>(M_PI) / 180.0f;
            float pitch_rad = cmd.pitch * static_cast<float>(M_PI) / 180.0f;
            float yaw_rate_rad = cmd.yaw_rate * static_cast<float>(M_PI) / 180.0f;
            fc.set_attitude_target(roll_rad, pitch_rad, yaw_rate_rad);
            std::cout << "OK SET_ATTITUDE" << std::endl;

        } else if (cmd.name == "SET_ALTITUDE") {
            fc.set_altitude_target(cmd.altitude);
            std::cout << "OK SET_ALTITUDE " << cmd.altitude << std::endl;

        } else if (cmd.name == "SET_THROTTLE") {
            fc.set_throttle(cmd.throttle);
            std::cout << "OK SET_THROTTLE " << cmd.throttle << std::endl;

        } else if (cmd.name == "RUN") {
            // Run simulation for specified duration
            float total_time = cmd.duration;
            if (total_time <= 0) total_time = 1.0f;

            int sim_steps_per_ctrl = static_cast<int>(ctrl_dt / sim_dt);
            int total_ctrl_steps = static_cast<int>(total_time / ctrl_dt);
            uint64_t ctrl_dt_us = static_cast<uint64_t>(ctrl_dt * 1e6f);

            for (int step = 0; step < total_ctrl_steps; step++) {
                // Run physics simulation at higher rate
                for (int sub = 0; sub < sim_steps_per_ctrl; sub++) {
                    physics.step(motor.outputs(), sim_dt);
                }

                // Advance simulated timer
                timer.advance(ctrl_dt_us);

                // Run flight controller
                fc.update();
            }

            std::cout << "OK RUN " << total_time << std::endl;

        } else if (cmd.name == "STATE") {
            // Report current state
            const auto& state = physics.state();
            std::cout << "STATE,"
                      << physics.altitude() << ","
                      << state.roll * 180.0f / static_cast<float>(M_PI) << ","
                      << state.pitch * 180.0f / static_cast<float>(M_PI) << ","
                      << state.yaw * 180.0f / static_cast<float>(M_PI) << ","
                      << physics.vertical_velocity()
                      << std::endl;

        } else if (cmd.name == "QUIT" || cmd.name == "EXIT") {
            std::cout << "OK QUIT" << std::endl;
            break;

        } else {
            std::cout << "ERR UNKNOWN_CMD " << cmd.name << std::endl;
        }
    }

    return 0;
}
