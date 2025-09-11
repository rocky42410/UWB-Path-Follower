# Enhanced UWB Path Follower for Unitree Go2

## Overview

This is an enhanced version of the UWB path follower that fixes the laggy/jittery UWB issues and startup segfaults. The implementation uses **only C++17 standard library** features - no external dependencies beyond the Unitree SDK.

## Key Improvements

### 1. **Time-Stamped Ring Buffer** (`ts_ring_buffer.hpp`)
- Stores UWB positions with monotonic timestamps
- Provides interpolation for any requested time
- Handles late-arriving UWB measurements gracefully
- 2-second history window by default

### 2. **Fused Heading Estimator**
- Fast updates from IMU gyro (50Hz)
- Slow corrections from UWB displacement over a window
- Complementary filter with tunable gain `k_uwb`
- Robust to UWB jitter and delays

### 3. **Hysteresis Control** 
- Separate enter/exit tolerances (15°/25° default)
- Prevents oscillation at heading gate boundaries
- Smooth transitions between TURN and FORWARD states

### 4. **Curvature Control with Floors**
- Always maintains minimum forward speed in FWD state
- Deadband floor on turn rate (0.65 rad/s minimum)
- Proportional reduction of forward speed with heading error
- De-chatters small corrections

### 5. **Stall Detection & Recovery**
- Monitors heading error improvement over time
- Executes probe move if stuck turning for >2 seconds
- Automatically recovers from local minima

## File Structure

```
/home/claude/
├── include/uwb_path_follower/
│   ├── types.hpp              # Basic types (Pose2D, Twist2D, etc.)
│   ├── config.hpp             # Enhanced configuration with new parameters
│   ├── math2d.hpp             # 2D math helpers
│   ├── ts_ring_buffer.hpp    # Time-stamped ring buffer (header-only)
│   ├── uwb_receiver.hpp      # UWB receiver with position buffering
│   └── all_components.hpp    # Estimator, converter, logger components
├── src/
│   └── uwb_path_controller.cpp  # Main controller with enhanced logic
├── config/
│   └── uwb_config.ini         # Configuration file with tuning parameters
├── CMakeLists.txt             # Build configuration
├── build.sh                   # Build script
└── README.md                  # This file
```

## Configuration Parameters

### Controller Parameters
- `enter_tol_deg`: Heading error to unlock from TURN to FWD (default: 15°)
- `exit_tol_deg`: Heading error to relock from FWD to TURN (default: 25°)
- `w_min`: Minimum turn rate to overcome deadband (default: 0.65 rad/s)
- `w_max`: Maximum turn rate (default: 1.20 rad/s)
- `v_min`: Minimum forward speed in FWD state (default: 0.20 m/s)
- `v_nom`: Nominal forward speed when aligned (default: 0.35 m/s)
- `k_turn`: Turn control gain (default: 1.0)
- `err_alpha`: Heading error low-pass filter constant (default: 0.25)

### Heading Fusion Parameters
- `k_uwb`: Complementary gain for UWB displacement heading (default: 0.12)
- `disp_window_sec`: Time window for displacement calculation (default: 0.6s)
- `disp_min_m`: Minimum displacement to trust UWB heading (default: 0.05m)

### Stall Recovery Parameters
- `stall_check_time`: Time before checking for stall (default: 2.0s)
- `probe_distance`: Distance to move in probe (default: 0.25m)
- `probe_speed`: Speed during probe move (default: 0.25 m/s)

## Building

```bash
# Set SDK path (if not already set)
export UNITREE_SDK_PATH=/path/to/unitree_sdk2-main

# Build
./build.sh

# Or manually:
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
```

## Running

```bash
# Basic usage
./build/uwb_path_controller --interface eth0

# With custom config
./build/uwb_path_controller --interface eth0 --config config/uwb_config.ini

# Different modes
./build/uwb_path_controller --interface eth0 --mode SMOOTH_LOOP  # Default, with fusion
./build/uwb_path_controller --interface eth0 --mode OPEN_LOOP    # Dead reckoning only
./build/uwb_path_controller --interface eth0 --mode SHADOW       # Log but don't apply
```

## What Was Fixed

### 1. **Startup Segfault**
- Proper initialization order of all components
- Null checks on subscriber callbacks
- Mutex protection for shared data

### 2. **Heading Gate Oscillation**
- Hysteresis with separate enter/exit tolerances
- Low-pass filtered error for gating decisions
- State change logging for debugging

### 3. **Turn Deadband Issues**
- Explicit floor on turn commands (w_min)
- Never allows zero velocity in TURN state
- Always maintains some forward motion in FWD state

### 4. **UWB Lag Handling**
- Ring buffer stores positions by timestamp
- Displacement calculated over fixed time window
- Complementary fusion instead of direct replacement

### 5. **Stall Recovery**
- Detects when turning isn't improving heading
- Executes probe move to escape local minima
- Automatic recovery without manual intervention

## Validation Tests

### Test 1: Standing Wiggle
Slowly yaw ±15° for 10s. The fused heading should track gyro with <5° drift.

### Test 2: 90° Turn
Set a waypoint 90° off current heading. Should complete turn in 2-3s and transition to FWD smoothly.

### Test 3: 180° U-Turn
Full out-and-back path. Should handle the 180° turn without oscillation.

### Test 4: Stall Recovery
Artificially block turning (e.g., against a wall). Should detect stall and execute probe move.

## Logging

The controller logs comprehensive CSV data including:
- Timestamps and states
- Position estimates (fused and predicted)
- UWB measurements and quality
- Innovations and gating decisions
- Commands (pre and post filtering)
- IMU data and biases
- Foot forces and ZUPT status

Analyze with the provided Python scripts in `scripts/analyze_uwb_logs.py`.

## Troubleshooting

### "Interface not found"
- Check available interfaces with `ip link show`
- Common interfaces: eth0, wlan0, enp3s0

### "UWB not available"
- Ensure base station is powered and in range
- Check UWB topic with `ros2 topic echo /rt/uwbstate`
- Verify tag is mounted and powered

### "Oscillating at gate"
- Increase `enter_tol_deg` or decrease `exit_tol_deg`
- Increase `err_alpha` for more filtering
- Check for mechanical backlash

### "Not turning fast enough"
- Increase `w_min` or `w_max`
- Check for mechanical resistance
- Verify motor commands are reaching robot

## Theory of Operation

The controller implements a **complementary filter** for heading estimation:
- High-frequency components from IMU gyro (fast, drifty)
- Low-frequency components from UWB displacement (slow, absolute)

The **hysteresis gate** prevents chattering:
- Must reach small error (15°) to unlock turning
- Must exceed large error (25°) to relock turning

The **curvature control** ensures smooth paths:
- Forward speed scales with cos²(error/2)
- Always maintains minimum forward motion
- Turn rate proportional to sin(error)

## License

Proprietary - for use with Unitree Go2 robots only.

## Support

For issues or questions, refer to the handoff documentation or contact the development team.