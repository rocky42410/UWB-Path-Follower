<!-- File: README.md -->
# UWB Path Follower (Unitree Go2)

C++17 implementation of a UWB-corrected path follower for Unitree Go2 with:
- UWB spherical gating (native), tilt-aware lever arm, complementary fusion
- ZUPT yaw-bias adaptation, latency compensation
- Modes: `SMOOTH_LOOP`, `SHADOW`, `OPEN_LOOP`
- Comprehensive CSV logging (innovations, shadow innovations, corrections)

## Build

```bash
export UNITREE_SDK_PATH=/path/to/unitree_sdk2-main
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
