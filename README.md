# UWB Path Follower for Unitree Go2

## ⚠️ Setup Required

This project was generated with placeholders. You need to:

1. **Copy the full implementations** from the provided artifacts:
   - `src/uwb_path_controller.cpp` - Copy Artifact #1 
   - `test/test_estimator.cpp` - Copy Artifact #4
   - `config/uwb_config.yaml` - Copy Artifact #3 (full version)
   - `scripts/analyze_uwb_logs.py` - Copy Artifact #6

2. **Set SDK path**:
   ```bash
   export UNITREE_SDK_PATH=/path/to/unitree_sdk2
   ```

3. **Build**:
   ```bash
   ./build.sh
   ```

## Project Structure
```
uwb_path_follower/
├── src/                    # C++ source files
├── include/                # Header files
├── test/                   # Unit tests
├── config/                 # Configuration (YAML)
├── scripts/                # Analysis tools (Python)
├── build/                  # Build output
├── data/                   # Log files
└── figures/                # Analysis output
```

## Quick Start

1. Configure base station location in `config/uwb_config.yaml`
2. Build: `./build.sh`
3. Run: `./build/uwb_path_controller`
4. Analyze: `python3 scripts/analyze_uwb_logs.py data/*.csv`

## Documentation

See individual artifact descriptions for:
- Architecture details
- Configuration parameters
- Tuning guidelines
- Data collection protocols
