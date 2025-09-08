#!/bin/bash
set -e

# Check for SDK path
if [ -z "$UNITREE_SDK_PATH" ]; then
    echo "⚠️  UNITREE_SDK_PATH not set"
    echo "   Export it or pass to cmake:"
    echo "   export UNITREE_SDK_PATH=/path/to/unitree_sdk2"
    echo "   OR"
    echo "   cmake .. -DUNITREE_SDK_PATH=/path/to/unitree_sdk2"
fi

# Create and enter build directory
mkdir -p build
cd build

# Configure
echo "Configuring..."
cmake .. ${UNITREE_SDK_PATH:+-DUNITREE_SDK_PATH=$UNITREE_SDK_PATH}

# Build
echo "Building..."
make -j$(nproc)

echo "✅ Build complete!"
echo "   Binary: build/uwb_path_controller"
