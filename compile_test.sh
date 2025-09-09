#!/bin/bash
# compile_test.sh - Test compilation without CMake

echo "Testing compilation of uwb_path_controller..."

# Set paths
UNITREE_SDK="/root/unitree_sdk2-main"
DDS_INCLUDE="$UNITREE_SDK/thirdparty/include/ddscxx"

# Compile command
g++ -std=c++17 \
    -I"$UNITREE_SDK/include" \
    -I"$DDS_INCLUDE" \
    -Iinclude \
    -c src/uwb_path_controller.cpp \
    -o uwb_path_controller.o \
    2>&1 | tee compile_output.txt

if [ ${PIPESTATUS[0]} -eq 0 ]; then
    echo ""
    echo "✅ Compilation successful!"
    echo "Object file created: uwb_path_controller.o"
    rm uwb_path_controller.o
else
    echo ""
    echo "❌ Compilation failed. See errors above."
    echo "Common issues:"
    echo "  1. Missing header files - run: bash create_headers.sh"
    echo "  2. Wrong SDK path - edit UNITREE_SDK in this script"
    echo "  3. Missing DDS headers - build Unitree SDK first"
fi