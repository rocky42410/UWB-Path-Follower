#!/bin/bash
# verify_paths.sh - Verify all required paths for UWB Path Follower

echo "================================================"
echo "Path Verification for UWB Path Follower"
echo "================================================"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Check function
check_file() {
    if [ -f "$1" ]; then
        echo -e "${GREEN}✓${NC} Found: $1"
        return 0
    else
        echo -e "${RED}✗${NC} Missing: $1"
        return 1
    fi
}

check_dir() {
    if [ -d "$1" ]; then
        echo -e "${GREEN}✓${NC} Found: $1"
        return 0
    else
        echo -e "${RED}✗${NC} Missing: $1"
        return 1
    fi
}

# Find Unitree SDK
UNITREE_SDK=""
for path in /root/unitree_sdk2-main /root/unitree_sdk2 ~/unitree_sdk2-main ~/unitree_sdk2 /opt/unitree_sdk2; do
    if [ -d "$path/include/unitree" ]; then
        UNITREE_SDK="$path"
        break
    fi
done

if [ -z "$UNITREE_SDK" ]; then
    echo -e "${RED}ERROR: Unitree SDK not found!${NC}"
    exit 1
fi

echo "Using Unitree SDK at: $UNITREE_SDK"
echo ""

# 1. Check critical headers
echo "1. Checking Critical Headers"
echo "-----------------------------"

# CORRECTED sport_client path
check_file "$UNITREE_SDK/include/unitree/robot/go2/sport/sport_client.hpp"

# Other critical headers
check_file "$UNITREE_SDK/include/unitree/robot/channel/channel_factory.hpp"
check_file "$UNITREE_SDK/include/unitree/robot/channel/channel_subscriber.hpp"
check_file "$UNITREE_SDK/include/unitree/idl/go2/SportModeState_.hpp"
check_file "$UNITREE_SDK/include/unitree/idl/go2/UwbState_.hpp"

echo ""

# 2. Check DDS headers
echo "2. Checking DDS Headers"
echo "-----------------------"

DDS_FOUND=false
if [ -f "$UNITREE_SDK/thirdparty/include/ddscxx/dds/dds.hpp" ]; then
    echo -e "${GREEN}✓${NC} DDS headers in thirdparty"
    DDS_INCLUDE="$UNITREE_SDK/thirdparty/include/ddscxx"
    DDS_FOUND=true
elif [ -f "/usr/local/include/dds/dds.hpp" ]; then
    echo -e "${GREEN}✓${NC} System DDS headers"
    DDS_INCLUDE="/usr/local/include"
    DDS_FOUND=true
elif [ -f "/usr/local/include/ddscxx/dds/dds.hpp" ]; then
    echo -e "${GREEN}✓${NC} System DDS headers (ddscxx)"
    DDS_INCLUDE="/usr/local/include/ddscxx"
    DDS_FOUND=true
else
    echo -e "${RED}✗${NC} DDS headers not found!"
fi

echo ""

# 3. Check libraries
echo "3. Checking Libraries"
echo "---------------------"

# Check unitree_sdk2 library
LIB_FOUND=false
for lib_path in "$UNITREE_SDK/lib" "$UNITREE_SDK/lib/x86_64" "$UNITREE_SDK/lib/aarch64"; do
    if [ -f "$lib_path/libunitree_sdk2.so" ] || [ -f "$lib_path/libunitree_sdk2.a" ]; then
        echo -e "${GREEN}✓${NC} unitree_sdk2 library at: $lib_path"
        LIB_FOUND=true
        break
    fi
done

if [ "$LIB_FOUND" = false ]; then
    echo -e "${RED}✗${NC} unitree_sdk2 library not found"
fi

# Check DDS libraries
if [ -f "$UNITREE_SDK/build/lib/libddsc.so" ]; then
    echo -e "${GREEN}✓${NC} DDS libraries in build/lib"
elif [ -f "/usr/local/lib/libddsc.so" ]; then
    echo -e "${GREEN}✓${NC} System DDS libraries"
else
    echo -e "${YELLOW}!${NC} DDS libraries may be missing"
fi

echo ""

# 4. Test compilation
echo "4. Test Compilation"
echo "-------------------"

# Create test file
cat > /tmp/test_sport_client.cpp << 'EOF'
#include <iostream>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <interface>" << std::endl;
        return 1;
    }
    
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    unitree::robot::go2::SportClient sport_client;
    sport_client.SetTimeout(10.0f);
    sport_client.Init();
    
    std::cout << "SportClient initialized successfully!" << std::endl;
    return 0;
}
EOF

echo "Attempting to compile test program..."

COMPILE_CMD="g++ -std=c++17 -I$UNITREE_SDK/include"

if [ "$DDS_FOUND" = true ]; then
    COMPILE_CMD="$COMPILE_CMD -I$DDS_INCLUDE"
fi

COMPILE_CMD="$COMPILE_CMD /tmp/test_sport_client.cpp -o /tmp/test_sport_client"
COMPILE_CMD="$COMPILE_CMD -L$UNITREE_SDK/lib -lunitree_sdk2 -lddsc -lddscxx -lpthread"

if $COMPILE_CMD 2>/dev/null; then
    echo -e "${GREEN}✓${NC} Test compilation successful!"
    rm -f /tmp/test_sport_client
else
    echo -e "${RED}✗${NC} Test compilation failed"
    echo "Try this command manually to see errors:"
    echo "$COMPILE_CMD"
fi

rm -f /tmp/test_sport_client.cpp

echo ""
echo "================================================"
echo "Summary"
echo "================================================"

if [ "$DDS_FOUND" = false ]; then
    echo -e "${RED}ACTION REQUIRED:${NC}"
    echo "1. Build Unitree SDK to get DDS headers:"
    echo "   cd $UNITREE_SDK"
    echo "   mkdir -p build && cd build"
    echo "   cmake .. && make -j4"
else
    echo -e "${GREEN}All paths verified!${NC}"
    echo ""
    echo "To compile your project:"
    echo "  cd uwb_path_follower"
    echo "  mkdir -p build && cd build"
    echo "  cmake .. -DUNITREE_SDK_PATH=$UNITREE_SDK"
    echo "  make"
fi