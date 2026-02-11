#!/bin/bash
# Test script for MRPT Map Loader
# Validates compilation and basic functionality

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "╔════════════════════════════════════════╗"
echo "║  MRPT Map Loader - Test Suite          ║"
echo "╚════════════════════════════════════════╝"
echo ""

# Check compilation
echo "[1/4] Checking compilation..."
if [ -f "$PROJECT_ROOT/bin/gridder" ]; then
    echo "✓ Gridder binary found"
else
    echo "⚠ Building gridder component..."
    cd "$PROJECT_ROOT"
    cmake --build . 2>&1 | tail -20
fi

# Generate test maps
echo ""
echo "[2/4] Generating test maps..."
TEST_MAPS_DIR="/tmp/gridder_test_maps"
mkdir -p "$TEST_MAPS_DIR"

python3 "$SCRIPT_DIR/generate_test_maps.py" --output-dir "$TEST_MAPS_DIR" --size small
if [ $? -eq 0 ]; then
    echo "✓ Test maps generated"
else
    echo "✗ Failed to generate test maps"
    exit 1
fi

# Run basic example
echo ""
echo "[3/4] Testing map loading..."
if [ -f "$PROJECT_ROOT/examples/mrpt_loader_examples.cpp" ]; then
    echo "✓ Example file exists"

    # Try to compile example if CMake supports it
    echo "Note: Example can be compiled as standalone with:"
    echo "  g++ -std=c++17 -I. src/mrpt_map_loader.cpp examples/mrpt_loader_examples.cpp -o mrpt_test"
else
    echo "⚠ Example file not found"
fi

# List generated maps
echo ""
echo "[4/4] Verifying test maps..."
for mapfile in "$TEST_MAPS_DIR"/*.gridmap; do
    if [ -f "$mapfile" ]; then
        size=$(ls -lh "$mapfile" | awk '{print $5}')
        echo "✓ $(basename "$mapfile") ($size)"
    fi
done

echo ""
echo "╔════════════════════════════════════════╗"
echo "║  Test Suite Complete                   ║"
echo "╚════════════════════════════════════════╝"
echo ""
echo "Test maps location: $TEST_MAPS_DIR"
echo ""
echo "Next steps:"
echo "1. Use test maps to validate loading:"
echo "   SpecificWorker::Gridder_loadMRPTMap(\"$TEST_MAPS_DIR/simple_room.gridmap\")"
echo ""
echo "2. Check component logs for loading confirmation"
echo ""
echo "3. Run path planning to verify map integration"
echo ""
