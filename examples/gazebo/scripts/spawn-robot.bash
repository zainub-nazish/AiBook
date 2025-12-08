#!/bin/bash
# spawn-robot.bash
# Module 2: The Digital Twin - Chapter 1
#
# This script demonstrates spawning a robot model at runtime
# into an already-running Gazebo simulation.
#
# Usage:
#   ./spawn-robot.bash [model_name] [x] [y] [z] [roll] [pitch] [yaw]
#
# Example:
#   ./spawn-robot.bash humanoid_robot 0 0 0.95 0 0 0

set -e

# Default values
MODEL_NAME="${1:-humanoid_robot}"
X="${2:-0}"
Y="${3:-0}"
Z="${4:-0.95}"
ROLL="${5:-0}"
PITCH="${6:-0}"
YAW="${7:-0}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Gazebo Robot Spawner ===${NC}"
echo ""

# Check if Gazebo is running
if ! pgrep -x "gz" > /dev/null && ! pgrep -x "ruby" > /dev/null; then
    echo -e "${YELLOW}Warning: Gazebo may not be running.${NC}"
    echo "Start Gazebo first with: gz sim minimal-world.sdf"
    echo ""
fi

# Get the directory containing this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODELS_DIR="$(dirname "$SCRIPT_DIR")/models"

# Check if model exists
MODEL_PATH="$MODELS_DIR/$MODEL_NAME"
if [ ! -d "$MODEL_PATH" ]; then
    echo -e "${RED}Error: Model not found at $MODEL_PATH${NC}"
    echo ""
    echo "Available models:"
    ls -1 "$MODELS_DIR" 2>/dev/null || echo "  (no models directory found)"
    exit 1
fi

echo "Model:    $MODEL_NAME"
echo "Position: x=$X, y=$Y, z=$Z"
echo "Rotation: roll=$ROLL, pitch=$PITCH, yaw=$YAW"
echo ""

# Set resource path to include our models directory
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:+$GZ_SIM_RESOURCE_PATH:}$MODELS_DIR"

# Generate a unique entity name (timestamp-based)
ENTITY_NAME="${MODEL_NAME}_$(date +%s)"

echo -e "${GREEN}Spawning robot...${NC}"

# Method 1: Using gz service (preferred for Gazebo Harmonic)
# This creates a spawn request using the Gazebo transport service

# Read the model SDF content
MODEL_SDF=$(cat "$MODEL_PATH/model.sdf")

# Create the spawn request
# The service expects an EntityFactory message
gz service -s /world/humanoid_world/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req "sdf: '$MODEL_SDF' name: '$ENTITY_NAME' pose: {position: {x: $X, y: $Y, z: $Z} orientation: {x: 0, y: 0, z: $(echo "s($YAW/2)" | bc -l), w: $(echo "c($YAW/2)" | bc -l)}}"

# Check result
if [ $? -eq 0 ]; then
    echo -e "${GREEN}Successfully spawned '$ENTITY_NAME'!${NC}"
    echo ""
    echo "Tips:"
    echo "  - Use 'gz topic -l' to see available topics"
    echo "  - Use 'gz model -m $ENTITY_NAME --pose' to get current pose"
    echo "  - Use 'gz model -m $ENTITY_NAME --delete' to remove the model"
else
    echo -e "${RED}Failed to spawn model.${NC}"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Ensure Gazebo is running with the correct world"
    echo "  2. Check that the world name matches (default: humanoid_world)"
    echo "  3. Verify GZ_SIM_RESOURCE_PATH includes the models directory"
    exit 1
fi

# Alternative Method 2: Using gz model command (simpler but less flexible)
# Uncomment this section if Method 1 doesn't work
#
# echo "Alternative: Using gz model command..."
# gz model --spawn-file "$MODEL_PATH/model.sdf" \
#     --name "$ENTITY_NAME" \
#     -x "$X" -y "$Y" -z "$Z" \
#     -R "$ROLL" -P "$PITCH" -Y "$YAW"

echo ""
echo -e "${GREEN}Done!${NC}"
