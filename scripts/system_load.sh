#!/bin/bash

# Get the project root directory (parent of scripts directory)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "Loading SynchroHumanoid environment..."
echo "Project root: $PROJECT_ROOT"

# Source the activate file
if [ -f "$PROJECT_ROOT/activate" ]; then
    echo "Sourcing activate..."
    source "$PROJECT_ROOT/activate"
else
    echo "Warning: activate file not found at $PROJECT_ROOT/activate"
fi

Source ROS 2 Humble base installation
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "Sourcing /opt/ros/humble/setup.bash..."
    source "/opt/ros/humble/setup.bash"
else
    echo "Warning: /opt/ros/humble/setup.bash not found"
fi

# Source ROS workspace
if [ -f "$PROJECT_ROOT/ros_src/install/setup.bash" ]; then
    echo "Sourcing ros_src/install/setup.bash..."
    source "$PROJECT_ROOT/ros_src/install/setup.bash"
else
    echo "Warning: ros_src/install/setup.bash not found"
fi

# Find and source all ROS package workspaces in the project directory
echo "Searching for ROS workspaces in project folders..."
for dir in "$PROJECT_ROOT"/projects/*/; do
    if [ -d "$dir" ]; then
        if [ -f "${dir}install/setup.bash" ]; then
            echo "Sourcing ${dir}install/setup.bash..."
            source "${dir}install/setup.bash"
        fi
    fi
done

echo "Environment setup complete!"
echo ""
echo "To use this script, run:"
echo "  source $0"