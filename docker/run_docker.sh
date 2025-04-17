#!/bin/bash

# Check if argument is provided
if [ $# -eq 0 ]; then
    echo "Usage: $0 [with_gz|without_gz]"
    echo "  with_gz    - Run with Gazebo support"
    echo "  without_gz - Run without Gazebo support"
    exit 1
fi

# Check if DISPLAY is set
if [ -z "$DISPLAY" ]; then
    echo "Error: DISPLAY environment variable is not set"
    exit 1
fi

# Allow X11 forwarding from Docker container
echo "Allowing X11 forwarding from Docker container..."
xhost +local:docker

# Set environment variables based on argument
case "$1" in
    "with_gz")
        echo "Running with Gazebo support..."
        export WITH_GAZEBO=true
        ;;
    "without_gz")
        echo "Running without Gazebo support..."
        export WITH_GAZEBO=false
        ;;
    *)
        echo "Invalid argument: $1"
        echo "Usage: $0 [with_gz|without_gz]"
        echo "  with_gz    - Run with Gazebo support"
        echo "  without_gz - Run without Gazebo support"
        exit 1
        ;;
esac

# Run docker-compose
docker compose -f docker-compose.yml up 