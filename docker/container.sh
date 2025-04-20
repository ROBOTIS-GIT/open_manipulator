#!/bin/bash

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CONTAINER_NAME="open_manipulator"

# Function to display help
show_help() {
    echo "Usage: $0 [command] [options]"
    echo ""
    echo "Commands:"
    echo "  help                    Show this help message"
    echo "  start [with_gz|without_gz]  Start the container with or without Gazebo support"
    echo "  enter                   Enter the running container"
    echo "  stop                    Stop the container"
    echo ""
    echo "Examples:"
    echo "  $0 start with_gz        Start container with Gazebo support"
    echo "  $0 start without_gz     Start container without Gazebo support"
    echo "  $0 enter                Enter the running container"
    echo "  $0 stop                 Stop the container"
}

# Function to start the container
start_container() {
    if [ $# -eq 0 ]; then
        echo "Error: Please specify 'with_gz' or 'without_gz'"
        show_help
        exit 1
    fi

    # Set up X11 forwarding only if DISPLAY is set
    if [ -n "$DISPLAY" ]; then
        echo "Setting up X11 forwarding..."
        xhost +local:docker || true
    else
        echo "Warning: DISPLAY environment variable is not set. X11 forwarding will not be available."
    fi

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
            show_help
            exit 1
            ;;
    esac

    # Run docker-compose
    docker compose -f "${SCRIPT_DIR}/docker-compose.yml" up -d
}

# Function to enter the container
enter_container() {
    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        echo "Error: Container is not running"
        exit 1
    fi
    docker exec -it "$CONTAINER_NAME" bash
}

# Function to stop the container
stop_container() {
    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        echo "Error: Container is not running"
        exit 1
    fi

    echo "Warning: This will stop and remove the container. All unsaved data in the container will be lost."
    read -p "Are you sure you want to continue? [y/N] " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        docker compose -f "${SCRIPT_DIR}/docker-compose.yml" down
    else
        echo "Operation cancelled."
        exit 0
    fi
}

# Main command handling
case "$1" in
    "help")
        show_help
        ;;
    "start")
        shift
        start_container "$@"
        ;;
    "enter")
        enter_container
        ;;
    "stop")
        stop_container
        ;;
    *)
        echo "Error: Unknown command"
        show_help
        exit 1
        ;;
esac
