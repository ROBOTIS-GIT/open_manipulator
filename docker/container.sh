#!/bin/bash

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CONTAINER_NAME="open_manipulator"
GITHUB_RELEASES_API="https://api.github.com/repos/ROBOTIS-GIT/open_manipulator/releases/latest"
META_PACKAGE_XML="${SCRIPT_DIR}/../open_manipulator/package.xml"

# Function to display help
show_help() {
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  help                    Show this help message"
    echo "  start                   Start the container"
    echo "  enter                   Enter the running container"
    echo "  stop                    Stop the container"
    echo ""
    echo "Examples:"
    echo "  $0 start                Start container"
    echo "  $0 enter                Enter the running container"
    echo "  $0 stop                 Stop the container"
}

# Function to start the container
start_container() {
    # Set up X11 forwarding only if DISPLAY is set
    if [ -n "$DISPLAY" ]; then
        echo "Setting up X11 forwarding..."
        xhost +local:docker || true
    else
        echo "Warning: DISPLAY environment variable is not set. X11 forwarding will not be available."
    fi

    echo "Starting Open Manipulator container..."
    # Notify if an update is available (meta package version vs GitHub latest release)
    CURRENT_VER=$(get_current_version)
    LATEST_VER=$(get_latest_version)
    if update_available "${CURRENT_VER}" "${LATEST_VER}"; then
        print_update_notice "${CURRENT_VER}" "${LATEST_VER}"
    fi

    ### rmw_zenoh notice (remove later)
    print_rmw_zenoh_notice
    ### rmw_zenoh notice (remove later)

    # Copy udev rule for FTDI (U2D2)
    echo 'KERNEL=="ttyUSB*", DRIVERS=="ftdi_sio", MODE="0666", ATTR{device/latency_timer}="1"' | sudo tee /etc/udev/rules.d/99-u2d2.rules > /dev/null

    # Reload udev rules
    echo "Reloading udev rules..."
    sudo udevadm control --reload-rules
    sudo udevadm trigger

    # Pull the latest images
    docker compose -f "${SCRIPT_DIR}/docker-compose.yml" pull

    # Run docker-compose
    docker compose -f "${SCRIPT_DIR}/docker-compose.yml" up -d
}

# Function to enter the container
enter_container() {
    # Set up X11 forwarding only if DISPLAY is set
    if [ -n "$DISPLAY" ]; then
        echo "Setting up X11 forwarding..."
        xhost +local:docker || true
    else
        echo "Warning: DISPLAY environment variable is not set. X11 forwarding will not be available."
    fi

    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        echo "Error: Container is not running"
        exit 1
    fi

    # Notify if an update is available (meta package version vs git tag)
    CURRENT_VER=$(get_current_version)
    GIT_VER=$(get_latest_version)
    if update_available "${CURRENT_VER}" "${GIT_VER}"; then
        print_update_notice "${CURRENT_VER}" "${GIT_VER}"
    fi

    ### rmw_zenoh notice (remove later)
    print_rmw_zenoh_notice
    ### rmw_zenoh notice (remove later)

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

# Get current version from meta package (open_manipulator) package.xml
print_update_notice() {
    local current_ver="$1"
    local latest_ver="$2"
    W=52
    BAR=$(printf '%*s' $W '' | tr ' ' '=')
    LINE1="New version available: ${latest_ver} (current: ${current_ver})."
    LINE2="1. Stop Container: ./container.sh stop"
    LINE3="2. Pull Git Repo: git pull origin jazzy"
    LINE4="3. Start Container: ./container.sh start"
    echo ""
    echo "  +${BAR}+"
    printf "  |  %-$((W-2))s|\n" "$LINE1"
    printf "  |  %-$((W-2))s|\n" "$LINE2"
    printf "  |  %-$((W-2))s|\n" "$LINE3"
    printf "  |  %-$((W-2))s|\n" "$LINE4"
    echo "  +${BAR}+"
    echo ""
}

### rmw_zenoh notice (remove later)
print_rmw_zenoh_notice() {
    W=52
    BAR=$(printf '%*s' $W '' | tr ' ' '=')
    LINE1="Since v5.0.0, rmw_zenoh_cpp is the default RMW."
    LINE2="RMW_IMPLEMENTATION is set in ~/.bashrc inside the"
    LINE3="container."
    echo ""
    echo "  +${BAR}+"
    printf "  |  %-$((W-2))s|\n" "$LINE1"
    printf "  |  %-$((W-2))s|\n" "$LINE2"
    printf "  |  %-$((W-2))s|\n" "$LINE3"
    echo "  +${BAR}+"
    echo ""
}
### rmw_zenoh notice (remove later)

get_current_version() {
    local ver
    if [ -f "${META_PACKAGE_XML}" ]; then
        ver=$(sed -n 's/.*<version>\([^<]*\)<\/version>.*/\1/p' "${META_PACKAGE_XML}" | head -1)
    fi
    echo "${ver:-unknown}"
}

# Get latest version from GitHub releases (ROBOTIS-GIT/open_manipulator)
get_latest_version() {
    local json tag
    json=$(curl -sL --connect-timeout 5 "${GITHUB_RELEASES_API}" 2>/dev/null)
    tag=$(echo "${json}" | sed -n 's/.*"tag_name":\s*"\([^"]*\)".*/\1/p' | head -1)
    # Strip optional 'v' prefix for comparison with package.xml
    if [ -n "${tag}" ]; then
        echo "${tag#v}"
    else
        echo ""
    fi
}

# Check if the latest version is newer than the current version
update_available() {
    local current="$1"
    local git_ver="$2"
    if [ -z "${git_ver}" ]; then
        return 1
    fi
    if [ "${git_ver}" = "${current}" ]; then
        return 1
    fi
    local newer
    newer=$(echo -e "${current}\n${git_ver}" | sort -V | tail -1)
    [ "${newer}" = "${git_ver}" ]
}

# Main command handling
case "$1" in
    "help")
        show_help
        ;;
    "start")
        start_container
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
