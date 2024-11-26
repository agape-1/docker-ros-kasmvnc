# https://docs.docker.com/engine/containers/multi-service_container/ and ChatGPTd
#!/bin/bash

# Help function
function show_help() {
    echo "Usage: ./docker-entrypoint.sh [--websocket-gzlaunch-file=<value>] [--websocket-port=<value>]"
    echo "Example: ./docker-entrypoint.sh --websocket-gzlaunch-file=\"/websocket.gzlaunch\" --websocket-port=8080"
    echo
    echo "Arguments:"
    echo "  --websocket-gzlaunch-file   Gazebo simulation options (default: \"$WEBSOCKET_GZLAUNCH_FILE\")"
    echo "  --websocket-port   Port for the websocket (default: \"$WEBSOCKET_PORT\")"
}

# Error function
function show_error_and_exit() {
    echo "Error: $1"
    show_help
    exit 1
}

# Parse arguments
for arg in "$@"; do
    case $arg in
        --websocket-gzlaunch-file=*)
            WEBSOCKET_GZLAUNCH_FILE="${arg#*=}"
            ;;
        --websocket-port=*)
            WEBSOCKET_PORT="${arg#*=}"
            ;;
        --help|-h)
            show_help
            exit 0
            ;;
        *)
            show_error_and_exit "Unknown option: $arg"
            ;;
    esac
done

# Dynamically assign the websocket port
xmlstarlet edit -L --update "//port" --value $WEBSOCKET_PORT $WEBSOCKET_GZLAUNCH_FILE #Assumes $WEBSOCKET_GZLAUNCH_FILE is a defined env variable 

# Start websocket server in background
gz launch $WEBSOCKET_GZLAUNCH_FILE &

# Launch Gazebo Simulation
gz sim ${GZ_SIM_OPTIONS}

# Wait for any process to exit
wait -n

# Exit with status of process that exited first
exit $?