# https://docs.docker.com/engine/containers/multi-service_container/ and ChatGPTd
#!/bin/sh

# Help function
show_help() {
    echo "Usage: ./docker-entrypoint.sh [--gz-sim-options=<value>] [--websocket-port=<value>]"
    echo "Example: ./docker-entrypoint.sh --gz-sim-options=\"-s --render\" --websocket-port=8080"
    echo
    echo "Arguments:"
    echo "  --gz-sim-options   Gazebo simulation options (default: \"$GZ_SIM_OPTIONS\")"
    echo "  --websocket-port   Port for the websocket (default: \"$WEBSOCKET_PORT\")"
}

# Error function
show_error_and_exit() {
    echo "Error: $1"
    show_help
    exit 1
}

# Parse arguments
for arg in "$@"; do
    case $arg in
        --gz-sim-options=*)
            GZ_SIM_OPTIONS="${arg#*=}"
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

# Start the first process
gz sim $GZ_SIM_OPTIONS &
GZ_SIM_PID=$!

xmlstarlet edit -L --update "//port" --value $WEBSOCKET_PORT $WEBSOCKET_GZLAUNCH_FILE #Assumes $WEBSOCKET_GZLAUNCH_FILE is a defined env variable 

# Start the second process
gz launch $WEBSOCKET_GZLAUNCH_FILE &
GZ_WS_PID=$1

# Wait for any process to exit
wait "$GZ_SIM_PID" || wait "$GZ_WS_PID"

# Exit with status of the process that exited first
exit $?



