# https://docs.docker.com/engine/containers/multi-service_container/ and ChatGPTd
#!/bin/bash

# Help function
function show_help() {
    echo "Usage: ./docker-entrypoint.sh [--websocket-gzlaunch-path=<value>] [--websocket-port=<value>]"
    echo "Example: ./docker-entrypoint.sh --websocket-gzlaunch-path=\"/websocket.gzlaunch\" --websocket-port=8080"
    echo
    echo "Arguments:"
    echo "  --websocket-gzlaunch-path   Gazebo simulation options (default: \"$WEBSOCKET_GZLAUNCH_PATH\")"
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
        --websocket-gzlaunch-path=*)
            WEBSOCKET_GZLAUNCH_PATH="${arg#*=}"
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
xmlstarlet edit -L --update "//port" --value $WEBSOCKET_PORT $WEBSOCKET_GZLAUNCH_PATH #Assumes $WEBSOCKET_GZLAUNCH_FILE is a defined env variable 

if [ "$GZ_VERSION" = "fortress" ]; then #Assumes $GZ_VERSION is a defined env variable 
  ws_launch='ign launch'
  sim_start='ign gazebo'
else
  ws_launch='gz launch'
  sim_start='gz sim'
fi

# Start websocket server in background
$ws_launch $WEBSOCKET_GZLAUNCH_PATH &

echo "
Gazebo Simulation is now ready.

Run Gazebo in the browser: http://localhost:$VNC_PORT

To run the Gazebo visualization tool, go to https://app.gazebosim.org/visualization and connect with the following Websocket URL:
ws://localhost:$WEBSOCKET_PORT
"

# Launch Gazebo Simulation
$sim_start $GZ_SIM_OPTIONS

# Wait for any process to exit
wait -n

# Exit with status of process that exited first
exit $?