# https://docs.docker.com/engine/containers/multi-service_container/ and ChatGPTd
#!/bin/bash
# Check if the correct number of arguments were passed
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <GZ_SIM_OPTIONS> <WEBSOCKET_PORT>
Example: $0 \"-s --headless-rendering\" \"9002\""
    exit 1
fi

# Assign passed arguments to variables
GZ_SIM_OPTIONS="$1"
WEBSOCKET_PORT="$2"

# Start the first process
gz sim $GZ_SIM_OPTIONS &

xmlstarlet edit -L --update "//port" --value $WEBSOCKET_PORT $WEBSOCKET_GZLAUNCH_FILE #Assumes $WEBSOCKET_GZLAUNCH_FILE is a defined env variable 

# Start the second process
gz launch $WEBSOCKET_GZLAUNCH_FILE &

# Wait for any process to exit
wait -n

# Exit with status of the process that exited first
exit $?



