#!/bin/bash

# Timeout duration (in seconds)
TIMEOUT=30

# Log file to capture simulator output
LOG_FILE="simulate.log"

# Run the simulator with timeout and capture output
timeout $TIMEOUT ../gradlew simulateJava > $LOG_FILE 2>&1
EXIT_CODE=$?

# Check the exit code of the timeout command
if [ $EXIT_CODE -eq 124 ]; then
    echo "Simulation timed out after ${TIMEOUT} seconds."
    exit 1
fi

# Check the log file for runtime errors
if grep -q "Exception" "$LOG_FILE"; then
    echo "Runtime error detected in the simulation:"
    cat $LOG_FILE # Display relevant error lines
    exit 2
fi

echo "Simulation completed without runtime errors."
exit 0
