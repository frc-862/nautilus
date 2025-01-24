#!/bin/bash

# Timeout duration (in seconds)
TIMEOUT=45

# Log file to capture simulator output
LOG_FILE="simulate.log"

echo -q "\033[0;33mStarted Gradle & Simulation...\033[0m"

# Run the simulator with timeout and capture output
timeout $TIMEOUT ./gradlew simulateJava > $LOG_FILE 2>&1
# timeout $TIMEOUT ./gradlew simulateJava   -Dorg.gradle.java.home="C:\Users\Public\wpilib\2025\jdk" > $LOG_FILE 2>&1
EXIT_CODE=$?

# Check the exit code of the timeout command
if [ $EXIT_CODE -eq 124 ]; then
    if grep -q "********** Robot program starting **********" "$LOG_FILE"; then
        echo -e "\033[0;32mSimulation ran for \033[0;34m${TIMEOUT}\033[0;32m seconds!\n\033[0;37mLog dump:\033[0m"
        cat $LOG_FILE
        exit 0
    fi
    echo -e "\033[1;31mSimulation did not start successfully. (Is the timeout too short?) \n\033[0;37mLog dump:\033[0m"
    cat $LOG_FILE
    exit 0
fi

# Check the log file for runtime errors
if grep -q "Error" "$LOG_FILE"; then
    echo -e "\033[1;31mRuntime error detected in the simulation:\033[0m"
    cat $LOG_FILE
    exit 2
fi

echo -e "\033[1;33mSimulation exited immediately. (Is gradlew working?)\033[0m"
cat $LOG_FILE
exit 1
