#!/bin/bash

# Timeout duration (in seconds)
TIMEOUT=40

# Log file to capture simulator output
LOG_FILE="simulate.log"

echo -e "\033[0;36mStarted Gradle Simulation with a timeout of \033[0;34m${TIMEOUT}\033[0;36m seconds...\n\033[0;37mNote: There is no logging while the simulation is running. The log will be dumped afterward.\033[0m"

# Run the simulator with timeout and capture output
timeout $TIMEOUT ./gradlew simulateJava > $LOG_FILE 2>&1
# timeout $TIMEOUT ./gradlew simulateJava   -Dorg.gradle.java.home="C:\Users\Public\wpilib\2025\jdk" > $LOG_FILE 2>&1
EXIT_CODE=$?

# Check the exit code of the timeout command
if [ $EXIT_CODE -eq 124 ]; then
    if grep -q "********** Robot program starting **********" "$LOG_FILE"; then
        cat $LOG_FILE
        echo -e "\033[1;32mSimulation ran for \033[1;34m${TIMEOUT}\033[1;32m seconds with no errors.\033[0m"
        exit 0
    fi
    cat $LOG_FILE
    echo -e "\033[1;33mSimulation did not start successfully. (Is the timeout too short?)\033[0m"
    exit 0
fi

# Check the log file for runtime errors
if grep -q "Error" "$LOG_FILE"; then
    cat $LOG_FILE
    echo -e "\033[1;31mRuntime error(s) detected in the simulation!\033[0m"
    exit 2
fi

cat $LOG_FILE
echo -e "\033[1;33mSimulation exited immediately. (Is gradlew working?)\033[0m"
exit 1
