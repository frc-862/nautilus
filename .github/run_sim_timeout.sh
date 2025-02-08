#!/bin/bash

run_simulator() {
    # Timeout duration (in seconds)
    TIMEOUT=$1

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
            # 0 means success
            return 0
        fi
        cat $LOG_FILE
        echo -e "\033[1;33mSimulation did not start successfully. (ran out of time)\033[0m"
        # 1 means runtime error
        return 1
    fi

    # Check the log file for runtime errors
    if grep -q "Error" "$LOG_FILE"; then
        cat $LOG_FILE
        echo -e "\033[1;31mRuntime error(s) detected in the simulation!\033[0m"
        # 3 means full errors in simulation
        return 3
    fi

    cat $LOG_FILE
    echo -e "\033[1;33mSimulation exited immediately. (Is gradlew working?)\033[0m"
    # 2 means gradlew error
    return 2
}

RUN_SUCCESSFULNESS=$(run_simulator "40")

if [[ $RUN_SUCCESSFULNESS -eq 0 ]]; then
    exit 0
elif [[ $RUN_SUCCESSFULNESS -eq 1 ]]; then
    RUN_SUCCESSFULNESS=$(run_simulator "120")
elif [[ $RUN_SUCCESSFULNESS -eq 2 ]]; then
    exit 2
elif [[ $RUN_SUCCESSFULNESS -eq 3 ]]; then
    exit 3
fi

if [[ $RUN_SUCCESSFULNESS -eq 0 ]]; then
    exit 0
elif [[ $RUN_SUCCESSFULNESS -eq 1 ]]; then
    exit 1
elif [[ $RUN_SUCCESSFULNESS -eq 2 ]]; then
    exit 2
elif [[ $RUN_SUCCESSFULNESS -eq 3 ]]; then
    exit 3
fi















# #!/bin/bash

# # Log file to capture simulator output
# LOG_FILE="simulate.log"

# run_simulator() {
#     # Timeout duration (in seconds)
#     TIMEOUT=$1

#     echo -e "\033[0;36mStarted Gradle Simulation with a timeout of \033[0;34m${TIMEOUT}\033[0;36m seconds...\n\033[0;37mNote: There is no logging while the simulation is running. The log will be dumped afterward.\033[0m"

#     # Run the simulator with timeout and capture output
#     timeout $TIMEOUT ./gradlew simulateJava > $LOG_FILE 2>&1
#     # timeout $TIMEOUT ./gradlew simulateJava   -Dorg.gradle.java.home="C:\Users\Public\wpilib\2025\jdk" > $LOG_FILE 2>&1
#     EXIT_CODE=$?

#     # Check if the simulation ran successfully in time
#     if [ $EXIT_CODE -eq 124 ]; then
#         if grep -q "********** Robot program starting **********" "$LOG_FILE"; then
#             cat $LOG_FILE
#             echo -e "\033[1;32mSimulation ran for \033[1;34m${TIMEOUT}\033[1;32m seconds with no errors.\033[0m"
#             return 0
#         fi
#         cat $LOG_FILE
#         echo -e "\033[1;33mSimulation did not start successfully. (Is the timeout too short?)\033[0m"
#         return 1
#     fi

#     # Check the log file for runtime errors
#     if grep -q "Error" "$LOG_FILE"; then
#         cat $LOG_FILE
#         echo -e "\033[1;31mRuntime error(s) detected in the simulation!\033[0m"
#         return 1
#     fi

#     # Check if the simulation exited immediately
#     cat $LOG_FILE
#     echo -e "\033[1;33mSimulation exited immediately. (Is gradlew working?)\033[0m"
#     return 1
# }

# # first run the simulation for 40 seconds
# if [[ $(run_simulator 40) -eq 0 ]]; then
#     exit 0
# fi
# echo "failed first run"

# # if timeout, run the simulation for 120 seconds
# if [[ $(run_simulator 120) -eq 0 ]]; then
#     exit 0
# fi
# echo "failed second run"

# # if timeout or another error, exit with error
# exit 1
