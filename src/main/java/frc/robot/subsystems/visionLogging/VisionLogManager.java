package frc.robot.subsystems.visionLogging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class VisionLogManager {

    private static boolean isLogging = false;
    private static boolean isOnDelayedStop = false;
    
    public static void startLogging() {
        VisionLogger.startLogging();
        isLogging = true;
    }

    public static void stopLogging() {
        VisionLogger.stopLogging();
        isLogging = false;
    }

    public static void delayedStop(double delaySeconds) {
        if (isOnDelayedStop || !isLogging) return;
        isOnDelayedStop = true;

        new WaitCommand(delaySeconds).andThen(new InstantCommand(() -> {
            if (isOnDelayedStop && isLogging)
                stopLogging();

            isOnDelayedStop = false;
            })
        ).schedule();
    }

    public static void delayedStop() {
        delayedStop(2d);
    }

    public static void startOrContinueLogging() {
        isOnDelayedStop = false;

        if (!isLogging) {
            startLogging();
        }
    }

    public static void autoLog(AUTO_LOG_MODE mode) {

        if (Robot.isSimulation()) return;
        
        switch (mode) {
            case ENABLED:
                if (!DriverStation.isFMSAttached() && DriverStation.isEnabled()) {
                    startOrContinueLogging();
                }
                break;

            case ALWAYS:
                startOrContinueLogging();
                break;

            case TOGGLE:
                if (LightningShuffleboard.getBool("Photon Vision", "Log Vision", false)) {
                    startOrContinueLogging();
                } else if (isLogging) {
                    stopLogging();
                }

                break;
        }
    }

    /**
     * @param FMSMode mode to use when FMS is attached
     * @param NoFMSMode mode to use when FMS is not attached
     */
    public static void autoLog(AUTO_LOG_MODE FMSMode, AUTO_LOG_MODE NoFMSMode) {

        if (DriverStation.isFMSAttached()){
            autoLog(FMSMode);
        } else {
            autoLog(NoFMSMode);   
        }
    }

    public enum AUTO_LOG_MODE {
        ENABLED,
        TOGGLE,
        ALWAYS
    }
}
