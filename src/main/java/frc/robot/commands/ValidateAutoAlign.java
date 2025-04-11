package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Swerve;

public class ValidateAutoAlign extends Command {
    private final Swerve drivetrain;

    /*
     * 3 states:
     * 1. Valid
     * 2. coral
     * 3. invalid
     */

    private enum AutoAlignState {
        VALID(0, 0),
        CORAL(0, 0),
        INVALID(0, 0);

        private double start;
        private double end;

        private AutoAlignState(double start, double end) {
            this.start = start;
            this.end = end;
        }

        public static AutoAlignState getState(double in) {
            if(in >= VALID.start && in <= VALID.end) {
                return VALID;
            } else if(in >= CORAL.start && in <= CORAL.end) {
                return CORAL;
            } else if(in >= INVALID.start && in <= INVALID.end) {
                return INVALID;
            } else {
                return INVALID;
            }
        }
    }

    public ValidateAutoAlign(Swerve drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        //if the range sensor values are greater than toelrance
        //TODO: add tolerance
        
        AutoAlignState state = AutoAlignState.getState(drivetrain.getRangeSensorAverage());
        switch(state) {
            case VALID:
                this.cancel();
                //probably gonna led
                break;
            case CORAL:
                //use secondary coral map
                break;
            case INVALID:
                
                break;
        }
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
