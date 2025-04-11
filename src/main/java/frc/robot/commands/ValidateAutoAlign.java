package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.VisionConstants.ReefPose;
import frc.robot.subsystems.LEDs;

public class ValidateAutoAlign extends Command {
    private final Swerve drivetrain;
    private final LEDs leds;
    private final ReefPose reefPose;

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

    public ValidateAutoAlign(Swerve drivetrain, LEDs led, ReefPose reefPose) {
        this.drivetrain = drivetrain;
        this.leds = led;
        this.reefPose = reefPose;
    }

    @Override
    public void initialize() {
        //if the range sensor values are greater than toelrance
        //TODO: add tolerance
        
        AutoAlignState state = AutoAlignState.getState(drivetrain.getRangeSensorAverage());
        switch(state) {
            case VALID:
                //probably gonna led
                this.cancel();
                return;
            case CORAL:
                //use secondary coral map
                addRequirements(drivetrain);
                break;
            case INVALID:
                CommandScheduler.getInstance().schedule(PoseBasedAutoAlign.getPoseAutoAlign(drivetrain, reefPose, leds));
                this.cancel();
                break;
        }
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        CommandScheduler.getInstance().cancel(this);
    }
}
