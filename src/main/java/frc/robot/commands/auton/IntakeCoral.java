package frc.robot.commands.auton;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.CoralCollectorConstants;
import frc.robot.subsystems.CoralCollector;

public class IntakeCoral extends Command {

    private final CoralCollector collector;
    private final double power;
    private final BooleanSupplier isCoralMode;

    public IntakeCoral(CoralCollector collector, double power, BooleanSupplier isCoralMode) {
        this.collector = collector;
        this.power = power;
        this.isCoralMode = isCoralMode;

        addRequirements(collector);
    }

    @Override
    public void initialize() {
        collector.setPower(power);
    }

    @Override
    public void execute() {
        collector.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        collector.setPower(isCoralMode.getAsBoolean() ? CoralCollectorConstants.CORAL_HOLD_POWER : CoralCollectorConstants.ALGAE_HOLD_POWER);
    }

    @Override
    public boolean isFinished() {
        if (Robot.isReal() && isCoralMode.getAsBoolean()) {
            // algae mode should never be hit but teeheeee
            return collector.getCollectCurrentHit(isCoralMode.getAsBoolean() ? CoralCollectorConstants.CORAL_COLLECTED_CURRENT : CoralCollectorConstants.ALGAE_COLLECTED_CURRENT);
        } else {
            return false;
        }
    }
}
