package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralCollectorConstants;
import frc.robot.subsystems.CoralCollector;

public class IntakeCoral extends Command {

    private CoralCollector collector;
    private double power;

    public IntakeCoral(CoralCollector collector, double power) {
        this.collector = collector;
        this.power = power;

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
        collector.setPower(CoralCollectorConstants.HOLD_POWER);
    }

    @Override
    public boolean isFinished() {
        return collector.getCollectCurrentHit();
    }
}
