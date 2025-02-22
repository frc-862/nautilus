package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.CoralCollectorConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDStates;
import frc.robot.subsystems.CoralCollector;
import frc.robot.subsystems.LEDs;
import frc.robot.RobotContainer;

public class CollectCoral extends Command {

    private CoralCollector collector;
    private LEDs leds;
    private DoubleSupplier triggerPower;

    public CollectCoral(CoralCollector collector, LEDs leds, DoubleSupplier triggerPower) {
        this.collector = collector;
        this.leds = leds;
        this.triggerPower = triggerPower;

        addRequirements(collector);
    }

    @Override
    public void initialize() {
        collector.setPower(triggerPower.getAsDouble());
    }

    @Override
    public void execute() {
        double power = triggerPower.getAsDouble();
        if (triggerPower.getAsDouble() == 0) {
            power = CoralCollectorConstants.HOLD_POWER;
        }
        collector.setPower(power * CoralCollectorConstants.CORAL_ROLLER_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        collector.setPower(CoralCollectorConstants.HOLD_POWER);
        RobotContainer.hapticCopilotCommand().schedule();
        if (!interrupted) {
            leds.strip.enableState(LEDStates.COLLECTED).withDeadline(new WaitCommand(LEDConstants.PULSE_TIME)).schedule();
        }

    }

    @Override
    public boolean isFinished() {
        if (DriverStation.isAutonomous()) {
            return collector.getCollectCurrentHit() && triggerPower.getAsDouble() > 0;
        } else {
            return false;
        }
    }
}
