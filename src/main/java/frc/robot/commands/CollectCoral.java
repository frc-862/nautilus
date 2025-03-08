// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    private BooleanSupplier useLowHoldPower;

    public CollectCoral(CoralCollector collector, LEDs leds, DoubleSupplier triggerPower, BooleanSupplier useLowHoldPower) {
        this.collector = collector;
        this.leds = leds;
        this.triggerPower = triggerPower;
        this.useLowHoldPower = useLowHoldPower;

        addRequirements(collector);
    }

    @Override
    public void initialize() {
        collector.setPower(triggerPower.getAsDouble());
    }

    @Override
    public void execute() {
        double power = triggerPower.getAsDouble();
        if (power == 0) {
            power = useLowHoldPower.getAsBoolean() ? CoralCollectorConstants.CORAL_HOLD_POWER : CoralCollectorConstants.ALGAE_HOLD_POWER;
        }
        collector.setPower(power * CoralCollectorConstants.CORAL_ROLLER_SPEED);

        if (collector.getCollectCurrentHit() && power >= 0) {
            RobotContainer.hapticCopilotCommand().schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
        collector.setPower(useLowHoldPower.getAsBoolean() ? CoralCollectorConstants.CORAL_HOLD_POWER : CoralCollectorConstants.ALGAE_HOLD_POWER);
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
