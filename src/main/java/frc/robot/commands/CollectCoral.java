// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralCollectorConstants;
import frc.robot.subsystems.CoralCollector;
import frc.robot.RobotContainer;

public class CollectCoral extends Command {

    private CoralCollector collector;
    private DoubleSupplier triggerPower;
    private Debouncer debouncer = new Debouncer(CoralCollectorConstants.DEBOUNCE_TIME, Debouncer.DebounceType.kBoth);

    public CollectCoral(CoralCollector collector, DoubleSupplier triggerPower) {
        this.collector = collector;
        this.triggerPower = triggerPower;

        addRequirements(collector);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        collector.setPower(triggerPower.getAsDouble() * CoralCollectorConstants.CORAL_ROLLER_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        collector.stop();
        RobotContainer.hapticCopilotCommand().schedule();
    }

    @Override
    public boolean isFinished() {
        return debouncer.calculate(collector.getBeamBreakOutput());
    }
}
