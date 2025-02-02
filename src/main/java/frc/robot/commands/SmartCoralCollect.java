// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CollectorConstants;
import frc.robot.subsystems.Collector;

public class SmartCoralCollect extends Command {

    private Collector collector;

    private DoubleSupplier powerSupplier;

    private Debouncer debouncer = new Debouncer(CollectorConstants.BEAMBREAK_DEBOUNCE);

    public SmartCoralCollect(Collector collector, DoubleSupplier powerSupplier) {
        this.collector = collector;
        this.powerSupplier = powerSupplier;

        addRequirements(collector);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        collector.setPower(powerSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        collector.stop();
    }

    @Override
    public boolean isFinished() {
        return debouncer.calculate(collector.getBeamBreakOutput()) && powerSupplier.getAsDouble() > 0;
    }
}
