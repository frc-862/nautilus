// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AlgaeCollectorConstants.AlgaePivotStates;
import frc.robot.subsystems.AlgaeCollector;
import frc.robot.subsystems.LEDs;
import frc.robot.Constants.AlgaeCollectorConstants;
import frc.robot.Constants.LEDConstants.LEDStates;
import frc.robot.Constants.LEDConstants;

public class CollectAlgae extends Command {

    private AlgaeCollector collector;
    private LEDs leds;

    private DoubleSupplier triggerPower;

    public CollectAlgae(AlgaeCollector collector, LEDs leds, DoubleSupplier triggerPower) {
        this.collector = collector;
        this.leds = leds;
        this.triggerPower = triggerPower;

        addRequirements(collector);
    }

    @Override
    public void initialize() {
        collector.setPivotState(AlgaePivotStates.DEPLOYED);
    }

    @Override
    public void execute() {
        collector.setRollerPower(triggerPower.getAsDouble() * AlgaeCollectorConstants.ALGAE_ROLLER_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        collector.setPivotState(AlgaePivotStates.STOWED);
        collector.stop();
        if (!interrupted) {
            leds.strip.enableState(LEDStates.COLLECTED).withDeadline(new WaitCommand(LEDConstants.PULSE_TIME)).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return collector.getRollerHit() && triggerPower.getAsDouble() > 0;
    }
}
