// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FishingRodConstants.RodStates;
import frc.robot.Constants.FishingRodConstants.RodTransitionStates;
import frc.robot.subsystems.Elevator;

public class ElevatorSyncStow extends Command {

    private Elevator elevator;

    public ElevatorSyncStow(Elevator elevator) {
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        // set target position to 0 just in case
        elevator.setPosition(0);
        elevator.stop();
    }

    @Override
    public void execute() {
        // move down until we hit current limit
        elevator.setRawPower(-0.25);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setEncoder(0);
        
        elevator.setState(RodStates.STOW);
    }

    @Override
    public boolean isFinished() {
        // end on current limit condition
        return elevator.getMotorCurrent() >= 75d;
    }
}
