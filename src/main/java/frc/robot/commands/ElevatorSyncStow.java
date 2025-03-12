// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FishingRodConstants.RodStates;
import frc.robot.subsystems.Elevator;

public class ElevatorSyncStow extends Command {

    private Elevator elevator;

    public ElevatorSyncStow(Elevator elevator) {
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        // set target position to 0 just in case and stop to make sure we dont move
        elevator.setPosition(0);
        elevator.stop();
    }

    @Override
    public void execute() {
        // move down until we hit current limit
        elevator.setRawPower(ElevatorConstants.BOTTOM_RAW_POWER);
    }

    @Override
    public void end(boolean interrupted) {
        // sync the encoder to zero and go back to stow after
        elevator.setEncoder(0d);
        elevator.setState(RodStates.STOW);
    }

    @Override
    public boolean isFinished() {
        // end on current limit condition
        return elevator.getMotorCurrent() >= ElevatorConstants.BOTTOM_CURRENT;
    }
}
