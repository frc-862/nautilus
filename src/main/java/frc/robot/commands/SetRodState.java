// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.FishingRodConstants.RodStates;
import frc.robot.Constants.FishingRodConstants.RodTransitionStates;
import frc.robot.subsystems.FishingRod;

public class SetRodState extends Command {

    private FishingRod rod;
    private RodStates state;

    /**
     * Creates a new SetRodState.
     *
     * @param rod   rod subsystem
     * @param state state to set
     */
    public SetRodState(FishingRod rod, RodStates state) {
        this.rod = rod;
        this.state = state;

        addRequirements(rod);
    }

    @Override
    public void initialize() {

        // If our current state is the same as the state we want to set, return
        // Stow regardless in case we get stuck somewhere
        if (rod.getState() == state && state != RodStates.STOW) {
            return;
        }

        switch (state) {
            case STOW:
                // Use fast stow if in L4 or Barge otherwise use default stow
                if (rod.getState() == RodStates.L4 || rod.getState() == RodStates.BARGE) {
                    rod.setState(state, RodTransitionStates.DEFAULT);
                } else if (rod.getState() == RodStates.L2 || rod.getState() == RodStates.L3) {
                    rod.setState(state, RodTransitionStates.DEFAULT);
                } else {
                    rod.setState(state);
                }
                break;
            case L2, L3:
                if (rod.getState() == RodStates.STOW) {
                    rod.setState(state, RodTransitionStates.CORAL_SAFE_ZONE);
                } else {
                    rod.setState(state);
                }
                break;
            case L4:
                rod.setState(state, RodTransitionStates.CORAL_SAFE_ZONE);
                break;
            default:
                rod.setState(state);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return DriverStation.isAutonomous() ? rod.onTarget() : false;
    }
}
