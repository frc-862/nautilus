// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FishingRodConstants.RodStates;
import frc.robot.Constants.FishingRodConstants.RodTransitionStates;
import frc.robot.subsystems.FishingRod;

public class SetRodState extends Command {

    private final FishingRod rod;
    private final RodStates state;

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
        if (rod.getState() == state && state != RodStates.STOW && state != RodStates.INVERSE_STOW) {
            return;
        }

        switch (state) {
            case STOW:
                RodStates desiredStow = RodStates.STOW;

                // Stow based on the current state of the rod
                switch (rod.getState()) {
                    case L2, L3, L4, BARGE:
                        rod.setState(desiredStow, RodTransitionStates.DEFAULT);
                        break;
                    default:
                        rod.setState(desiredStow, RodTransitionStates.WRIST_UP_THEN_ELE);
                        break;
                }
                break;
            case INVERSE_STOW:
                rod.setState(state, RodTransitionStates.WRIST_UP_THEN_ELE);
                break;
            case L2, L3, L4:
                if (rod.getState() == RodStates.STOW || state == RodStates.L4) {
                    rod.setState(state, RodTransitionStates.CORAL_SAFE_ZONE);
                } else if (rod.getState() == RodStates.INVERSE_STOW) {
                    rod.setState(state, RodTransitionStates.WRIST_DOWN_THEN_ELE);
                } else {
                    rod.setState(state);
                }
                break;
            default:
                rod.setState(state);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return rod.onTarget();
    }
}
