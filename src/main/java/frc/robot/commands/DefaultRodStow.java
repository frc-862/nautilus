// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FishingRodConstants.RodStates;
import frc.robot.Constants.FishingRodConstants.RodTransitionStates;
import frc.robot.Constants.PoseConstants.StowZone;
import frc.robot.subsystems.FishingRod;
import frc.robot.subsystems.Swerve;

public class DefaultRodStow extends Command {

    private final Swerve drivetrain;
    private final FishingRod rod;

    private RodStates desiredState = RodStates.STOW;
    private RodTransitionStates desiredTransition = RodTransitionStates.WRIST_UP_THEN_ELE;
    private StowZone currentZone = StowZone.SAFE;
    private StowZone lastZone = StowZone.SAFE;

    /**
     * Default Rod behavior for teleop
     * Determines whether the rod in stow should
     * be up or down in the current StowZone
     *
     * @param drivetrain subsystem
     * @param rod        subsystem
     */
    public DefaultRodStow(Swerve drivetrain, FishingRod rod) {
        this.drivetrain = drivetrain;
        this.rod = rod;

        addRequirements(rod);
    }

    @Override
    public void execute() {
        // Return if autonomous
        if (DriverStation.isAutonomous()) {
            return;
        }

        // Return if we are not in either stow position
        if (rod.getState() != RodStates.STOW && rod.getState() != RodStates.INVERSE_STOW) {
            return;
        }

        desiredState = RodStates.STOW;
        desiredTransition = RodTransitionStates.WRIST_UP_THEN_ELE;

        // current must be called before last to update the last position
        currentZone = drivetrain.getCurrentStowZone();
        lastZone = drivetrain.getLastStowZone();

        // SOURCE -> SAFE
        if (lastZone == StowZone.SOURCE && currentZone == StowZone.SAFE) {
            desiredState = RodStates.INVERSE_STOW;
            desiredTransition = RodTransitionStates.DEFAULT;
        }
        // SAFE -> SOURCE
        else if (lastZone == StowZone.SAFE && currentZone == StowZone.SOURCE) {
            desiredState = RodStates.STOW;
            desiredTransition = RodTransitionStates.DEFAULT;
        }
        // REEF -> SAFE
        else if (lastZone == StowZone.REEF && currentZone == StowZone.SAFE) {
            desiredState = RodStates.STOW;
            desiredTransition = RodTransitionStates.DEFAULT;
        }
        // SAFE -> REEF
        else if (lastZone == StowZone.SAFE && currentZone == StowZone.REEF) {
            // If in the reef zone, do not change the stow position
            return;
        }

        // If we are not already targetting the desired state, we will apply the new
        // target
        if (desiredState != rod.getTargetState()) {
            rod.setState(desiredState, desiredTransition);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
