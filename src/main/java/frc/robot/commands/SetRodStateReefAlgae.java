// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PoseConstants;
import frc.robot.subsystems.FishingRod;
import frc.robot.subsystems.Swerve;

public class SetRodStateReefAlgae extends Command {
    
    private Swerve swerve;
    private FishingRod rod;

    /**
     * Automatically determines the HIGH/LOW position to transition to
     * and sets the rod state
     * @param swerve subsystem for pose only
     * @param rod fishing rod
     */
    public SetRodStateReefAlgae(Swerve swerve, FishingRod rod) {
        this.swerve = swerve;
        this.rod = rod;

        addRequirements(rod); // only add requirements for rod since we don't move swerve
    }

    @Override
    public void initialize() {
        rod.setState(PoseConstants.getAlgaeScoreState(swerve, rod));
    }

    @Override
    public boolean isFinished() {
        return DriverStation.isAutonomous() ? rod.onTarget() : false;
    }
}
