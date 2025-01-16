// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.PoseConstants.ScoringPoses;

public class PoseBasedAutoAlign extends Command {

    Command pathCommand;
    Swerve drivetrain;
    Pose2d pose;
    ScoringPoses poseEnum; 

    /**
     * Move to pose using path
     * @param drivetrain
     * @param pose
     */
    public PoseBasedAutoAlign(Swerve drivetrain, Pose2d pose) {

        this.drivetrain = drivetrain;
        this.pose = pose;

        addRequirements(drivetrain);
    }

    /**
     * Move to pose using path
     * @param poseEnum
     * @param drivetrain from PoseConstants
     */
    public PoseBasedAutoAlign(Swerve drivetrain, ScoringPoses poseEnum){

        this.drivetrain = drivetrain;
        this.poseEnum = poseEnum;
        this.pose = PoseConstants.poseHashMap.get(poseEnum);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

        // create a path command using a pose2d

        pathCommand = AutoBuilder.pathfindToPose(pose, PoseConstants.PATHFINDING_CONSTRAINTS);

        // schedule that pathcommand

        pathCommand.schedule();

    }

    @Override
    public void end(boolean interrupted) {
        pathCommand.cancel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
