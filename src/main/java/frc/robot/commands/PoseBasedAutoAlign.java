// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.PoseConstants.poses;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PoseBasedAutoAlign extends Command {
  /** Creates a new PoseBasedAutoAlign. */

  Command pathCommand;
  Swerve drivetrain;
  Pose2d pose;
  poses poseEnum; 

  /**
   * Move to pose using path
 * @param drivetrain
 * @param pose
 */
public PoseBasedAutoAlign(Swerve drivetrain, Pose2d pose) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.drivetrain = drivetrain;
    this.pose = pose;

    addRequirements(drivetrain);
  }

  /**
   * Move to pose using path
   * @param drivetrain
   * @param enum from PoseConstants
   */
  public PoseBasedAutoAlign(poses poseEnum, Swerve drivetrain){

    this.drivetrain = drivetrain;
    this.poseEnum = poseEnum;
    this.pose = PoseConstants.poseHashMap.get(poseEnum);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // create a path command using a pose2d

    pathCommand = AutoBuilder.pathfindToPose(pose, PoseConstants.PATHFINDING_CONSTRAINTS);

    // schedule that pathcommand

    pathCommand.schedule();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
