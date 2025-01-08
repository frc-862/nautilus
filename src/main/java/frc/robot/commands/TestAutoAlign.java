// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.features2d.FlannBasedMatcher;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestAutoAlign extends Command {
  PhotonVision vision;
  Swerve drivetrain;

  double tx = 0;
  PIDController controller = new PIDController(0.05d, 0, 0);
  double calculatedSpeed;

  public TestAutoAlign(PhotonVision vision, Swerve drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setSetpoint(0);
    controller.setTolerance(0.2d);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tx = vision.getXBestTarget();
    if (Math.abs(tx) > 0.2d) {
      calculatedSpeed = controller.calculate(tx);
      drivetrain.applyRequest(DriveRequests.getRobotCentric(() -> calculatedSpeed, () -> 0, () -> 0));
    } else {
      DataLogManager.log("Auto Align Finished");

    }

    LightningShuffleboard.setDouble("TestAutoAlign", "tX", tx);
    LightningShuffleboard.setDouble("TestAutoAlign", "calculatedspeed", calculatedSpeed);
    
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(tx) > 0.2d;
  }
}
