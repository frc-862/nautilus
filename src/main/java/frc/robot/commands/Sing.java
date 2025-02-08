// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class Sing extends Command {
  
  private Swerve drivetrain;
  private Orchestra sing;
  
  /** Creates a new Sing. */
  public Sing(Orchestra sing, Swerve drivetrain) {
    this.drivetrain = drivetrain;
    sing = new Orchestra();
    
  }

  @Override
  public void initialize() {
        sing.clearInstruments();
        sing.stop();
        for (int i = 0; i < 4; i++) {
            sing.addInstrument(drivetrain.getModule(i).getDriveMotor());
            sing.addInstrument(drivetrain.getModule(i).getSteerMotor());
        }
        sing.loadMusic("music.chrp");
        sing.play();


  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
      sing.stop();
      sing.clearInstruments();
  }
  @Override
  public boolean isFinished() {
    return false;
  }
}
