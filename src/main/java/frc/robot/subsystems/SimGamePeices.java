// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimGamePeices extends SubsystemBase {

  public Elevator elevator;
  public FishingRod fishingRod;
  public Wrist wrist;
  public Swerve drivetrain;
  public Peice[] peices = new Peice[] {};

  public class Peice{
    protected Pose2d pose;
    protected boolean inRobot;

    public Pose2d getPose(){
      return pose;
    }

    public void setPose(Pose2d pose){
      this.pose = pose;
    }
  }

  public class Coral extends Peice {

    public Coral(Pose2d pose){
      this.pose = super.pose;
    }
  }

  public class Algae extends Peice {
    public Algae(Pose2d pose){
      this.pose = super.pose;
    }
  }

  /** Creates a new SimGamePeices. */
  public SimGamePeices(Elevator elevator, FishingRod fishingRod, Swerve drivetrain, Wrist wrist, 
    Peice[] peices) {

    this.elevator = elevator;
    this.fishingRod = fishingRod;
    this.drivetrain = drivetrain;
    this.wrist = wrist;
    this.peices = peices;
  }

  public SimGamePeices(Elevator elevator, FishingRod fishingRod, Swerve drivetrain, Wrist wrist) {

    this.elevator = elevator;
    this.fishingRod = fishingRod;
    this.drivetrain = drivetrain;
    this.wrist = wrist;
    
    peices = new Peice[] {};
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
