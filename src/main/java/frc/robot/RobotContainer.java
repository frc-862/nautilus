// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVision;
import frc.thunder.LightningContainer;

public class RobotContainer extends LightningContainer {

    // public Swerve drivetrain;
    public PhotonVision vision;

    @Override
    protected void initializeSubsystems() {
        // drivetrain = getDrivetrain();
        // vision = new PhotonVision();
    }
    
    @Override
    protected void initializeNamedCommands() {
    }

    @Override
    protected void configureButtonBindings() {
    }

    @Override
    protected void configureDefaultCommands() {
    }

    public Command getAutonomousCommand() {
        return null;
    }

    // public Swerve getDrivetrain() {
    //     return drivetrain == null ? TunerConstants.getDrivetrain() : drivetrain;
    // }

}
