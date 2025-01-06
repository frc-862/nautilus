// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.thunder.LightningContainer;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

public class RobotContainer extends LightningContainer {

    public Swerve drivetrain;
    public PhotonVision vision;

    private final XboxController driver = new XboxController(ControllerConstants.DRIVER_CONTROLLER);

    @Override
    protected void initializeSubsystems() {
        drivetrain = TunerConstants.createDrivetrain();
        // vision = new PhotonVision();
    }

    @Override
    protected void configureDefaultCommands() {
        drivetrain.setDefaultCommand(drivetrain.applyRequest(DriveRequests.getDrive(()-> -driver.getLeftX(), ()-> -driver.getLeftY(), ()-> driver.getRightX())));
    }


    @Override
    protected void configureButtonBindings() {
    }
    
    @Override
    protected void initializeNamedCommands() {
    }


    

    public Command getAutonomousCommand() {
        return null;
    }
}
