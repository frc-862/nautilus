// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.thunder.LightningContainer;

public class RobotContainer extends LightningContainer {

    public Swerve drivetrain;
    public PhotonVision vision;
    private Telemetry logger;

    private XboxController driver;

    @Override
    protected void initializeSubsystems() {
        drivetrain = TunerConstants.createDrivetrain();
        // vision = new PhotonVision();
        logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
        driver = new XboxController(ControllerConstants.DRIVER_CONTROLLER);
    }

    @Override
    protected void configureDefaultCommands() {
        drivetrain.setDefaultCommand(drivetrain.applyRequest(DriveRequests.getDrive(()-> -driver.getLeftX(), ()-> -driver.getLeftY(), ()-> driver.getRightX())));
        drivetrain.registerTelemetry(logger::telemeterize);
    }


    @Override
    protected void configureButtonBindings() {
        (new Trigger(driver::getAButton)).whileTrue(drivetrain.applyRequest(DriveRequests.getSlow(()-> -driver.getLeftX(), ()-> -driver.getLeftY(), ()-> driver.getRightX())));
        (new Trigger(driver::getBButton)).whileTrue(drivetrain.applyRequest(DriveRequests.getRobotCentric(()-> -driver.getLeftX(), ()-> -driver.getLeftY(), ()-> driver.getRightX())));
        (new Trigger(driver::getXButton)).whileTrue(drivetrain.applyRequest(DriveRequests.getBrake()));

    }
    
    @Override
    protected void initializeNamedCommands() {
    }


    

    public Command getAutonomousCommand() {
        return null;
    }
}
