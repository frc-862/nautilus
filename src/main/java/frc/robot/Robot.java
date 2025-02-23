// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.thunder.LightningRobot;

public class Robot extends LightningRobot {

    private RobotContainer container;

    public Robot() {
        super(new RobotContainer());
    }

    @Override
    public void robotInit() {
        super.robotInit();

        // WebServer.start(5800, Filesystem.getDeployDirectory().getPath() + "/elastic");
    }

    @Override
    public void autonomousInit() {
        super.autonomousInit();

        container = (RobotContainer) getContainer();
    }
    
    @Override
    public void autonomousPeriodic() {
        container.leds.pdhLedsBlink(container.pdh, 0.25d);
    }

    @Override
    public void teleopInit() {
        super.teleopInit();

        container = (RobotContainer) getContainer();
    }

    @Override
    public void teleopPeriodic() {
        container.leds.pdhLedsBlink(container.pdh, 0.75d);
    }

    @Override
    public void disabledInit() {
        super.disabledInit();

        container = (RobotContainer) getContainer();

        container.drivetrain.setControl(DriveRequests.getBrake().get());
        container.leds.pdhLedsSolid(container.pdh);
    }
}