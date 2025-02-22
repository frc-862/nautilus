// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.thunder.LightningRobot;

public class Robot extends LightningRobot {

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

        RobotContainer container = (RobotContainer) getContainer();
        container.leds.pdhLedsBlink(container.pdh);
    }

    @Override
    public void teleopInit() {
        super.teleopInit();

        RobotContainer container = (RobotContainer) getContainer();
        container.leds.pdhLedsBlink(container.pdh);
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
    }

    @Override
    public void disabledInit() {
        super.disabledInit();

        RobotContainer container = (RobotContainer) getContainer();

        container.drivetrain.setControl(DriveRequests.getBrake().get());
        container.leds.pdhLedsSolid(container.pdh);
    }
}