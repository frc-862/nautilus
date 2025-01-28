// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.thunder.LightningRobot;

public class Robot extends LightningRobot {

    public Robot() {
        super(new RobotContainer());
    }

    @Override
    public void autonomousInit() {
        super.autonomousInit();
    }

    @Override
    public void teleopInit() {
        super.teleopInit();
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
    }

    @Override
    public void disabledInit() {
        super.disabledInit();

        RobotContainer container = (RobotContainer) getContainer();

        container.drivetrain.applyRequest(DriveRequests.getBrake());
    }
}
