// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.io.IOException;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.Constants.RobotIdentifiers;
import frc.thunder.LightningRobot;

public class Robot extends LightningRobot {

    private RobotContainer container;

    private Command ledCmd;

    public Robot() {
        super(new RobotContainer());
    }

    @Override
    public void robotInit() {
        super.robotInit();

        if (Robot.isReal()) {
            try {
                Runtime.getRuntime().exec("ssh pi@10.8.62.15 sudo systemctl restart photonvision");
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public void autonomousInit() {
        super.autonomousInit();

        container = (RobotContainer) getContainer();
        container.drivetrain.setOperatorPerspectiveForward(
                new Rotation2d(Degrees.of(DriverStation.getAlliance().get() == Alliance.Red ? 180 : 0)));
            
        container.drivetrain.setTakingVision(true);

        if (Constants.ROBOT_IDENTIFIER == RobotIdentifiers.NAUTILUS) {
            ledCmd = new RepeatCommand(
                    new InstantCommand(() -> container.pdh.setSwitchableChannel(!container.pdh.getSwitchableChannel()))
                            .alongWith(new WaitCommand(0.4)));
            ledCmd.schedule();
        }
    }

    @Override
    public void teleopInit() {
        super.teleopInit();

        container = (RobotContainer) getContainer();
        container.drivetrain.setOperatorPerspectiveForward(new Rotation2d(
                Degrees.of(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 180 : 0)));
        
        container.drivetrain.setTakingVision(true);

        if (Constants.ROBOT_IDENTIFIER == RobotIdentifiers.NAUTILUS) {
            ledCmd = new RepeatCommand(
                    new InstantCommand(() -> container.pdh.setSwitchableChannel(!container.pdh.getSwitchableChannel()))
                            .alongWith(new WaitCommand(0.2)));
            ledCmd.schedule();
        }
    }

    @Override
    public void disabledInit() {
        super.disabledInit();

        container = (RobotContainer) getContainer();

        container.drivetrain.setControl(DriveRequests.getBrake().get());

        if (Constants.ROBOT_IDENTIFIER == RobotIdentifiers.NAUTILUS) {
            if (ledCmd != null) {
                if (ledCmd.isScheduled()) {
                    ledCmd.cancel();
                }

            }
            container.pdh.setSwitchableChannel(true);
        }
    }

    @Override
    public void disabledPeriodic() {
        super.disabledPeriodic();

        // if (container.rod.getState() == RodStates.STOW) {
        //     container.elevator.setEncoder(container.elevator.getCANRangeDist());
        // }
    }

}
