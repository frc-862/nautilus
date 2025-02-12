// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.DrivetrainConstants.SysIdTestType;
import frc.robot.subsystems.Swerve;

public class SysIdSequence extends SequentialCommandGroup {

    /**
     * Sys Id test sequence (dynamic first, then quazistatic).
     * Starts and ends the SignalLogger
     *
     * @param swerve subsystem
     * @param testType type of test to run
     * @param deadlineSeconds deadline for each command in the sequence
     */
    public SysIdSequence(Swerve swerve, SysIdTestType testType, double deadlineSeconds) {
        addCommands(
            // new InstantCommand(() -> SignalLogger.start()),
            new InstantCommand(() -> DataLogManager.log("\033[0;35m\033[1mSYSID Start " + testType.toString() + "\033[0m")),
            swerve.sysId(testType, Direction.kForward, true).withDeadline(new WaitCommand(deadlineSeconds)),
            swerve.sysId(testType, Direction.kReverse, true).withDeadline(new WaitCommand(deadlineSeconds)),
            swerve.sysId(testType, Direction.kForward, false).withDeadline(new WaitCommand(deadlineSeconds)),
            swerve.sysId(testType, Direction.kReverse, false).withDeadline(new WaitCommand(deadlineSeconds)),
            new InstantCommand(() -> SignalLogger.stop()),
            new InstantCommand(() -> DataLogManager.log("\033[0;35mSYSID End\033[0m"))
        );
    }

    public SysIdSequence(Swerve swerve, SysIdTestType testType) {
        this(swerve, testType, 5d);
    }
}
