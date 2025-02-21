// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralCollector;

public class ScoreCoral extends SequentialCommandGroup {
    public ScoreCoral(CoralCollector coral) {
        addCommands(
            new InstantCommand(() -> coral.setPower(-1), coral),
            new WaitCommand(0.75),
            new InstantCommand(() -> coral.setPower(0), coral)
        );
    }
}
