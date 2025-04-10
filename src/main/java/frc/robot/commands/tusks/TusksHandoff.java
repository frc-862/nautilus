// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tusks;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FishingRodConstants.RodStates;
import frc.robot.Constants.TuskConstants.TuskStates;
import frc.robot.subsystems.FishingRod;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Tusks;

public class TusksHandoff extends SequentialCommandGroup {

    private Tusks tusks;
    private FishingRod rod;

    public TusksHandoff(Tusks tusks, FishingRod rod, LEDs leds, DoubleSupplier joystick) {
        this.tusks = tusks;
        this.rod = rod;

        addCommands(
            setRod(RodStates.STOW),
            new SetTusksState(tusks, () -> TuskStates.DEPLOYED, () -> 0.5),
            new SetTusksState(tusks, () -> TuskStates.STOWED, () -> 0).withSlow(),
            setRod(RodStates.INVERSE_STOW)
        );

        addRequirements(rod);
    }

    private Command setRod(RodStates state) {
        return new InstantCommand(() -> rod.setState(state));
    }

    public static ParallelDeadlineGroup getHandoff(Tusks tusks, FishingRod rod, LEDs leds, DoubleSupplier joystick) {
        return new TusksHandoff(tusks, rod, leds, joystick).withDeadline(new Command() {
            @Override
            public boolean isFinished() {
                return joystick.getAsDouble() > 0.15;
            }
        });
    }

}
