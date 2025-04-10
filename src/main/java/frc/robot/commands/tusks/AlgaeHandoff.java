// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tusks;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.Constants.FishingRodConstants.RodStates;
import frc.robot.Constants.LEDConstants.LEDStates;
import frc.robot.Constants.TuskConstants.TuskStates;
import frc.robot.subsystems.FishingRod;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Tusks;
import frc.thunder.shuffleboard.LightningShuffleboard;

/*
 * Premise - Defines the sequential group for the handoff sequence using the tusks
 * 
 * Sequence:
 *  1. Set the rod to the TUSKS_INIT state (higher stow ish)
 * 
 * Written by Kyle Rush (WindowsVistaisCool)
 * 4.10.2025
 */

public class AlgaeHandoff extends SequentialCommandGroup {

    private static Command instance;

    private Tusks tusks;
    private FishingRod rod;

    public AlgaeHandoff(Tusks tusks, FishingRod rod, DoubleSupplier triggers) {
        this.tusks = tusks;
        this.rod = rod;

        // PROTO: wait command with condition ele > safe dist for tusks
        addCommands(
                logState("INIT"),
                setRod(RodStates.TUSKS_INIT),
                logState("PRECOLLECT"),
                new ParallelCommandGroup(
                        new SetTusksState(tusks, () -> TuskStates.DEPLOYED, triggers)
                                .withDeadline(new WaitCommand(0.5)), // DEADLINE FOR SIM PURPOSES
                        setRod(RodStates.TUSKS_PRECOLLECT)),
                logState("TUSKS STOW"),
                // waitForStow(), // Wait for the copilot to pull up the tusks
                new SetTusksState(tusks, () -> TuskStates.STOWED).withSlow()
                        .withDeadline(new WaitCommand(0.5)),
                logState("COLLECT"),
                setRod(RodStates.TUSKS_COLLECT),
                logState("END"));

        addRequirements(rod);
    }

    /**
     * Custom Rod command since the whole group requires rod, we cannot use
     * SetRodState
     * 
     * @param state
     * @return
     */
    private Command setRod(RodStates state) {
        return new RunCommand(() -> rod.setState(state)) {
            @Override
            public boolean isFinished() {
                return rod.onTarget();
            }
        };
    }

    private Command waitForStow() {
        return new Command() {
            @Override
            public boolean isFinished() {
                return tusks.getState() == TuskStates.STOWED;
            }
        };
    }

    private Command logState(String s) {
        return new InstantCommand(() -> {
            if (Robot.isSimulation()) {
                LightningShuffleboard.setString("Tusks", "Handoff", s);
            }
        });
    }

    public static Command getHandoff(Tusks tusks, FishingRod rod, DoubleSupplier triggers) {
        if (instance != null) {
            try {
                instance.cancel();
            } catch (Exception e) {
                System.err.println("Failed to cancel AlgaeHandoff command: " + e.getMessage());
            }
            
            instance = null;
            return new InstantCommand();
        }

        instance = new AlgaeHandoff(tusks, rod, triggers);
        // .raceWith(new Command() {
        //     @Override
        //     public boolean isFinished() {
        //         return button.getAsBoolean(); // pressing the stick button cancels the handoff
        //     }
        // });
        // NOTE: this race is redundant now, but still commented here just in case
        return instance;
    }
}
