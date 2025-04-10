// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tusks;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FishingRodConstants.RodStates;
import frc.robot.Constants.FishingRodConstants.RodTransitionStates;
import frc.robot.Constants.TuskConstants.TuskStates;
import frc.robot.subsystems.CoralCollector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FishingRod;
import frc.robot.subsystems.Tusks;
import frc.thunder.shuffleboard.LightningShuffleboard;

/*
 * Premise - Defines the sequential group for the handoff sequence using the tusks
 * 
 * Sequence:
 *  1. Set the rod to the FREE_TUSKS state (higher stow ish)
 *  2. once the elevator is at a safe height, deploy the tusks ASAP
 *  3. Wait for the tusks to be deployed, then move the rod to collect pos ASAP
 * 
 * Written by Kyle Rush (WindowsVistaisCool)
 * 4.10.2025
 */

public class AlgaeHandoff extends SequentialCommandGroup {

    private Tusks tusks;
    private FishingRod rod;
    private Elevator elevator;

    public AlgaeHandoff(Tusks tusks, FishingRod rod, CoralCollector collector, Elevator elevator, DoubleSupplier triggers) {
        this.tusks = tusks;
        this.rod = rod;
        this.elevator = elevator;

        // PROTO: wait command with condition ele > safe dist for tusks
        addCommands(
                new InstantCommand(() -> tusks.setHandoffMode(true)),
                logState("FREE ROD AND DEPLOY"),

                setRod(RodStates.FREE_TUSKS),
                // .alongWith(waitForElevator().andThen(
                        new SetTusksState(tusks, () -> TuskStates.DEPLOYED, triggers)
                                .withDeadline(new WaitCommand(1)),//), // DEADLINE FOR SIM PURPOSES
                logState("ROD DOWN"),
                setRod(RodStates.TUSKS_COLLECT),
                logState("WAIT COLLECT ALGAE"),
                waitForCollect(collector::getAlgaeCurrentHit)
                    .withDeadline(new WaitCommand(1)), // wait for collect
                logState("ROD UP"),
                setRod(RodStates.SOURCE, RodTransitionStates.WITH_WRIST_SLOW),
                logState("TUSKS UP"),
                new SetTusksState(tusks, () -> TuskStates.STOWED).withSlow()
                        .withDeadline(new WaitCommand(1)),
                logState("ALGAE DOWN"),
                setRod(RodStates.PROCESSOR),
                logState("END"),
                new InstantCommand(() -> tusks.setHandoffMode(false))
        );

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
    private Command setRod(RodStates state, RodTransitionStates transition) {
        return new RunCommand(() -> rod.setState(state, transition)) {
            @Override
            public boolean isFinished() {
                return rod.onTarget();
            }
        };
    }

    private Command waitForCollect(BooleanSupplier collected) {
        return new Command() {
            @Override
            public boolean isFinished() {
                return collected.getAsBoolean();
            }
        };
    }

    private Command waitForElevator() {
        return new Command() {
            @Override
            public boolean isFinished() {
                return elevator.getPosition() > 12;
            }
        };
    }

    private Command logState(String s) {
        return new InstantCommand(() -> {
            // if (Robot.isSimulation()) {
            LightningShuffleboard.setString("Tusks", "Handoff", s);
            // }
        });
    }
}
