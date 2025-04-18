// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tusks;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.FishingRodConstants.RodStates;
import frc.robot.Constants.FishingRodConstants.RodTransitionStates;
import frc.robot.Constants.TuskConstants.TuskStates;
import frc.robot.commands.SetRodState;
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

    private DoubleSupplier triggers;

    public AlgaeHandoff(Tusks tusks, FishingRod rod, CoralCollector collector, Elevator elevator, DoubleSupplier triggers) {
        this.tusks = tusks;
        this.rod = rod;
        this.elevator = elevator;
        this.triggers = triggers;

        // PROTO: wait command with condition ele > safe dist for tusks
        addCommands(
                new InstantCommand(() -> tusks.setHandoffMode(true)),
                logState("FREE ROD AND DEPLOY"),

                new ConditionalCommand(new InstantCommand(), setRod(RodStates.FREE_TUSKS), rod::canMoveTusks),
                    // .alongWith(waitForElevator().andThen(
                        new SetTusksState(tusks, () -> TuskStates.DEPLOYED, triggers)
                                .withDeadline(new WaitCommand(2)), // DEADLINE FOR SIM PURPOSES
                logState("ROD DOWN"),
                setRod(RodStates.TUSKS_COLLECT),
                logState("WAIT COLLECT ALGAE"),
                waitForCollect(collector::getAlgaeCurrentHit),
                    // .withDeadline(new WaitCommand(1)), // wait for collect
                logState("ROD UP"),
                setRod(RodStates.LOW, RodTransitionStates.WITH_WRIST_SLOW),
                logState("TUSKS UP"),
                new SetTusksState(tusks, () -> TuskStates.STOWED)
                        .withDeadline(new WaitCommand(2)),
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
        return new RunCommand(() -> rod.setState(state)).until(rod::onTarget);
    }
    private Command setRod(RodStates state, RodTransitionStates transition) {
        return new RunCommand(() -> rod.setState(state, transition)).until(rod::onTarget);
    }

    private Command waitForCollect(BooleanSupplier collected) {
        return new RunCommand(() -> tusks.setRollerPower(triggers.getAsDouble()), tusks).until(collected::getAsBoolean);
    }

    private Command waitForElevator() {
        return new WaitUntilCommand(() -> elevator.getPosition() > 12);
    }

    private Command logState(String s) {
        return new InstantCommand(() -> {
            // if (Robot.isSimulation()) {
            LightningShuffleboard.setString("Tusks", "Handoff", s);
            // }
        });
    }

    public static Command getHandoff(Tusks tusks, FishingRod rod, CoralCollector collector, Elevator elevator, DoubleSupplier triggers) {
        return new Command() {
            SequentialCommandGroup cmd = new SequentialCommandGroup(new AlgaeHandoff(tusks, rod, collector, elevator, triggers));

            @Override
            public void initialize() {
                cmd.schedule();
            }

            @Override
            public void end(boolean interrupted) {
                cmd.cancel();
                if (interrupted) {
                    new SequentialCommandGroup(
                        new SetRodState(rod, RodStates.FREE_TUSKS),
                        new SetTusksState(tusks, () -> TuskStates.STOWED),
                        new SetRodState(rod, RodStates.PROCESSOR)
                    );
                }
            }
        };
    }

}
