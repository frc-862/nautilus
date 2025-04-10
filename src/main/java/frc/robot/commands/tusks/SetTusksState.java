// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tusks;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FishingRodConstants.RodStates;
import frc.robot.Constants.TuskConstants.TuskStates;
import frc.robot.subsystems.FishingRod;
import frc.robot.subsystems.Tusks;

public class SetTusksState extends Command {

    private final Tusks tusks;
    private final Supplier<TuskStates> state;
    private final DoubleSupplier rollerPower;

    private boolean withSlow = false;
    private FishingRod rod;
    private BooleanSupplier withRodCheck = () -> false; // check the rod state to determine if the tusks are safe to move

    private boolean hasMovedPivot = false;

    public SetTusksState(Tusks tusks, Supplier<TuskStates> state, DoubleSupplier rollerPower) {
        this.tusks = tusks;
        this.state = state;
        this.rollerPower = rollerPower;

        this.withSlow = false;

        addRequirements(tusks);
    }

    public SetTusksState(Tusks tusks, Supplier<TuskStates> state) {
        this(tusks, state, () -> 0);
    }

    @Override
    public void initialize() {
        hasMovedPivot = false;
    }

    @Override
    public void execute() {
        tusks.setRollerPower(rollerPower.getAsDouble());

        if (!hasMovedPivot) {
            if (rod != null && withRodCheck.getAsBoolean()) {
                switch (rod.getState()) {
                    case STOW, INVERSE_STOW, PROCESSOR, L1: // States that are unsafe for the tusks
                        return;
                    default:
                        break;
                }
            }
            tusks.setPivot(state.get(), withSlow);
            hasMovedPivot = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        tusks.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return tusks.pivotOnTarget();
    }

    public SetTusksState withSlow() {
        this.withSlow = true;

        return this;
    }

    public SetTusksState withRodSafety(FishingRod rod) {
        this.rod = rod;
        this.withRodCheck = () -> true;

        return this;
    }
}
