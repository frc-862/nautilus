// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tusks;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FishingRodConstants.RodStates;
import frc.robot.Constants.TuskConstants.TuskStates;
import frc.robot.subsystems.Tusks;

public class SetTusksState extends Command {

    private final Tusks tusks;
    private final Supplier<TuskStates> state;
    private final DoubleSupplier rollerPower;

    private boolean withSlow = false;

    public SetTusksState(Tusks tusks, Supplier<TuskStates> state, DoubleSupplier rollerPower) {
        this.tusks = tusks;
        this.state = state;
        this.rollerPower = rollerPower;

        this.withSlow = false;

        addRequirements(tusks);
    }

    @Override
    public void initialize() {
        tusks.setRollerPower(rollerPower.getAsDouble());
        tusks.setPivot(state.get(), withSlow);
    }

    @Override
    public void end(boolean interrupted) {
        tusks.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return tusks.pivotOnTarget() && (state.get() == TuskStates.DEPLOYED ? tusks.getRollerHit() : true);
    }

    public SetTusksState withSlow() {
        this.withSlow = true;

        return this;
    }
}
