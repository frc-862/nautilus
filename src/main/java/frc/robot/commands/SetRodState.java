// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FishingRodConstants.states;
import frc.robot.subsystems.FishingRod;

public class SetRodState extends Command {
    private FishingRod rod;
    private states state;

    
    /** Creates a new SetRodState. 
     * @param rod rod subsystem
     * @param state state to set
    */
    public SetRodState(FishingRod rod, states state) {
        this.rod = rod;
        this.state = state;

        addRequirements(rod);
    }

    @Override
    public void initialize() {
        rod.setState(state);
    }

    @Override
    public boolean isFinished() {
        return rod.onTarget();
    }
}
