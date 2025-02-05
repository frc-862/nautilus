// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FishingRodConstants.ROD_STATES;
import frc.robot.subsystems.FishingRod;

public class SetRodState extends Command {
    private FishingRod rod;
    private ROD_STATES state;

    
    /** Creates a new SetRodState. 
     * @param rod rod subsystem
     * @param state state to set
    */
    public SetRodState(FishingRod rod, ROD_STATES state) {
        this.rod = rod;
        this.state = state;

        addRequirements(rod);
    }

    @Override
    public void execute() {
        rod.setState(state);
    }

    @Override
    public boolean isFinished() {
        if(DriverStation.isAutonomous()) {
            return rod.onTarget();
        } else {
            return false;
        }
    }
}
