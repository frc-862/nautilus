// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.FishingRodConstants;
import frc.robot.Constants.FishingRodConstants.states;

public class FishingRod extends SubsystemBase {

    private Wrist wrist;
    private Elevator elevator;
    private states currState;

    /** Creates a new Fishing_Rod. */
    public FishingRod() {
        wrist = new Wrist();
        elevator = new Elevator();
    }

    @Override
    public void periodic() {
        switch(currState) {
            case STOW:
                wrist.setPosition(FishingRodConstants.WRIST_MAP.get(currState));
                elevator.setPosition(FishingRodConstants.ELEVATOR_MAP.get(currState));
                break;
            case L1:
                wrist.setPosition(FishingRodConstants.WRIST_MAP.get(currState));
                elevator.setPosition(FishingRodConstants.ELEVATOR_MAP.get(currState));
                break;
            case L2:
                wrist.setPosition(FishingRodConstants.WRIST_MAP.get(currState));
                elevator.setPosition(FishingRodConstants.ELEVATOR_MAP.get(currState));
                break;
            case L3:
                wrist.setPosition(FishingRodConstants.WRIST_MAP.get(currState));
                elevator.setPosition(FishingRodConstants.ELEVATOR_MAP.get(currState));
                break;
            case L4:
                wrist.setPosition(FishingRodConstants.WRIST_MAP.get(currState));
                elevator.setPosition(FishingRodConstants.ELEVATOR_MAP.get(currState));
                break;
            case SOURCE:
                wrist.setPosition(FishingRodConstants.WRIST_MAP.get(currState));
                elevator.setPosition(FishingRodConstants.ELEVATOR_MAP.get(currState));
                break;
            default:
                break;    
        }
    }

    /**
     * Sets the state of the fishing rod
     * @param state
     */
    public void setState(states state) {
        if(onTarget()) {
            currState = state;
        }
    }

    /**
     * Gets the state of the fishing rod
     * @return the current state of the fishing rod
     */
    public states getState() {
        return currState;
    }

    /**
     * Checks if the whole fishing rod system is on target
     * @return true if the wrist and elevator are on target false otherwise
     */
    public boolean onTarget() {
        return wrist.isOnTarget() && elevator.isOnTarget();
    }
}
