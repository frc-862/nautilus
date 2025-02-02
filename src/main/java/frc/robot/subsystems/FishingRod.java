// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FishingRodConstants;
import frc.robot.Constants.FishingRodConstants.states;
import frc.robot.Constants.WristConstants;
import frc.robot.Robot;

public class FishingRod extends SubsystemBase {

    private Wrist wrist;
    private Elevator elevator;
    private states currState = states.STOW;
    private states targetState = states.STOW;

    private double wristBias = 0;
    private double elevatorBias = 0;

    //simulation stuff
    private Mechanism2d mech2d;
    private MechanismRoot2d root;
    private MechanismLigament2d stage1;
    private MechanismLigament2d stage2;
    private MechanismLigament2d stage3;
    private MechanismLigament2d wristSim;
    

    public FishingRod(Wrist wrist, Elevator elevator) {
        this.wrist = wrist;
        this.elevator = elevator;

        //simulation stuff
        if (Robot.isSimulation()) {
            stage1 = new MechanismLigament2d("STAGE 1", ElevatorConstants.CUSHION_METERS, 90, 20, new Color8Bit(Color.kSeaGreen));
            stage2 = new MechanismLigament2d("STAGE 2", ElevatorConstants.CUSHION_METERS, 0, 15, new Color8Bit(Color.kDarkRed));
            stage3 = new MechanismLigament2d("STAGE 3", ElevatorConstants.CUSHION_METERS, 0, 10, new Color8Bit(Color.kDarkBlue));

            wristSim = new MechanismLigament2d("WRIST SIM", WristConstants.LENGTH.magnitude(), 90d, 5d, new Color8Bit(Color.kWhite));
         
            mech2d = new Mechanism2d(0.69, 2.29);
            root = mech2d.getRoot("rod root", 0.35, 0);
            
            root.append(stage1).append(stage2).append(stage3).append(wristSim);

        }
    }

    @Override
    public void periodic() {
        if(onTarget()) {
            currState = targetState;
        }

        // if(currState != targetState) {
            switch(targetState) {
                case STOW, L1, L2, L3, SOURCE:
                    wrist.setPosition(FishingRodConstants.WRIST_MAP.get(targetState) + wristBias);
                    elevator.setPosition(FishingRodConstants.ELEVATOR_MAP.get(targetState) + elevatorBias);
                    break;
                // case L1:
                //     wrist.setPosition(FishingRodConstants.WRIST_MAP.get(targetState));
                //     elevator.setPosition(FishingRodConstants.ELEVATOR_MAP.get(targetState));
                //     break;
                // case L2:
                //     wrist.setPosition(FishingRodConstants.WRIST_MAP.get(targetState));
                //     elevator.setPosition(FishingRodConstants.ELEVATOR_MAP.get(targetState));
                //     break;
                // case L3:
                //     wrist.setPosition(FishingRodConstants.WRIST_MAP.get(targetState));
                //     elevator.setPosition(FishingRodConstants.ELEVATOR_MAP.get(targetState));
                //     break;
                case L4:
                    elevator.setPosition(FishingRodConstants.ELEVATOR_MAP.get(targetState) + elevatorBias);
                    if(elevator.isOnTarget()) {
                        wrist.setPosition(FishingRodConstants.WRIST_MAP.get(targetState) + wristBias);
                    }
                    break;
                // case SOURCE:
                //     wrist.setPosition(FishingRodConstants.WRIST_MAP.get(targetState));
                //     elevator.setPosition(FishingRodConstants.ELEVATOR_MAP.get(targetState));
                //     break;
                default:
                    break;    
            }
        // }
    }

    /**
     * Sets the state of the fishing rod
     * @param state
     */
    public void setState(states state) {
        targetState = state;

        elevatorBias = 0;
        wristBias = 0;
    }

    public Command addWristBias(double bias) {
        return runOnce(() -> wristBias += bias);
    }

    public Command addElevatorBias(double bias) {
        return runOnce(() -> elevatorBias += bias);
    }

    public Command resetWristBias() {
        return runOnce(() -> wristBias = 0);
    }

    public Command resetElevatorBias() {
        return runOnce(() -> elevatorBias = 0);
    }

    /**
     * Gets the state of the fishing rod
     * @return the current state of the fishing rod
     */

    @Logged(importance = Importance.CRITICAL)
    public states getState() {
        return currState;
    }

    /**
     * Checks if the whole fishing rod system is on target
     * @return true if the wrist and elevator are on target false otherwise
     */
    @Logged(importance = Importance.DEBUG)
    public boolean onTarget() {
        return wrist.isOnTarget() && elevator.isOnTarget();
    }

    @Override
    public void simulationPeriodic() {
        double stageLen = Units.inchesToMeters(elevator.getPosition());

        if(stageLen < ElevatorConstants.STAGE_LEN_METERS) {
            stage1.setLength(stageLen + ElevatorConstants.CUSHION_METERS);
            stage2.setLength(ElevatorConstants.CUSHION_METERS);
            stage3.setLength(ElevatorConstants.CUSHION_METERS);
        } else if(stageLen < ElevatorConstants.STAGE_LEN_METERS * 2) {
            stage2.setLength(stageLen  - ElevatorConstants.STAGE_LEN_METERS + ElevatorConstants.CUSHION_METERS);
            stage3.setLength(ElevatorConstants.CUSHION_METERS);
        } else {
            stage3.setLength(stageLen  - ElevatorConstants.STAGE_LEN_METERS * 2 + ElevatorConstants.CUSHION_METERS);
        }

        wristSim.setAngle(wrist.getAngle()-90);

        LightningShuffleboard.send("fishing rod", "Mech2d", mech2d);
    }

}
