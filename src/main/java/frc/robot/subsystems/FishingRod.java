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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FishingRodConstants;
import frc.robot.Constants.FishingRodConstants.*;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class FishingRod extends SubsystemBase {

    private Wrist wrist;
    private Elevator elevator;

    private RodStates currState = RodStates.STOW;
    private RodStates targetState = RodStates.STOW;
    private RodTransitionStates transitionState = RodTransitionStates.DEFAULT;

    private double wristBias = 0;
    private double elevatorBias = 0;

    // simulation stuff
    private Mechanism2d mech2d;
    private MechanismRoot2d root;
    private MechanismLigament2d stage1;
    private MechanismLigament2d stage2;
    private MechanismLigament2d stage3;
    private MechanismLigament2d wristSim;

    public FishingRod(Wrist wrist, Elevator elevator) {
        this.wrist = wrist;
        this.elevator = elevator;

        // simulation stuff
        if (Robot.isSimulation()) {
            stage1 = new MechanismLigament2d("STAGE 1", ElevatorConstants.CUSHION_METERS, 90, 20,
                    new Color8Bit(Color.kSeaGreen));
            stage2 = new MechanismLigament2d("STAGE 2", ElevatorConstants.CUSHION_METERS, 0, 15,
                    new Color8Bit(Color.kDarkRed));
            stage3 = new MechanismLigament2d("STAGE 3", ElevatorConstants.CUSHION_METERS, 0, 10,
                    new Color8Bit(Color.kDarkBlue));

            wristSim = new MechanismLigament2d("WRIST SIM", WristConstants.LENGTH.magnitude(), 90d, 5d,
                    new Color8Bit(Color.kWhite));

            mech2d = new Mechanism2d(0.69, 2.29);
            root = mech2d.getRoot("Rod root", 0.35, 0);

            root.append(stage1).append(stage2).append(stage3).append(wristSim);
        }
    }

    @Override
    public void periodic() {
        // if the rod hasn't reached target state
        if (!onTarget()) {
            switch (transitionState) {
                case SCORE_X: // wrist up, move ele, move wrist
                    wrist.setState(RodStates.STOW);
                    if (wrist.isOnTarget()) {
                        transitionState = RodTransitionStates.TRANSITIONING; // finalize transition
                    }
                    break;
                case X_SCORE: // wrist down, move ele
                    wrist.setState(targetState);
                    if (wrist.isOnTarget()) {
                        transitionState = RodTransitionStates.TRANSITIONING; // finalize transition
                    }
                    break;
                case DEFAULT, TRITON, TRANSITIONING: // all states should end here
                    wrist.setPosition(FishingRodConstants.WRIST_MAP.get(targetState));
                    elevator.setPosition(FishingRodConstants.ELEVATOR_MAP.get(targetState));
                    if (wrist.isOnTarget() && elevator.isOnTarget()) {
                        currState = targetState;
                    }
                    break;

                default:
                    throw new IllegalArgumentException("transition state not defined");
            }
        }

        LightningShuffleboard.setBool("Rod", "onTarget", onTarget());
        LightningShuffleboard.setString("Rod", "currentState", currState.toString());
        LightningShuffleboard.setString("Rod", "targetState", targetState.toString());
        LightningShuffleboard.setString("Rod", "transitionState", transitionState.toString());
    }

    /**
     * Sets the state of the fishing rod
     *
     * @param state
     */
    public void setState(RodStates state) {
        targetState = state;

        // logic for transition states goes here
        if (currState == RodStates.L4 || currState == RodStates.L3 || currState == RodStates.L2 || targetState == RodStates.L4) { //TODO: temporary fix don't do this
            transitionState = RodTransitionStates.SCORE_X;
        } else if (targetState == RodStates.L3 || targetState == RodStates.L2) {
            transitionState = RodTransitionStates.X_SCORE;
        } else {
            transitionState = Constants.IS_TRITON ? RodTransitionStates.TRITON : RodTransitionStates.DEFAULT;
        }

        // zero biases to ensure no silliness happens cross-state
        elevatorBias = 0;
        wristBias = 0;
    }

    // biases will only work if no other position is actively being set.
    public Command addWristBias(double bias) {
        return new InstantCommand(() -> {
            if (Math.signum(bias) != Math.signum(wristBias)) {
                wristBias = 0;
            }
            wristBias += bias;
            wrist.setPosition(wrist.getTargetAngle() + wristBias);
        });
    }

    public Command addElevatorBias(double bias) {
        return new InstantCommand(() -> {
            if (Math.signum(bias) != Math.signum(elevatorBias)) {
                elevatorBias = 0;
            }
            elevatorBias += bias;
            elevator.setPosition(elevator.getTargetPosition() + elevatorBias);
        });
    }

    /**
     * Gets the state of the fishing rod
     *
     * @return the current state of the fishing rod
     */

    @Logged(importance = Importance.CRITICAL)
    public RodStates getState() {
        return currState;
    }

    /**
     * Checks if the whole fishing rod system is on target
     *
     * @return true if the wrist and elevator are on target false otherwise
     */
    @Logged(importance = Importance.DEBUG)
    public boolean onTarget() {
        return currState == targetState;
    }

    @Override
    public void simulationPeriodic() {
        double stageLen = Units.inchesToMeters(elevator.getPosition());

        if (stageLen < ElevatorConstants.STAGE_LEN_METERS) {
            stage1.setLength(stageLen + ElevatorConstants.CUSHION_METERS);
            stage2.setLength(ElevatorConstants.CUSHION_METERS);
            stage3.setLength(ElevatorConstants.CUSHION_METERS);
        } else if (stageLen < ElevatorConstants.STAGE_LEN_METERS * 2) {
            stage2.setLength(stageLen - ElevatorConstants.STAGE_LEN_METERS + ElevatorConstants.CUSHION_METERS);
            stage3.setLength(ElevatorConstants.CUSHION_METERS);
        } else {
            stage3.setLength(stageLen - ElevatorConstants.STAGE_LEN_METERS * 2 + ElevatorConstants.CUSHION_METERS);
        }

        wristSim.setAngle(wrist.getAngle() - 90);

        LightningShuffleboard.send("Rod", "Mech2d", mech2d);
    }

}
