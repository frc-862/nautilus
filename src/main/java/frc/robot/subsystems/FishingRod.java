package frc.robot.subsystems;

import java.util.Currency;
import java.util.concurrent.ConcurrentSkipListMap;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FishingRodConstants;
import frc.robot.Constants.FishingRodConstants.*;
import frc.robot.commands.auton.ScoreCoral;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class FishingRod extends SubsystemBase {

    private final Wrist wrist;
    private final Elevator elevator;
    private final CoralCollector collector;

    private RodStates currState = RodStates.DEFAULT;
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

    private boolean coralMode = true;

    public FishingRod(Wrist wrist, Elevator elevator, CoralCollector collector) {
        this.wrist = wrist;
        this.elevator = elevator;

        // this is so cooked but its technically kind of read only
        this.collector = collector;

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
                case WRIST_UP_THEN_ELE: // wrist up, move ele, move wrist
                    wrist.setState(RodStates.STOW);
                    if (wrist.isOnTarget()) {
                        elevator.setState(targetState);
                        if (elevator.isOnTarget()) {
                            transitionState = RodTransitionStates.DEFAULT; // finalize transition
                        }
                    }
                    break;
                case WRIST_DOWN_THEN_ELE: // wrist down, move ele
                    wrist.setState(RodStates.INVERSE_STOW);
                    if (wrist.isOnTarget()) {
                        elevator.setState(targetState);
                        if (elevator.isOnTarget()) {
                            transitionState = RodTransitionStates.DEFAULT; // finalize transition
                        }
                    }
                    break;
                case CORAL_SAFE_ZONE: // transition to L4, moves wrist earlier than onTarget()
                    double targetSafeZone = switch (targetState) {
                        case L2 -> FishingRodConstants.L2_SAFEZONE_ELE;
                        case L3 -> FishingRodConstants.L3_SAFEZONE_ELE;
                        case L4 -> FishingRodConstants.L4_SAFEZONE_ELE;
                        case HIGH -> FishingRodConstants.HIGH_SAFEZONE_ELE;
                        case LOW -> FishingRodConstants.LOW_SAFEZONE_ELE;
                        default -> throw new IllegalArgumentException("Target state not defined");
                    };
                    elevator.setState(targetState);
                    if (elevator.getPosition() > targetSafeZone) {
                        wrist.setState(targetState);
                        if (wrist.isOnTarget()) {
                            transitionState = RodTransitionStates.DEFAULT; // finalize transition
                        }
                    }
                    break;
                case STOW_SAFE_ZONE:// wrist up, move ele, move wrist
                    wrist.setState(targetState);
                    if (wrist.getAngle() > FishingRodConstants.STOW_SAFEZONE_ANGLE) {
                        elevator.setState(targetState);
                        if (elevator.isOnTarget()) {
                            transitionState = RodTransitionStates.DEFAULT; // finalize transition
                        }
                    }
                    break;
                case BARGE_THROW: // wrist up, move ele, move wrist
                    // basically our initial throw position
                    // if (elevator.isOnTarget() && wrist.isOnTarget()) {
                        elevator.setState(targetState);
                        if (elevator.getPosition() > FishingRodConstants.BARGE_THROW_ELE) {
                            wrist.setState(targetState);
                            if(wrist.getAngle() > FishingRodConstants.BARGE_THROW_ANGLE) {
                                CommandScheduler.getInstance().schedule(new ScoreCoral(collector, () -> -1d));
                                transitionState = RodTransitionStates.DEFAULT; // finalize transition
                            }
                        }
                    // }
                    break;
                case DEFAULT, TRITON, WITH_WRIST_SLOW: // all states should end here
                    wrist.setPosition(FishingRodConstants.WRIST_MAP.get(targetState),
                            transitionState == RodTransitionStates.WITH_WRIST_SLOW); // Run with slow wrist if WITH_WRIST_SLOW
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

    public boolean isCoralMode() {
        return coralMode;
    }

    public void setCoralMode(boolean mode) {
        coralMode = mode;
    }

    /**
     * Sets the state of the fishing rod
     *
     * @param state
     */
    public void setState(RodStates state) {
        targetState = state;

        if (targetState == RodStates.BARGE || targetState == RodStates.PROCESSOR) { // slow wrist so we do not flick the
                                                                                    // algae
            transitionState = RodTransitionStates.WITH_WRIST_SLOW;
        } else if (currState.isScoring() || targetState.isScoring()) { // any scoring state wrist up first to not skewer
            if (targetState == RodStates.L1 && (currState == RodStates.STOW || currState == RodStates.INVERSE_STOW)) {
                transitionState = RodTransitionStates.DEFAULT;
            } else {
                transitionState = RodTransitionStates.WRIST_UP_THEN_ELE;
            }
        } else { // default state (i hate triton)
            transitionState = Constants.IS_TRITON ? RodTransitionStates.TRITON : RodTransitionStates.DEFAULT;
        }

        // zero biases to ensure no silliness happens cross-state
        elevatorBias = 0;
        wristBias = 0;
    }

    public void setState(RodStates state, RodTransitionStates transition) {
        targetState = state;
        transitionState = transition;

        // zero biases to ensure no silliness happens cross-state
        elevatorBias = 0;
        wristBias = 0;
    }

    /**
     * biases will only work if no other position is actively being set.
     *
     * @param bias the amount to add to the wrist position
     * @return a command that will add the bias to the wrist position
     */
    public Command addWristBias(double bias) {
        return new InstantCommand(() -> {
            if (Math.signum(bias) != Math.signum(wristBias)) {
                wristBias = 0;
            }
            wristBias += bias;
            wrist.setPosition(wrist.getTargetAngle() + wristBias);
        });
    }

    /**
     * biases will only work if no other position is actively being set.
     *
     * @param bias the amount to add to the elevastor position
     * @return a command that will add the bias to the elevator position
     */
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
     * gets the target state of the fishing rod
     *
     * @return the target state of the fishing rod
     */
    @Logged(importance = Importance.DEBUG)
    public RodStates getTargetState() {
        return targetState;
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
