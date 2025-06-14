// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.TuskConstants;
import frc.robot.Constants.TuskConstants.TuskStates;
import frc.robot.Robot;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Tusks extends SubsystemBase {

    private ThunderBird pivotMotor;
    private ThunderBird rollerMotor;

    private TuskStates targetPivotState = TuskStates.STOWED;
    private TuskStates currentPivotState = TuskStates.STOWED;

    private final DutyCycleOut dutyControl = new DutyCycleOut(0);
    private final PositionVoltage positionControl = new PositionVoltage(0);

    private boolean handoffMode = false;

    // sim stuff
    private TalonFXSimState rollerMotorSim;
    private TalonFXSimState pivotMotorSim;

    private SingleJointedArmSim pivotSim;
    private LinearSystemSim<N1, N1, N1> rollerSim;

    private DCMotor pivotGearbox;

    private Mechanism2d mech2d;
    private MechanismRoot2d mechRoot;
    private MechanismLigament2d pivotArm;

    public Tusks(ThunderBird pivotMotor, ThunderBird rollerMotor) {
        this.pivotMotor = pivotMotor;
        this.rollerMotor = rollerMotor;

        TalonFXConfiguration pivotMotorConfig = pivotMotor.getConfig();

        pivotMotorConfig.Slot0.kP = TuskConstants.PIVOT_KP;
        pivotMotorConfig.Slot0.kI = TuskConstants.PIVOT_KI;
        pivotMotorConfig.Slot0.kD = TuskConstants.PIVOT_KD;
        pivotMotorConfig.Slot0.kG = TuskConstants.PIVOT_KG;
        pivotMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        pivotMotorConfig.Feedback.RotorToSensorRatio = 1;
        pivotMotorConfig.Feedback.SensorToMechanismRatio = TuskConstants.ENCODER_TO_MECHANISM_RATIO;

        pivotMotor.applyConfig(pivotMotorConfig);

        pivotMotor.setPosition(TuskConstants.STOW_ANGLE);

        if (Robot.isSimulation()) {
            // simulate motors
            pivotMotorSim = new TalonFXSimState(pivotMotor);
            rollerMotorSim = new TalonFXSimState(rollerMotor);

            pivotGearbox = DCMotor.getKrakenX60(1);

            // create physics sims
            pivotSim = new SingleJointedArmSim(pivotGearbox, TuskConstants.PIVOT_GEAR_RATIO,
                    TuskConstants.PIVOT_MOI, TuskConstants.PIVOT_LENGTH,
                    Units.degreesToRadians(TuskConstants.PIVOT_MIN_ANGLE),
                    Units.degreesToRadians(TuskConstants.PIVOT_MAX_ANGLE), true,
                    TuskConstants.PIVOT_START_ANGLE, 0, 1);

            rollerSim = new LinearSystemSim<N1, N1, N1>(LinearSystemId.identifyVelocitySystem(TuskConstants.ROLLER_KV,
                    TuskConstants.ROLLER_KA));

            pivotArm = new MechanismLigament2d("Pivot", 0.25, 90d, 5d, new Color8Bit(Color.kSkyBlue));

            mech2d = new Mechanism2d(0.75, 0.5);
            mechRoot = mech2d.getRoot("Tusk Root", 0.45, 0);
            mechRoot.append(pivotArm);
        }
    }

    @Override
    public void periodic() {
        // if (currentHit()) {
        // // currentPivotState = targetPivotState;
        // stopPivot();
        // }

        // if (currentPivotState != TuskStates.MOVING) {
        // stopPivot();
        // }

        if (currentPivotState != targetPivotState) {
            switch (targetPivotState) {
                case DEPLOYED:
                    setRawPivot(-1);
                    break;
                case STOWED:
                    setRawPivot(1);
                    break;
                }
            if (pivotOnTarget()) {
                currentPivotState = targetPivotState;
                stopPivot();
            }
        }

        LightningShuffleboard.setBool("Tusks", "onTarget", pivotOnTarget());
        LightningShuffleboard.setDouble("Tusks", "pivot motor angle", pivotMotor.getPosition().getValueAsDouble());
        LightningShuffleboard.setString("Tusks", "Target State", getTargetState().toString());
        LightningShuffleboard.setDouble("Tusks", "Pivot Current", pivotMotor.getStatorCurrent().getValueAsDouble());
        LightningShuffleboard.setDouble("Tusks", "Roller Velocity", getRollerVelocity());
        LightningShuffleboard.setDouble("Diagnostic", "Tusks roller motor temp",
                rollerMotor.getDeviceTemp().getValueAsDouble());
        LightningShuffleboard.setDouble("Diagnostic", "Tusks pivot motor temp",
                pivotMotor.getDeviceTemp().getValueAsDouble());
    }

    public void setHandoffMode(boolean handoffMode) {
        this.handoffMode = handoffMode;
    }

    public boolean isHandoffMode() {
        return handoffMode;
    }

    /**
     * Set the power of the pivot motor
     *
     * @param speed
     */
    public void setRawPivot(double speed) {
        pivotMotor.setControl(new DutyCycleOut(speed));
    }

    /**
     * set target state for pivot
     *
     * @param state to set
     */
    public void setPivot(TuskStates state) {
        targetPivotState = state;
        currentPivotState = TuskStates.MOVING;

        // double angle = (switch (state) {
        //     case DEPLOYED -> TuskConstants.DEPLOY_ANGLE;
        //     case STOWED -> TuskConstants.STOW_ANGLE;
        //     default -> pivotMotor.getPosition().getValue().in(Degrees);
        // });
        // pivotMotor.setControl(positionControl.withPosition(angle));
    }

    public void coastPivot() {
        pivotMotor.setControl(new CoastOut());
    }

    public TuskStates getTargetState() {
        return targetPivotState;
    }

    /**
     * @return current angle of pivot in degrees
     */
    public double getPivotAngle() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    /**
     * @return if pivot is on target ( position)
     */
    public boolean pivotOnTarget() {
        if (targetPivotState == TuskStates.DEPLOYED) {
            return pivotMotor.getPosition().getValueAsDouble() < TuskConstants.DEPLOY_ANGLE;
        } else {
            return pivotMotor.getPosition().getValueAsDouble() > TuskConstants.STOW_ANGLE;
        }
    }

    public boolean currentHit() {
        double targetCurrent = targetPivotState == TuskStates.DEPLOYED ? TuskConstants.DEPLOY_CURRENT
                : TuskConstants.STOW_CURRENT;
        return pivotMotor.getStatorCurrent().getValueAsDouble() > targetCurrent;
    }

    /**
     * Stops the pivot motor
     */
    public void stopPivot() {
        pivotMotor.setControl(new DutyCycleOut(0d));
    }

    /**
     * Set the power of the roller motor
     *
     * @param power
     */
    public void setRollerPower(double power) {
        rollerMotor.setControl(new DutyCycleOut(power * TuskConstants.ROLLER_SPEED));
    }

    public Command runRoller(DoubleSupplier power) {
        return run(() -> setRollerPower(power.getAsDouble()));
    }

    /**
     * @return if the roller is stalling due to algae in the collector
     */
    public boolean getRollerHit() {
        return rollerMotor.getStatorCurrent().getValueAsDouble() >= TuskConstants.ROLLER_CURRENT;
    }

    /**
     * @return current velocity of roller in rot/sec
     */
    public double getRollerVelocity() {
        return rollerMotor.getVelocity().getValueAsDouble();
    }

    /**
     * @return current power of roller
     */
    public double getRollerPower() {
        return rollerMotor.get();
    }

    /**
     * stops the roller
     */
    public void stopRoller() {
        rollerMotor.setControl(new DutyCycleOut(0d));
    }

    @Override
    public void simulationPeriodic() {
        final double batteryVoltage = RobotController.getBatteryVoltage();

        // set supply voltages to motors
        rollerMotorSim.setSupplyVoltage(batteryVoltage);
        pivotMotorSim.setSupplyVoltage(batteryVoltage);

        double pivotAngle = Units.radiansToRotations(pivotSim.getAngleRads());
        double pivotVelocity = Units.radiansToRotations(pivotSim.getVelocityRadPerSec());

        double rollerVelocity = rollerSim.getOutput(0);

        // set simulated motor states
        pivotMotorSim.setRawRotorPosition(pivotAngle);
        pivotMotorSim.setRotorVelocity(pivotVelocity);

        rollerMotorSim.setRotorVelocity(rollerVelocity);

        // use motor voltages to set input voltages for physics simulations
        pivotSim.setInputVoltage(pivotMotorSim.getMotorVoltage());
        rollerSim.setInput(rollerMotorSim.getMotorVoltage());

        // update physics simulations
        pivotSim.update(RobotMap.UPDATE_FREQ);
        rollerSim.update(RobotMap.UPDATE_FREQ);

        pivotArm.setAngle(pivotMotor.getPosition().getValue().in(Degrees));

        LightningShuffleboard.send("Tusks", "Mech2d", mech2d);
    }
}
