// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.ctre.phoenix6.sim.CANrangeSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FishingRodConstants;
import frc.robot.Constants.FishingRodConstants.RodStates;
import frc.robot.Constants.RobotMap;
import frc.robot.Robot;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Elevator extends SubsystemBase {

    private ThunderBird leftMotor;
    @SuppressWarnings("unused")
    private ThunderBird rightMotor;
    private CANrange rangeSensor;

    private double targetPosition = 0;
    private double currentPosition = 0;

    private MotionMagicVoltage positionPID;

    // sim stuff
    private DCMotor gearbox;
    private ElevatorSim elevatorSim;
    private TalonFXSimState leftSim;
    private TalonFXSimState rightSim;
    private CANrangeSimState rangeSensorSim;

    public Elevator(ThunderBird leftMotor, ThunderBird rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;

        TalonFXConfiguration config = leftMotor.getConfig();
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.kP = ElevatorConstants.MOTORS_KP;
        config.Slot0.kI = ElevatorConstants.MOTORS_KI;
        config.Slot0.kD = ElevatorConstants.MOTORS_KD;
        config.Slot0.kS = ElevatorConstants.MOTORS_KS;
        config.Slot0.kV = ElevatorConstants.MOTORS_KV;
        config.Slot0.kA = ElevatorConstants.MOTORS_KA;
        config.Slot0.kG = ElevatorConstants.MOTORS_KG;
        config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.VELOC;
        config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.ACCEL;
        config.MotionMagic.MotionMagicJerk = ElevatorConstants.JERK;

        config.Feedback.RotorToSensorRatio = ElevatorConstants.ROTOR_TO_SENSOR_RATIO;
        config.Feedback.SensorToMechanismRatio = ElevatorConstants.ENCODER_TO_MECHANISM_RATIO;

        rangeSensor = new CANrange(RobotMap.ELEVATOR_CANRANGE, RobotMap.CANIVORE_CAN_NAME);

        CANrangeConfiguration rangeConfig = new CANrangeConfiguration();
        rangeConfig.ToFParams.UpdateFrequency = 50;
        rangeConfig.ToFParams.UpdateMode = UpdateModeValue.LongRangeUserFreq;
        rangeSensor.getConfigurator().apply(rangeConfig);

        leftMotor.applyConfig(config);
        rightMotor.setControl(new Follower(RobotMap.L_ELEVATOR, true));

        positionPID = new MotionMagicVoltage(ElevatorConstants.MIN_EXTENSION.magnitude()).withSlot(0);

        leftMotor.setPosition(ElevatorConstants.MIN_EXTENSION.magnitude());

        if (Robot.isSimulation()) {
            /*
             * TODO:(for simulation)
             * Determine what Standard Deviations are ideal for noise
             * make the speed more realistic
             */

            gearbox = DCMotor.getKrakenX60(2);
            elevatorSim = new ElevatorSim(gearbox, ElevatorConstants.GEAR_RATIO,
                    ElevatorConstants.CARRIAGE_WEIGHT.in(Kilograms), ElevatorConstants.DRUM_RADIUS.in(Meters),
                    ElevatorConstants.MIN_EXTENSION.in(Meters), ElevatorConstants.MAX_EXTENSION.in(Meters), true, 0, 0d,
                    1d);

            leftSim = new TalonFXSimState(leftMotor);
            rightSim = new TalonFXSimState(rightMotor);
            rangeSensorSim = new CANrangeSimState(rangeSensor);

            // TalonFX sim states do not retain inverts.
            leftSim.Orientation = ElevatorConstants.L_INVERTED ? ChassisReference.Clockwise_Positive
                    : ChassisReference.CounterClockwise_Positive;
            rightSim.Orientation = ElevatorConstants.R_INVERTED ? ChassisReference.Clockwise_Positive
                    : ChassisReference.CounterClockwise_Positive;
        }
    }

    @Override
    public void periodic() {
        currentPosition = getPosition();

        // LightningShuffleboard.setDouble("Diagnostic", "Elevator CANRange Value", rangeSensor.getDistance().getValueAsDouble());

        LightningShuffleboard.setDouble("Elevator", "target pos", targetPosition);
        LightningShuffleboard.setDouble("Elevator", "current pos", currentPosition);
        LightningShuffleboard.setBool("Elevator", "onTarget", isOnTarget());

        LightningShuffleboard.setDouble("Diagnostic", "ELE Left Temperature", leftMotor.getDeviceTemp().getValueAsDouble());
        LightningShuffleboard.setDouble("Diagnostic", "ELE Right Temperature", rightMotor.getDeviceTemp().getValueAsDouble());
        LightningShuffleboard.setBool("Diagnostic", "ELE Overheating", isOverheating());
    }

    /**
     * sets the target position for the elevator
     *
     * @param target height value for the elevator
     */
    public void setPosition(double target) {
        targetPosition = MathUtil.clamp(target, ElevatorConstants.MIN_EXTENSION.magnitude(),
                ElevatorConstants.MAX_EXTENSION.magnitude());

        leftMotor.setControl(positionPID.withPosition(target));
    }

    /**
     * sets the elevator position according to the map
     *
     * @param state State of the rod
     */
    public void setState(RodStates state) {
        setPosition(FishingRodConstants.ELEVATOR_MAP.get(state));
    }

    /**
     * sets basic percentage power to the elevator motors
     *
     * @param power
     */
    public void setRawPower(double power) {
        leftMotor.setControl(new DutyCycleOut(power));
    }

    /**
     * stops the elevator motors
     */
    public void stop() {
        setRawPower(0d);
    }

    /**
     * Checks if either motor is overheating
     *
     * @return true if either motor temperature exceeds the overheat threshold
     */
    @Logged(importance = Importance.DEBUG)
    public boolean isOverheating() {
        return leftMotor.getDeviceTemp().getValueAsDouble() > ElevatorConstants.OVERHEAT_TEMP
                || rightMotor.getDeviceTemp().getValueAsDouble() > ElevatorConstants.OVERHEAT_TEMP
                || (Math.abs(rightMotor.getDeviceTemp().getValueAsDouble() - leftMotor.getDeviceTemp().getValueAsDouble()) > ElevatorConstants.OVERHEAT_TEMP_DIFFERENCE);
    }

    /**
     * checks if the elevator is on target
     *
     * @return true if the elevator is within the tolerance of the target position
     */
    @Logged(importance = Importance.DEBUG)
    public boolean isOnTarget() {
        return Math.abs(targetPosition - currentPosition) <= ElevatorConstants.TOLERANCE;
    }

    /**
     * gets the position of the elevator motors
     *
     * @return left motor position (which the right is synced to)
     */
    @Logged(importance = Importance.DEBUG)
    public double getPosition() {
        return leftMotor.getPosition().getValueAsDouble();
    }

    /**
     * gets the basic percentage power of the elevator motors
     *
     * @return left motor power (which the right is synced to)
     */
    @Logged(importance = Importance.DEBUG)
    public double getCurrentPower() {
        return leftMotor.get();
    }

    /**
     * gets the target position of the elevator motors
     *
     * @return target position of the elevator motors
     */
    @Logged(importance = Importance.DEBUG)
    public double getTargetPosition() {
        return targetPosition;
    }

    @Override
    public void simulationPeriodic() {
        double batteryVoltage = RobotController.getBatteryVoltage();
        leftSim.setSupplyVoltage(batteryVoltage);
        rightSim.setSupplyVoltage(batteryVoltage);
        rangeSensorSim.setSupplyVoltage(batteryVoltage);

        // TODO: I'm unclear if rightsim is necessary, or if this is correct. The WPILib
        // example code only implements one motor, even though it's attached to a
        // 2-motor gearbox
        elevatorSim.setInputVoltage(leftSim.getMotorVoltage() + rightSim.getMotorVoltage());
        elevatorSim.update(RobotMap.UPDATE_FREQ);

        leftSim.setRawRotorPosition(
                Units.metersToInches(elevatorSim.getPositionMeters()) * ElevatorConstants.ENCODER_TO_MECHANISM_RATIO);
        rangeSensorSim.setDistance(elevatorSim.getPositionMeters());

        LightningShuffleboard.setDouble("Elevator", "getPose", getPosition());
        LightningShuffleboard.setDouble("Elevator", "getRawPose",
                Units.metersToInches(elevatorSim.getPositionMeters()));
        LightningShuffleboard.setDouble("Elevator", "amps", leftMotor.getStatorCurrent().getValueAsDouble());
        // setPower(LightningShuffleboard.getDouble("Elevator", "setPower", 0));
        // setPosition(LightningShuffleboard.getDouble("Elevator", "setTarget", 0));

        // TalonFXConfiguration motorConfig = leftMotor.getConfig();

        // motorConfig.Slot0.kP = LightningShuffleboard.getDouble("Elevator", "kP", 0);
        // motorConfig.Slot0.kI = LightningShuffleboard.getDouble("Elevator", "kI", 0);
        // motorConfig.Slot0.kD = LightningShuffleboard.getDouble("Elevator", "kD", 0);
        // motorConfig.Slot0.kS = LightningShuffleboard.getDouble("Elevator", "kF", 0);
        // motorConfig.Slot0.kV = LightningShuffleboard.getDouble("Elevator", "kV", 0);
        // motorConfig.Slot0.kA = LightningShuffleboard.getDouble("Elevator", "kA", 0);
        // motorConfig.Slot0.kG = LightningShuffleboard.getDouble("Elevator", "kG", 0);

        // leftMotor.applyConfig(motorConfig);
    }
}
