// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.ctre.phoenix6.sim.CANrangeSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.Robot;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotMap;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;


public class Elevator extends SubsystemBase {

    private ThunderBird leftMotor;
    private ThunderBird rightMotor;
    private CANrange rangeSensor;

    private double targetPosition = 0;
    private double currentPosition = 0;

    private final PositionVoltage elevatorPID = new PositionVoltage(0).withSlot(0);

    //sim stuff
    private DCMotor gearbox;
    private ElevatorSim elevatorSim;
    private TalonFXSimState leftSim;
    private TalonFXSimState rightSim;
    private CANrangeSimState rangeSensorSim;

    public Elevator() {
        leftMotor = new ThunderBird(RobotMap.L_ELEVATOR, RobotMap.CANIVORE_CAN_NAME, ElevatorConstants.L_INVERTED,
            ElevatorConstants.STATOR_CURRENT_LIMIT, ElevatorConstants.BRAKE_MODE);
        rightMotor = new ThunderBird(RobotMap.R_ELEVATOR, RobotMap.CANIVORE_CAN_NAME, ElevatorConstants.R_INVERTED,
            ElevatorConstants.STATOR_CURRENT_LIMIT, ElevatorConstants.BRAKE_MODE);

        TalonFXConfiguration config = leftMotor.getConfig();
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.kP = ElevatorConstants.MOTORS_KP;
        config.Slot0.kI = ElevatorConstants.MOTORS_KI;
        config.Slot0.kD = ElevatorConstants.MOTORS_KD;  
        config.Slot0.kS = ElevatorConstants.MOTORS_KS;
        config.Slot0.kV = ElevatorConstants.MOTORS_KV;
        config.Slot0.kA = ElevatorConstants.MOTORS_KA;
        config.Slot0.kG = ElevatorConstants.MOTORS_KG;

        config.Feedback.RotorToSensorRatio = ElevatorConstants.ROTOR_TO_SENSOR_RATIO;
        config.Feedback.SensorToMechanismRatio = ElevatorConstants.ENCODER_TO_MECHANISM_RATIO;

        rangeSensor = new CANrange(RobotMap.ELEVATOR_CANRANGE);

        CANrangeConfiguration rangeConfig = new CANrangeConfiguration();
        rangeConfig.ToFParams.UpdateFrequency = 50;
        rangeConfig.ToFParams.UpdateMode = UpdateModeValue.LongRangeUserFreq;
        rangeSensor.getConfigurator().apply(rangeConfig);

        leftMotor.applyConfig(config);
        rightMotor.setControl(new Follower(RobotMap.L_ELEVATOR, true));

        if (Robot.isSimulation()) {
            /* TODO:(for simulation)
             * Determine what Drum Radius Means for our mechanism (Mr. Hurley question)
             * Determine what Standard Deviations are ideal for noise
             * Make Starting Height = HOME position when implemented
             */

            gearbox = DCMotor.getKrakenX60(2);
            elevatorSim = new ElevatorSim(gearbox, ElevatorConstants.GEAR_RATIO, ElevatorConstants.CARRIAGE_WEIGHT.in(Kilograms), ElevatorConstants.DRUM_RADIUS.in(Meters), ElevatorConstants.MIN_EXTENSION.in(Meters), ElevatorConstants.MAX_EXTENSION.in(Meters), true, 0, 0d, 1d); 

            leftSim = new TalonFXSimState(leftMotor);
            rightSim = new TalonFXSimState(rightMotor);
            rangeSensorSim = new CANrangeSimState(rangeSensor);

            // TalonFX sim states do not retain inverts. 
            leftSim.Orientation = ElevatorConstants.L_INVERTED ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
            rightSim.Orientation = ElevatorConstants.R_INVERTED ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
        }
    }    

    @Override

    public void periodic() {
        currentPosition = getPosition();
    }

    @Override
    public void simulationPeriodic() {
        double batteryVoltage = RobotController.getBatteryVoltage();
        leftSim.setSupplyVoltage(batteryVoltage);
        rightSim.setSupplyVoltage(batteryVoltage);
        rangeSensorSim.setSupplyVoltage(batteryVoltage);

        //TODO: I'm unclear if rightsim is necessary, or if this is correct. The WPILib example code only implements one motor, even though it's attached to a 2-motor gearbox
        elevatorSim.setInputVoltage(leftSim.getMotorVoltage()); 
        elevatorSim.update(RobotMap.UPDATE_FREQ);

        leftSim.setRawRotorPosition(Units.metersToInches(elevatorSim.getPositionMeters()) * ElevatorConstants.ENCODER_TO_MECHANISM_RATIO);
        rangeSensorSim.setDistance(elevatorSim.getPositionMeters());

        LightningShuffleboard.setDouble("elevator", "getPose", getPosition());
        LightningShuffleboard.setDouble("elevator", "getRawPose", Units.metersToInches(elevatorSim.getPositionMeters()));
        LightningShuffleboard.setDouble("elevator", "amps", leftMotor.getStatorCurrent().getValueAsDouble());
        // setPower(LightningShuffleboard.getDouble("elevator", "setPower", 0));
        // setPosition(LightningShuffleboard.getDouble("elevator", "setTarget", 0));


        // TalonFXConfiguration motorConfig = leftMotor.getConfig();

        // motorConfig.Slot0.kP = LightningShuffleboard.getDouble("elevator", "kP", 0);
        // motorConfig.Slot0.kI = LightningShuffleboard.getDouble("elevator", "kI", 0);
        // motorConfig.Slot0.kD = LightningShuffleboard.getDouble("elevator", "kD", 0);
        // motorConfig.Slot0.kS = LightningShuffleboard.getDouble("elevator", "kF", 0);
        // motorConfig.Slot0.kV = LightningShuffleboard.getDouble("elevator", "kV", 0);
        // motorConfig.Slot0.kA = LightningShuffleboard.getDouble("elevator", "kA", 0);
        // motorConfig.Slot0.kG = LightningShuffleboard.getDouble("elevator", "kG", 0);
        
        // leftMotor.applyConfig(motorConfig);
    }

    /**
     * sets the target position for the elevator
     * @param target height value for the elevator
     */
    public void setPosition(double target) {
        leftMotor.setControl(elevatorPID.withPosition(target));
        targetPosition = target;
    }

    /**
     * sets basic percentage power to the elevator motors
     * @param power
     */
    public void setPower(double power) {
        leftMotor.setControl(new DutyCycleOut(power));
    }

    /**
     * stops the elevator motors
     */
    public void stop() {
        setPower(0d);
    }

    /**
     * checks if the elevator is on target
     * @return true if the elevator is within the tolerance of the target position
     */
    @Logged(importance = Importance.DEBUG)
    public boolean isOnTarget() {
        return Math.abs(targetPosition - currentPosition) <= ElevatorConstants.TOLERANCE;
    }

    /**
     * gets the position of the elevator motors
     * @return left motor position (which the right is synced to)
     */
    @Logged(importance = Importance.DEBUG)
    public double getPosition() {
        return leftMotor.getPosition().getValueAsDouble();
    }

    /**
     * gets the basic percentage power of the elevator motors
     * @return left motor power (which the right is synced to)
     */

    @Logged(importance = Importance.DEBUG)
    public double getCurrentPower() {
        return leftMotor.get();
    }

    @Logged(importance = Importance.DEBUG)
    public double getTargetPosition(){
        return targetPosition;
    }
}

