// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotMap;
import frc.thunder.hardware.ThunderBird;

public class Elevator extends SubsystemBase {

    private ThunderBird leftMotor;
    private ThunderBird rightMotor;

    private double targetPosition = 0;
    private double currentPosition = 0;

    private final PositionVoltage elevatorPID = new PositionVoltage(0).withSlot(0);

    /** Creates a new Lift. */
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

        leftMotor.applyConfig(config);
        rightMotor.setControl(new Follower(RobotMap.L_ELEVATOR, true));
    }    

    @Override
    public void periodic() {
        currentPosition = getPosition();
    }

    /**
     * sets the target position for the elevator
     * @param target height value for the elevator
     */
    public void setPosition(double target) {
        // we dont know exactly how far the elevator will move in one rotation, so no math is done here
        leftMotor.setControl(elevatorPID.withPosition(target));
        targetPosition = target;
    }

    /**
     * sets basic percentage power to the elevator motors
     * @param power
     */
    public void setPower(double power) {
        leftMotor.set(power);
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
    public boolean isOnTarget() {
        return Math.abs(targetPosition - currentPosition) <= ElevatorConstants.TOLERANCE;
    }

    /**
     * gets the position of the elevator motors
     * @return left motor position (which the right is synced to)
     */
    public double getPosition() {
        return leftMotor.getPosition().getValueAsDouble();
    }

    /**
     * gets the basic percentage power of the elevator motors
     * @return left motor power (which the right is synced to)
     */
    public double getCurrentPower() {
        return leftMotor.get();
    }
}

