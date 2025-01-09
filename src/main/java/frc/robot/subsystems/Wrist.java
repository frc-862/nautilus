// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.WristConstants;
import frc.thunder.hardware.ThunderBird;
public class Wrist extends SubsystemBase {
    
    private ThunderBird motor;
    private CANcoder encoder;

    public final PositionVoltage positionPID = new PositionVoltage(0);

    public Wrist() {
        motor = new ThunderBird(RobotMap.WRIST, RobotMap.CANIVORE_CAN_NAME, WristConstants.INVERTED, WristConstants.STATOR_CURRENT_LIMIT, WristConstants.BRAKE_MODE);
        
        TalonFXConfiguration motorConfig = motor.getConfig();
        motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        motorConfig.Slot0.kP = ElevatorConstants.MOTORS_KP;
        motorConfig.Slot0.kI = ElevatorConstants.MOTORS_KI;
        motorConfig.Slot0.kD = ElevatorConstants.MOTORS_KD;
        motorConfig.Slot0.kS = ElevatorConstants.MOTORS_KS;
        motorConfig.Slot0.kV = ElevatorConstants.MOTORS_KV;
        motorConfig.Slot0.kA = ElevatorConstants.MOTORS_KA;
        motorConfig.Slot0.kG = ElevatorConstants.MOTORS_KG;

        motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.SensorToMechanismRatio = WristConstants.ENCODER_TO_MECHANISM_RATIO;
        motorConfig.Feedback.RotorToSensorRatio = WristConstants.ROTOR_TO_ENCODER_RATIO;

        CANcoderConfiguration angleConfig = new CANcoderConfiguration();
        angleConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        encoder = new CANcoder(RobotMap.WRIST_ENCODER, RobotMap.CANIVORE_CAN_NAME);
        encoder.getConfigurator().apply(angleConfig);
    }

    @Override
    public void periodic() {
        
    }
 /**
  * Sets the target position for the wrist
  * @param position is the angle of the motor
  */
    public void setPosition(double position) {
        motor.setPosition(position);
    }
/**
 * Gets the position of the wrist motor
 * @return Wrist motor position
 */
    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }
/**
 * Sets the power to the Wrist motor
 * @param power Wrist motor power
 */
    public void setPower(double power) {
        motor.set(power);
    }
/**
 * Stops the wrist motors
 */
    public void stop() {
        setPower(0);

    }
}
