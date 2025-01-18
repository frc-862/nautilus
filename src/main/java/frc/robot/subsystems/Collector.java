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
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.CollectorConstants;
import frc.thunder.hardware.ThunderBird;


public class Collector extends SubsystemBase {
    private ThunderBird motor;
    private CANcoder encoder;

    private double targetPosition = 0;
    private double currentPosition = 0;

    public final PositionVoltage positionPID = new PositionVoltage(0);

  /** Creates a new Collector. */
  public Collector(ThunderBird motor) {
    this.motor = motor;

      TalonFXConfiguration motorConfig = motor.getConfig();
        CANcoderConfiguration angleConfig = new CANcoderConfiguration();

        angleConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        encoder = new CANcoder(RobotMap.COLLECTOR_ENCODER, RobotMap.CANIVORE_CAN_NAME);
        encoder.getConfigurator().apply(angleConfig);


        motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;  //make all of this Collector Constants
        motorConfig.Slot0.kP = CollectorConstants.MOTORS_KP; 
        motorConfig.Slot0.kI = CollectorConstants.MOTORS_KI;
        motorConfig.Slot0.kD = CollectorConstants.MOTORS_KD;
        motorConfig.Slot0.kS = CollectorConstants.MOTORS_KS;
        motorConfig.Slot0.kV = CollectorConstants.MOTORS_KV;
        motorConfig.Slot0.kA = CollectorConstants.MOTORS_KA;
        motorConfig.Slot0.kG = CollectorConstants.MOTORS_KG;

        motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
         motorConfig.Feedback.SensorToMechanismRatio = CollectorConstants.ENCODER_TO_MECHANISM_RATIO;
        motorConfig.Feedback.RotorToSensorRatio = CollectorConstants.ROTOR_TO_ENCODER_RATIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double power){
      motor.set(power);
  }
  public void stop(){
    motor.stopMotor();
  }
}
