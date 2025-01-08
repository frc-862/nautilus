// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import frc.thunder.hardware.ThunderBird;
public class Wrist extends SubsystemBase {
    
    public ThunderBird wristMotor;
    private CANcoder angleEncoder;

    public final PositionVoltage positionPID = new PositionVoltage(0);

    public Wrist() {
        wristMotor = new ThunderBird(44, RobotMap.CANIVORE_CAN_NAME, false, 60, true);

        CANcoderConfiguration angleConfig = new CANcoderConfiguration();
        angleConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        angleEncoder = new CANcoder(45, RobotMap.CANIVORE_CAN_NAME);
        angleEncoder.getConfigurator().apply(angleConfig);
    }

    @Override
    public void periodic() {
        

    }
    public void setPosition(double position) {
        position *= 36;
        wristMotor.setPosition(position);
    }

    public double getPosition() {
        return wristMotor.getPosition().getValueAsDouble();
    }

    public void setPower(double power) {
        wristMotor.set(power);
    }

    public void stop() {
        setPower(0);

    }
}
