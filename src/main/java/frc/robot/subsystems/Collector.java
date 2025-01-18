// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.thunder.hardware.ThunderBird;


public class Collector extends SubsystemBase {

    private ThunderBird motor;
    public double power = 0;

    /** Creates a new Collector.
     * @param motor
     */
    public Collector(ThunderBird motor) {
        this.motor = motor;
    }

    @Override
    public void periodic() {
        setPower(power);
    }

    /**
     * Set the power of the motor using a duty cycle
     * @param power
     */
    public void setPower(double power){
        motor.setControl(new DutyCycleOut(power));
    }

    /**
     * stops the motor
     */
    public void stop(){
        motor.stopMotor();
    }
}
