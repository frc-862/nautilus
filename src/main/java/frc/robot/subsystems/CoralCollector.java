// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralCollectorConstants;
import frc.robot.Constants.RobotMap;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class CoralCollector extends SubsystemBase {

    private ThunderBird motor;
    private DigitalInput beamBreak;

    private double targetPower = 0; // used to track the last power set to the motor

    // sim stuff
    private TalonFXSimState motorSim;
    private SimDevice beamBreakSim;
    private SimBoolean simBoolean;
    private LinearSystemSim<N1, N1, N1> coralCollectorSim;

    public CoralCollector(ThunderBird motor) {
        this.motor = motor;

        beamBreak = new DigitalInput(RobotMap.CORAL_COLLECTOR_BEAM_BREAK_DIO);

        if (RobotBase.isSimulation()) {
            // simulate motor
            motorSim = new TalonFXSimState(motor);

            // simulate beam break sensor

            beamBreakSim = SimDevice.create("BeamBreak");
            simBoolean = beamBreakSim.createBoolean("BeamBreak", Direction.kBidir, false);

            beamBreak.setSimDevice(beamBreakSim);

            // simulate collector
            coralCollectorSim = new LinearSystemSim<N1, N1, N1>(
                LinearSystemId.identifyVelocitySystem(CoralCollectorConstants.KV,
                CoralCollectorConstants.KA));
        }
    }

    @Override
    public void periodic() {
        // LightningShuffleboard.setBool("Collector", "BeamBreak", getBeamBreakOutput());
        LightningShuffleboard.setDouble("Collector", "current", motor.getStatorCurrent().getValueAsDouble());
        // LightningShuffleboard.setBool("Diagnostic", "Collector Raw BeamBreak", beamBreak.get());
        LightningShuffleboard.setDouble("Diagnostic", "COLLECTOR Temperature", motor.getDeviceTemp().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        final double batteryVoltage = RobotController.getBatteryVoltage();

        // set supply voltage to battery voltage (12 v)
        motorSim.setSupplyVoltage(batteryVoltage);

        // set motorspeed using collector physics simulation
        motorSim.setRotorVelocity(coralCollectorSim.getOutput(0));

        // update collector physics simulation
        coralCollectorSim.setInput(motorSim.getMotorVoltage());
        coralCollectorSim.update(RobotMap.UPDATE_FREQ);
    }

    /**
     * Set the power of the motor using a duty cycle
     *
     * @param power
     */
    public void setPower(double power) {
        motor.setControl(new DutyCycleOut(power));
        targetPower = power; // store the current power for reference
    }

    /**
     * stops the motor
     */
    public void stop() {
        motor.stopMotor();
        targetPower = 0; // reset the current power since we stopped the motor
    }

    /**
     * @return current velocity of collector rot/sec
     */
    public double getVelocity() {
        return motor.getRotorVelocity().getValueAsDouble();
    }

    public double getPower() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    /**
     * @return the last power set to the motor. This may not be the actual power due to motor control loops
     */
    public double getTargetPower() {
        // this returns the last power set to the motor
        // this may not be the actual power due to motor control loops
        return targetPower;
    }

    /**
     * @return if bream break sensor is triggered
     */
    public boolean getBeamBreakOutput() {
        return RobotBase.isReal() ? beamBreak.get() : simBoolean.get();
    }

    /**
     * @return if we are stalling the motor
     */
    public boolean getCollectCurrentHit() {
        return motor.getStatorCurrent().getValueAsDouble() >= CoralCollectorConstants.CORAL_COLLECTED_CURRENT;
    }

    public boolean getAlgaeCurrentHit() {
        return motor.getStatorCurrent().getValueAsDouble() >= CoralCollectorConstants.ALGAE_COLLECTED_CURRENT;
    }

    /**
     * @return if we are stalling the motor
     * @param current to check if over
     */
    public boolean getCollectCurrentHit(double current) {
        return motor.getStatorCurrent().getValueAsDouble() >= current;
    }

    public boolean isOverheating() {
        return motor.getDeviceTemp().getValueAsDouble() >= CoralCollectorConstants.OVERHEAT_TEMP;
    }
    
    /**
     * can be used to set beambreak value to simulate beambreak
     *
     * @param value
     */
    public void setSimBeamBreak(boolean value) {
        simBoolean.set(value);
    }
}
