// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.RobotMap;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;


public class Collector extends SubsystemBase {

    private ThunderBird motor;
    private TalonFXSimState motorSim;

    private DigitalInput beamBreak;
    private SimDevice beamBreakSim;
    private SimBoolean simBoolean;

    @SuppressWarnings("rawtypes")
    private LinearSystemSim collectorSim;

    /** Creates a new Collector.
     * @param motor
     */
    @SuppressWarnings({ "rawtypes", "unchecked" })
    public Collector(ThunderBird motor) {
        this.motor = motor;

        // Instantiate Beam Break Sensor
        beamBreak = new DigitalInput(RobotMap.COLLECTOR_BEAM_BREAK_DIO);

        if (RobotBase.isSimulation()){

            // simulate motor
            motorSim = new TalonFXSimState(motor);

            // simulate beam break sensor

            beamBreakSim = SimDevice.create("BeamBreak");
            simBoolean = beamBreakSim.createBoolean("BeamBreak", Direction.kBidir, false);   

            beamBreak.setSimDevice(beamBreakSim);

            // simulate collector

            collectorSim = new LinearSystemSim(LinearSystemId.identifyVelocitySystem(CollectorConstants.COLLECTOR_KV, 
                CollectorConstants.COLLECTOR_KA));
        }
    }

    @Override
    public void periodic() {
        LightningShuffleboard.setBool("Collector", "BeamBreak", getBeamBreakOutput());
        LightningShuffleboard.setDouble("Collector", "Velocity", getVelocity());
        LightningShuffleboard.setBool("Diagnostic", "Collector Raw BeamBreak", beamBreak.get());
    }

    @Override
    public void simulationPeriodic() {

        // set supply voltage to battery voltage (12 v)
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // set motorspeed using collector physics simulation
        motorSim.setRotorVelocity(collectorSim.getOutput(0));

        // update collector physics simulation
        collectorSim.setInput(motorSim.getMotorVoltage());
        collectorSim.update(RobotMap.UPDATE_FREQ);
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

    /**
     * @return current velocity of collector rot/sec
     */
    public double getVelocity(){
        return motor.getRotorVelocity().getValueAsDouble();
    }



    /**
     * @return if bream break sensor is triggered
     */
    public boolean getBeamBreakOutput(){
        return RobotBase.isReal() ? beamBreak.get() : simBoolean.get();
    }

    /**
     * can be used to set beambreak value to simulate beambreak
     * @param value
     */
    public void setSimBeamBreak(boolean value){
        simBoolean.set(value);
    }

}
