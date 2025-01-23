// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.RobotMap;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;


public class Collector extends SubsystemBase {

    private ThunderBird motor;
    public TalonFXSimState motorSim;
    public SimDevice beamBreakSim;
    public DigitalInput beamBreak;
    public SimBoolean simBoolean;
    public LinearSystemSim collectorSim;


    /** Creates a new Collector.
     * @param motor
     */
    public Collector(ThunderBird motor) {
        this.motor = motor;

        beamBreak = new DigitalInput(RobotMap.COLLECTOR_BEAM_BREAK_PORT);

        if (RobotBase.isSimulation()){
            motorSim = new TalonFXSimState(motor);

            beamBreakSim = new SimDevice(RobotMap.COLLECTOR_BEAM_BREAK_PORT);
            simBoolean = beamBreakSim.createBoolean("BeamBreak", Direction.kBidir, false);   

            beamBreak.setSimDevice(beamBreakSim);

            collectorSim = new LinearSystemSim(LinearSystemId.identifyVelocitySystem(CollectorConstants.COLLECTOR_KV, 
                CollectorConstants.COLLECTOR_KA));
        }
    }

    @Override
    public void periodic() {
        LightningShuffleboard.setBool("Collector", "BeamBreak", getBeamBreakOutput());
        LightningShuffleboard.setDouble("Collector", "Velocity", getVelocity());
    }

    @Override
    public void simulationPeriodic() {
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        collectorSim.setInput(motorSim.getMotorVoltage());
        collectorSim.update(RobotMap.UPDATE_FREQ);

        motorSim.setRotorVelocity(collectorSim.getOutput(0));
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

    public double getVelocity(){
        // return motorSim.getRotorVelocity().getValueAsDouble();

        // return motor.getRotorVelocity().getValueAsDouble();
    }

    public boolean getBeamBreakOutput(){
        return RobotBase.isReal() ? beamBreak.get() : simBoolean.get();
    }

    public void setSimBeamBreak(boolean value){
        simBoolean.set(value);
    }

}
