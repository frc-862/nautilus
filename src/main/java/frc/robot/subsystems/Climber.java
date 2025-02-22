// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Robot;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Climber extends SubsystemBase {

    private ThunderBird motor;
    private TalonFXSimState motorSim;

    private ElevatorSim climbSim;
    private DCMotor gearbox;

    private final PositionVoltage positionPID = new PositionVoltage(0);
    private double targetPostion = 0;
    
    public Climber(ThunderBird motor) {

        TalonFXConfiguration config = motor.getConfig();

        this.motor = motor;

        if(Robot.isSimulation()){

            motorSim = new TalonFXSimState(motor);

            gearbox = DCMotor.getFalcon500(1);

            climbSim = new ElevatorSim(gearbox, ClimberConstants.GEAR_RATIO, ClimberConstants.CARRIAGE_MASS, ClimberConstants.DRUM_RADIUS, 
                ClimberConstants.MIN_EXTENSION, ClimberConstants.MAX_EXTENSION, false, 0, 0, 1);
        }

        config.Slot0.kP = ClimberConstants.KP;
        config.Slot0.kI = ClimberConstants.KI;
        config.Slot0.kD = ClimberConstants.KD;  

        motor.applyConfig(config);
    }

    @Override
    public void periodic() {
        // LightningShuffleboard.setDouble("Climber", "Position", getPostion());
        // LightningShuffleboard.setBool("Climber", "On Target", getOnTarget());
        // LightningShuffleboard.setDouble("Climber", "targetPosition", targetPostion);
        // LightningShuffleboard.setDouble("Diagnostics", "climber motor temp", motor.getDeviceTemp().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        // Update the supply voltage of the motorsim
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Update the physics simulation
        climbSim.setInputVoltage(motorSim.getMotorVoltage());
        climbSim.update(RobotMap.UPDATE_FREQ);

        // Update the state of the motor
        motorSim.setRawRotorPosition(Units.metersToInches(climbSim.getPositionMeters()) * ClimberConstants.ROTOR_TO_MECHANISM_RATIO);
    }

    public void setPower(double speed) {
        motor.setControl(new DutyCycleOut(speed));
    }

    /**
     * @param position set target position in device rotations
     */
    public void setTargetPosition(double position) {
        motor.setControl(positionPID.withPosition(position));
        targetPostion = position;
    }

    /**
     * @return position in device rotations
     */
    public double getPostion() {
        return motor.getPosition().getValueAsDouble();
    }

    /**
     * @return if the climber is on target
     */
    public boolean getOnTarget() {
        return Math.abs(getPostion() - targetPostion) < ClimberConstants.TOLERANCE;
    }


}
