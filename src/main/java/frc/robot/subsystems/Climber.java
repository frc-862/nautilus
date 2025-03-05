// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
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

    private DigitalInput limitSwitch;

    public Climber(ThunderBird motor) {

        this.motor = motor;

        limitSwitch = new DigitalInput(RobotMap.CLIMBER_LIMIT_SWITCH_DIO);

        if (Robot.isSimulation()) {

            motorSim = new TalonFXSimState(motor);

            gearbox = DCMotor.getFalcon500(1);

            climbSim = new ElevatorSim(gearbox, ClimberConstants.GEAR_RATIO, ClimberConstants.CARRIAGE_MASS,
                    ClimberConstants.DRUM_RADIUS,
                    ClimberConstants.MIN_EXTENSION, ClimberConstants.MAX_EXTENSION, false, 0, 0, 1);
        }
    }

    @Override
    public void periodic() {
        // LightningShuffleboard.setDouble("Climber", "Position", getPostion());
        // LightningShuffleboard.setBool("Climber", "On Target", getOnTarget());
        // LightningShuffleboard.setDouble("Climber", "targetPosition", targetPostion);
        LightningShuffleboard.setBool("Climber", "limit switch", getLimitSwitch());
        // LightningShuffleboard.setDouble("Diagnostic", "climber motor temp", motor.getDeviceTemp().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        // Update the supply voltage of the motorsim
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Update the physics simulation
        climbSim.setInputVoltage(motorSim.getMotorVoltage());
        climbSim.update(RobotMap.UPDATE_FREQ);

        // Update the state of the motor
        motorSim.setRawRotorPosition(
                Units.metersToInches(climbSim.getPositionMeters()) * ClimberConstants.ROTOR_TO_MECHANISM_RATIO);
    }

    /**
     * sets the power to the climber motor
     * @param power
     */
    public void setPower(double power) {
        if (power < 0 && getLimitSwitch()) {
            motor.setControl(new DutyCycleOut(0));
            return;
        } else {
            motor.setControl(new DutyCycleOut(power));
        }
    }

    /**
     * @return position in device rotations
     */
    public double getPostion() {
        return motor.getPosition().getValueAsDouble();
    }

    /**
     * assuming that true means triggered and false means not triggered
     *
     * @return if the limit switch is triggered
     */
    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }
}
