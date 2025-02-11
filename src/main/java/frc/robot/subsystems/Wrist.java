// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.FishingRodConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.FishingRodConstants.RodStates;
import frc.robot.Robot;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Wrist extends SubsystemBase {

    private ThunderBird motor;
    private CANcoder encoder;

    private double targetPosition = 0;
    private double currentPosition = 0;

    public final PositionVoltage positionPID = new PositionVoltage(0);

    // Sim stuff
    private DCMotor gearbox;
    private SingleJointedArmSim wristSim;
    private TalonFXSimState motorSim;
    private CANcoderSimState encoderSim;

    public Wrist(ThunderBird motor) {
        this.motor = motor;

        TalonFXConfiguration motorConfig = motor.getConfig();
        CANcoderConfiguration angleConfig = new CANcoderConfiguration();

        encoder = new CANcoder(RobotMap.WRIST_ENCODER, RobotMap.CANIVORE_CAN_NAME);
        angleConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        angleConfig.MagnetSensor.MagnetOffset = Robot.isReal() ? EncoderConstants.tritonWristOffset : 0;
        encoder.getConfigurator().apply(angleConfig);

        motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        motorConfig.Slot0.kP = WristConstants.MOTORS_KP;
        motorConfig.Slot0.kI = WristConstants.MOTORS_KI;
        motorConfig.Slot0.kD = WristConstants.MOTORS_KD;
        motorConfig.Slot0.kS = WristConstants.MOTORS_KF;
        motorConfig.Slot0.kV = WristConstants.MOTORS_KV;
        motorConfig.Slot0.kA = WristConstants.MOTORS_KA;
        motorConfig.Slot0.kG = WristConstants.MOTORS_KG;

        motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        motorConfig.Feedback.SensorToMechanismRatio = WristConstants.ENCODER_TO_MECHANISM_RATIO;
        motorConfig.Feedback.RotorToSensorRatio = WristConstants.ROTOR_TO_ENCODER_RATIO;

        motor.applyConfig(motorConfig);

        if (Robot.isSimulation()) {
            gearbox = DCMotor.getKrakenX60(1);
            wristSim = new SingleJointedArmSim(gearbox, WristConstants.ROTOR_TO_ENCODER_RATIO,
                    WristConstants.MOI.magnitude(), WristConstants.LENGTH.in(Meters),
                    WristConstants.MIN_ANGLE.in(Radians), WristConstants.MAX_ANGLE.in(Radians), true,
                    WristConstants.MIN_ANGLE.in(Radians), 0d, 1d);

            motorSim = new TalonFXSimState(motor);
            encoderSim = new CANcoderSimState(encoder);

            encoderSim.setRawPosition(WristConstants.MIN_ANGLE.in(Rotations));
            motorSim.setRawRotorPosition(WristConstants.MIN_ANGLE.in(Rotations));
        }
    }

    @Override
    public void periodic() {
        currentPosition = getAngle();

        LightningShuffleboard.setDouble("Wrist", "Current Position", currentPosition);
        LightningShuffleboard.setBool("Wrist", "onTarget", isOnTarget());

        LightningShuffleboard.setDouble("Diagnostics", "wrist motor temp", motor.getDeviceTemp().getValueAsDouble());
    }

    /**
     * Set the wrist position in degrees
     *
     * @param position in degrees
     */
    public void setPosition(double position) {
        targetPosition = MathUtil.clamp(position, WristConstants.MIN_ANGLE.in(Degrees),
                WristConstants.MAX_ANGLE.in(Degrees));
        motor.setControl(positionPID.withPosition(Units.degreesToRotations(position)));
    }

    /**
     * Sets the wrist position according to the map
     *
     * @param state State of the rod
     */
    public void setState(RodStates state) {
        setPosition(FishingRodConstants.WRIST_MAP.get(state));
    }

    /**
     * Gets the current angle of the wrist
     *
     * @return Current angle of the wrist
     */
    @Logged(importance = Importance.DEBUG)
    public double getAngle() {
        return encoder.getAbsolutePosition().getValue().in(Degrees);
    }

    /**
     * Gets the target angle of the wrist
     *
     * @return Target angle of the wrist
     */
    public double getTargetAngle() {
        return targetPosition;
    }

    /**
     * Checks if the wrist is on target
     *
     * @return True if the wrist is on target
     */
    public boolean isOnTarget() {
        return Math.abs(targetPosition - currentPosition) < WristConstants.TOLERANCE;
    }

    /**
     * Sets the power to the Wrist motor
     *
     * @param power Wrist motor power
     */
    public void setRawPower(double power) {
        motor.setControl(new DutyCycleOut(power));
    }

    /**
     * Stops the wrist motors
     */
    public void stop() {
        setRawPower(0d);
    }

    @Override
    public void simulationPeriodic() {
        double batteryVoltage = RobotController.getBatteryVoltage();
        motorSim.setSupplyVoltage(batteryVoltage);
        encoderSim.setSupplyVoltage(batteryVoltage);

        double simAngle = Units.radiansToRotations(wristSim.getAngleRads());
        double simVeloc = Units.radiansToRotations(wristSim.getVelocityRadPerSec());
        motorSim.setRawRotorPosition(simAngle);
        motorSim.setRotorVelocity(simVeloc);
        encoderSim.setRawPosition(simAngle);
        encoderSim.setVelocity(simVeloc);

        LightningShuffleboard.setDouble("Wrist", "CANCoder angle",
                encoder.getAbsolutePosition().getValue().in(Degrees));
        LightningShuffleboard.setDouble("Wrist", "sim angle", simAngle);
        LightningShuffleboard.setDouble("Wrist", "getPose", getAngle());
        // setPower(LightningShuffleboard.getDouble("Wrist", "setPower", 0));
        // setPosition(LightningShuffleboard.getDouble("Wrist", "setPosition", 0));

        wristSim.setInputVoltage(motorSim.getMotorVoltage());
        wristSim.update(RobotMap.UPDATE_FREQ);

        // TalonFXConfiguration motorConfig = motor.getConfig();

        // motorConfig.Slot0.kP = LightningShuffleboard.getDouble("Wrist", "kP", 0);
        // motorConfig.Slot0.kI = LightningShuffleboard.getDouble("Wrist", "kI", 0);
        // motorConfig.Slot0.kD = LightningShuffleboard.getDouble("Wrist", "kD", 0);
        // motorConfig.Slot0.kS = LightningShuffleboard.getDouble("Wrist", "kF", 0);
        // motorConfig.Slot0.kV = LightningShuffleboard.getDouble("Wrist", "kV", 0);
        // motorConfig.Slot0.kA = LightningShuffleboard.getDouble("Wrist", "kA", 0);
        // motorConfig.Slot0.kG = LightningShuffleboard.getDouble("Wrist", "kG", 0);

        // motor.applyConfig(motorConfig);

        LightningShuffleboard.setDouble("Wrist", "current angle", getAngle());
        LightningShuffleboard.setDouble("Wrist", "target angle", getTargetAngle());
        LightningShuffleboard.setBool("Wrist", "on target", isOnTarget());
    }
}
