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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.WristConstants;
import frc.robot.Robot;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Wrist extends SubsystemBase {
  
    private ThunderBird motor;
    private CANcoder encoder;

    private double targetPosition = 0;
    private double currentPosition = 0;

    public final PositionVoltage positionPID = new PositionVoltage(0);

    //Sim stuff
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
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.SensorToMechanismRatio = WristConstants.ENCODER_TO_MECHANISM_RATIO;
        motorConfig.Feedback.RotorToSensorRatio = WristConstants.ROTOR_TO_ENCODER_RATIO;
        
        motor.applyConfig(motorConfig);

        if(Robot.isSimulation()) {
            gearbox = DCMotor.getKrakenX60(1);
            wristSim = new SingleJointedArmSim(gearbox, WristConstants.ROTOR_TO_ENCODER_RATIO, WristConstants.MOI.magnitude(), WristConstants.LENGTH.in(Meters), WristConstants.MIN_ANGLE.in(Radians), WristConstants.MAX_ANGLE.in(Radians), true, WristConstants.MIN_ANGLE.in(Radians), 0d, 1d); 

            motorSim = new TalonFXSimState(motor);
            encoderSim = new CANcoderSimState(encoder);

            encoderSim.setRawPosition(WristConstants.MIN_ANGLE.in(Rotations));
            motorSim.setRawRotorPosition(WristConstants.MIN_ANGLE.in(Rotations));
        }
    }

    @Override
    public void periodic() {
        currentPosition = getAngle();
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

        LightningShuffleboard.setDouble("wrist", "CANCoder angle", encoder.getAbsolutePosition().getValue().in(Degrees));
        LightningShuffleboard.setDouble("wrist", "getPose", getAngle());
        LightningShuffleboard.setDouble("wrist", "getTarget", motor.getClosedLoopError().getValueAsDouble());
        // setPower(LightningShuffleboard.getDouble("wrist", "setPower", 0));
        // setPosition(LightningShuffleboard.getDouble("wrist", "setPosition", 0));

        wristSim.setInputVoltage(motorSim.getMotorVoltage()); 
        wristSim.update(RobotMap.UPDATE_FREQ);


        // TalonFXConfiguration motorConfig = motor.getConfig();

        // motorConfig.Slot0.kP = LightningShuffleboard.getDouble("wrist", "kP", 0);
        // motorConfig.Slot0.kI = LightningShuffleboard.getDouble("wrist", "kI", 0);
        // motorConfig.Slot0.kD = LightningShuffleboard.getDouble("wrist", "kD", 0);
        // motorConfig.Slot0.kS = LightningShuffleboard.getDouble("wrist", "kF", 0);
        // motorConfig.Slot0.kV = LightningShuffleboard.getDouble("wrist", "kV", 0);
        // motorConfig.Slot0.kA = LightningShuffleboard.getDouble("wrist", "kA", 0);
        // motorConfig.Slot0.kG = LightningShuffleboard.getDouble("wrist", "kG", 0);
        
        // motor.applyConfig(motorConfig);


        LightningShuffleboard.setDouble("wrist", "current angle", getAngle());
        LightningShuffleboard.setDouble("wrist", "target angle", getTargetAngle());
        LightningShuffleboard.setBool("wrist", "on target", isOnTarget());
    }

    public void setPosition(double position) {
        motor.setControl(positionPID.withPosition(Units.degreesToRotations(position)));
        targetPosition = position;
    }

    @Logged(importance = Importance.DEBUG)
    public double getAngle() {
        return motor.getRotorPosition().getValue().in(Degrees);
    }

    public double getTargetAngle() {
        return targetPosition;
    }
    
    public boolean isOnTarget() {
        return Math.abs(targetPosition - currentPosition) < WristConstants.TOLERANCE;
    }
   
    /**
     * Sets the power to the Wrist motor
     * @param power Wrist motor power
     */
    public void setPower(double power) {
        motor.setControl(new DutyCycleOut(power));

    }
   
    /**
     * Stops the wrist motors
     */
    public void stop() {
        setPower(0);

    }
}
