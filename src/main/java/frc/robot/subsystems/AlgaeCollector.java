// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.robot.Robot;
import frc.robot.Constants.AlgaeCollectorConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.AlgaeCollectorConstants.PIVOT_STATES;

public class AlgaeCollector extends SubsystemBase {

    private ThunderBird rollerMotor;
    private ThunderBird pivotMotor;

    private PositionVoltage pivotPID = new PositionVoltage(0);
    private double targetAngle = 0;

    private TalonFXSimState rollerMotorSim;
    private TalonFXSimState pivotMotorSim;

    private SingleJointedArmSim pivotSim;
    @SuppressWarnings("rawtypes")
    private LinearSystemSim rollerSim;

    private DCMotor pivotGearbox;

    @SuppressWarnings({ "rawtypes", "unchecked" })
    public AlgaeCollector(ThunderBird rollerMotor, ThunderBird pivotMotor) {
        this.rollerMotor = rollerMotor;
        this.pivotMotor = pivotMotor;

        TalonFXConfiguration pivotMotorConfig = pivotMotor.getConfig();

        if(Robot.isSimulation()){
            // simulate motors
            rollerMotorSim = new TalonFXSimState(rollerMotor);
            pivotMotorSim = new TalonFXSimState(pivotMotor);

            pivotGearbox = DCMotor.getKrakenX60(1);

            // create physics sims
            pivotSim = new SingleJointedArmSim(pivotGearbox, AlgaeCollectorConstants.PIVOT_GEAR_RATIO, 
                AlgaeCollectorConstants.PIVOT_MOI, AlgaeCollectorConstants.PIVOT_LENGTH, Units.degreesToRadians(AlgaeCollectorConstants.PIVOT_MIN_ANGLE), 
                Units.degreesToRadians(AlgaeCollectorConstants.PIVOT_MAX_ANGLE), true, AlgaeCollectorConstants.PIVOT_START_ANGLE, 0, 1);

            rollerSim = new LinearSystemSim(LinearSystemId.identifyVelocitySystem(AlgaeCollectorConstants.ROLLER_KV, 
                AlgaeCollectorConstants.ROLLER_KA));
        }

        pivotMotorConfig.Slot0.kP = AlgaeCollectorConstants.PIVOT_KP;
        pivotMotorConfig.Slot0.kI = AlgaeCollectorConstants.PIVOT_KI;
        pivotMotorConfig.Slot0.kD = AlgaeCollectorConstants.PIVOT_KD;

        pivotMotor.applyConfig(pivotMotorConfig);
    }

    @Override
    public void periodic() {
        LightningShuffleboard.setDouble("Algae Collector", "Pivot Angle", getPivotAngle());
        LightningShuffleboard.setDouble("Algae Collector", "Roller Velocity", getRollerVelocity());
        LightningShuffleboard.setDouble("Algae Collector", "Pivot Target", getTargetAngle());
        LightningShuffleboard.setDouble("Algae Collector", "Pivot Motor current", pivotMotorSim.getMotorVoltage());
        LightningShuffleboard.setDouble("Algae Collector", "Pivot raw roter pos0ition", pivotSim.getAngleRads());
        
    }

    @Override
    public void simulationPeriodic() {
        final double batteryVoltage = RobotController.getBatteryVoltage();

        // set supply voltages to motors
        rollerMotorSim.setSupplyVoltage(batteryVoltage);
        pivotMotorSim.setSupplyVoltage(batteryVoltage);

        double pivotAngle = Units.radiansToRotations(pivotSim.getAngleRads());
        double pivotVelocity = Units.radiansToRotations(pivotSim.getVelocityRadPerSec());

        double rollerVelocity = rollerSim.getOutput(0);

        // set simulated motor states
        pivotMotorSim.setRawRotorPosition(pivotAngle);
        pivotMotorSim.setRotorVelocity(pivotVelocity);

        rollerMotorSim.setRotorVelocity(rollerVelocity);

        // use motor voltages to set input voltages for physics simulations
        pivotSim.setInputVoltage(pivotMotorSim.getMotorVoltage());
        rollerSim.setInput(rollerMotorSim.getMotorVoltage());
        
        // update physics simulations
        pivotSim.update(RobotMap.UPDATE_FREQ);
        rollerSim.update(RobotMap.UPDATE_FREQ);
    }

    /**
     * Set the power of the roller motor
     * @param power
     */
    public void setRollerPower(double power) {
        rollerMotor.setControl(new DutyCycleOut(power));
    }

    /**
     * Set the power of the pivot motor
     * @param speed
     */
    public void setPivotPower(double speed) {
        pivotMotor.setControl(new DutyCycleOut(speed));
    }

    /**
     * set target angle for pivot in degrees
     * @param angle
     */
    public void setPivotState(PIVOT_STATES state) {
        double angle = (state == PIVOT_STATES.DEPLOYED) ? AlgaeCollectorConstants.DEPLOY_ANGLE : AlgaeCollectorConstants.STOW_ANGLE;
        pivotMotor.setControl(pivotPID.withPosition(Units.degreesToRotations(angle)));
        targetAngle = angle;
    }

    /**
     * @return current target angle for pivot in degrees
     */
    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * @return current angle of pivot in degrees
     */
    public double getPivotAngle() {
        return Units.rotationsToDegrees(pivotMotor.getPosition().getValueAsDouble());
    }

    /**
     * @return if pivot is on target
     */
    public boolean pivotOnTarget(){
        return Math.abs(getPivotAngle() - targetAngle) < AlgaeCollectorConstants.PIVOT_TOLERANCE;
    }

    /**
     * @return current velocity of roller in rot/sec
     */
    public double getRollerVelocity() {
        return rollerMotor.getVelocity().getValueAsDouble();
    }

    /**
     * @return current power of roller
     */
    public double getRollerPower() {
        return rollerMotor.get();
    }

    public void stop(){
        this.setRollerPower(0);
    }
}
