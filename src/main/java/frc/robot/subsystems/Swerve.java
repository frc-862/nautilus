package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Map.Entry;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.VisionConstants;
import frc.thunder.shuffleboard.LightningShuffleboard;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private boolean[] reef1Status = { false, false, false, false, false, false, false, false, false, false, false,
            false };
    private boolean[] reef2Status = { false, false, false, false, false, false, false, false, false, false, false,
            false };
    private boolean[] reef3Status = { false, false, false, false, false, false, false, false, false, false, false,
            false };

    private double[] currentCANCoderValues = { 0d, 0d, 0d, 0d };
    private static final double[] CANCoderOffsetConstants = { EncoderConstants.frontLeftOffset,
            EncoderConstants.frontRightOffset,
            EncoderConstants.backLeftOffset, EncoderConstants.backRightOffset };

    // other swerve requests are in Constants.java/DrivetrainConstants/DriveRequests
    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

    private final LinearFilter xFilter = LinearFilter.movingAverage(5);
    private final LinearFilter yFilter = LinearFilter.movingAverage(5);
    private final LinearFilter rotFilter = LinearFilter.movingAverage(5);

    private boolean slowMode = false;

    private double speedMult = 1d;
    private double turnMult = 1d;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public Swerve(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configurePathPlanner();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public Swerve(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, odometryUpdateFrequency,
                odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configurePathPlanner();
    }

    @Override
    public void periodic() {
        updateCANcoderOffsets();

        Pose2d pose = getPose();
        xFilter.calculate(pose.getX());
        yFilter.calculate(pose.getY());
        rotFilter.calculate(pose.getRotation().getRadians());

        // LightningShuffleboard.setDoubleArray("Diagnostic", "Swerve CANCoder Offsets", currentCANCoderValues);

        // update reef status booleans in the future
        SmartDashboard.putBooleanArray("Reef Level One", reef1Status);
        SmartDashboard.putBooleanArray("Reef Level Two", reef2Status);
        SmartDashboard.putBooleanArray("Reef Level Three", reef3Status);

        // LightningShuffleboard.setPose2d("Drivetrain", "pose", getState().Pose);

        LightningShuffleboard.setDouble("Drivetrain", "tag id", PoseConstants.getScorePose(pose));
    }

    private void configurePathPlanner() {
        AutoBuilder.configure(
                () -> getPose(), // Supplier of current robot pose
                this::resetPose, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds, feedforwards) -> this
                        .setControl(autoRequest.withSpeeds(speeds).withDriveRequestType(DriveRequestType.Velocity)
                                .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX())
                                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY())), // Consumer of
                                                                                                    // ChassisSpeeds to
                                                                                                    // drive the robot
                new PPHolonomicDriveController(AutonomousConstants.TRANSLATION_PID, AutonomousConstants.ROTATION_PID),
                AutonomousConstants.CONFIG,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this); // Subsystem for requirements
    }

    public boolean poseZero() {
        return Math.abs(getPose().getTranslation().getNorm()) <= 0.1;
    }

    public Pose2d getPose() {
        SwerveDriveState state = getState();
        if (state == null || getState().Pose == null) {
            return new Pose2d();
        }
        return state.Pose;
    }

    public boolean poseStable() {
        Pose2d pose = getPose();
        return (
            Math.abs(xFilter.calculate(pose.getX()) - pose.getX()) < 0.1 &&
            Math.abs(yFilter.calculate(pose.getY()) - pose.getY()) < 0.1 &&
            Math.abs(rotFilter.calculate(pose.getRotation().getRadians()) - pose.getRotation().getRadians()) < 1);
    }

    public void resetForwardWithOpPerspective(Rotation2d rotation) {
        seedFieldCentric();
        setOperatorPerspectiveForward(rotation);
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param requestSupplier Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return getState().Speeds;
    }

    /**
     * gets slow mode true/false
     *
     * @return slow mode true/false
     */
    public boolean inSlowMode() {
        return slowMode;
    }

    /**
     * gets the speed multiplier
     *
     * @return speed multiplier (1.0 for normal, 0.4 for slow mode)
     */
    public double getSpeedMult() {
        return speedMult;
    }

    /**
     * gets the turn multiplier
     *
     * @return turn multiplier (1.0 for normal, 0.7 for slow mode)
     */
    public double getTurnMult() {
        return turnMult;
    }

    /**
     * sets slow mode true/false
     * speedMult and turnMult are set to the appropriate values
     *
     * @param slowMode
     */
    public void setSlowMode(boolean slowMode) {
        this.slowMode = slowMode;

        if (slowMode) {
            speedMult = DrivetrainConstants.SLOW_SPEED_MULT;
            turnMult = DrivetrainConstants.SLOW_TURN_MULT;
        } else {
            speedMult = 1d;
            turnMult = 1d;
        }
    }

    private void updateCANcoderOffsets() {
        int i = 0;
        for (SwerveModule<TalonFX, TalonFX, CANcoder> swerveModule : getModules()) {
            currentCANCoderValues[i] = swerveModule.getEncoder().getAbsolutePosition().getValueAsDouble()
                    - CANCoderOffsetConstants[i];
            i++;
        }
    }

    public void addVisionMeasurement(EstimatedRobotPose pose, double distance) {
        if (DriverStation.isDisabled()) {
            addVisionMeasurement(pose.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(pose.timestampSeconds),
                    VecBuilder.fill(0.1, 0.1, 0.1));
        } else {
            // addVisionMeasurement(pose.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(pose.timestampSeconds),
            //         VecBuilder.fill(VisionConstants.VISION_X_STDEV, VisionConstants.VISION_Y_STDEV, VisionConstants.VISION_THETA_STDEV));
        
            // if(distance < 0.25) {
            //     addVisionMeasurement(pose.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(pose.timestampSeconds),
            //         VecBuilder.fill(0.01, 0.01, 0.01));
            // } else {

                // for ambiguity-based (or distance-based) std deviations
                addVisionMeasurement(pose.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(pose.timestampSeconds),
                        VecBuilder.fill(distance / 2, distance / 2, distance / 2));
            // }

            }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void simulationPeriodic() {
        /* Assume 20ms update rate, get battery voltage from WPILib */
        // updateSimState(0.020, RobotController.getBatteryVoltage());
    }

    public int reefTagToRobot(Pose2d pos){
        for (Entry<Rectangle2d, Integer> entry : PoseConstants.aprilTagRegions.entrySet()){
            if(entry.getKey().contains(pos.getTranslation())){
                return entry.getValue().intValue();
            }
        }
        return 0;
    }

    public int reefTagToRobot(){
        for (Entry<Rectangle2d, Integer> entry : PoseConstants.aprilTagRegions.entrySet()) {
            if(entry.getKey().contains(getPose().getTranslation())) {
                return entry.getValue().intValue();
            }
        }
        return 0;
    }

    public Command leaveAuto() {
        return (applyRequest((DriveRequests.getRobotCentric(() -> 0d, () -> -0.5, () -> 0d))).withDeadline(new WaitCommand(3))).andThen(applyRequest(DriveRequests.getBrake()));
    }

    // SYSID
    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /**
     * Runs the SysIdtest in the given direction for the routine
     * specified
     *
     * @param testType Type of test to run (ex. DRIVE)
     * @param direction Direction of the SysId Quasistatic test
     * @param isDynamic Whether to run a dynamic/quasistatic test (both need to run for full sysid)
     * @return Command to run
     */
    public Command sysId(DrivetrainConstants.SysIdTestType testType, SysIdRoutine.Direction direction, boolean isDynamic) {
        switch (testType) {
            case DRIVE:
                return isDynamic ? m_sysIdRoutineTranslation.dynamic(direction) : m_sysIdRoutineTranslation.quasistatic(direction);
            case STEER:
                return isDynamic ? m_sysIdRoutineSteer.dynamic(direction) : m_sysIdRoutineSteer.quasistatic(direction);
            case ROTATE:
                return isDynamic ? m_sysIdRoutineRotation.dynamic(direction) : m_sysIdRoutineRotation.quasistatic(direction);
            default:
                return new InstantCommand();
        }
    }
}
