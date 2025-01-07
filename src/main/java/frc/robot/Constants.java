package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionTargetSim;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;



public class Constants {

    public static class RobotMap {


    }

    public static class ControllerConstants {
        public static final int DRIVER_CONTROLLER = 0;
        public static final int COPILOT_CONTROLLER = 1;

        public static final double JOYSTICK_DEADBAND = 0.1; 
        public static final double TRIGGER_DEADBAND = 0.05; 


    }

    public class DrivetrainConstants {
        public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        public static final double SLOW_MODE_MULT = 0.3; // 3/4 of a rotation per second max angular velocity

        public class DriveRequests {
            private static final SwerveRequest.FieldCentric DRIVE = new SwerveRequest.FieldCentric();
            private static final SwerveRequest.FieldCentric SLOW = new SwerveRequest.FieldCentric();
            private static final SwerveRequest.RobotCentric ROBO_CENTRIC = new SwerveRequest.RobotCentric();
            private static final SwerveRequest.SwerveDriveBrake BRAKE = new SwerveRequest.SwerveDriveBrake();

            public static Supplier<SwerveRequest> getDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
                return () -> DRIVE
                                .withVelocityX(y.getAsDouble() * DrivetrainConstants.MAX_SPEED) // Drive forward with negative Y (forward)
                                .withVelocityY(x.getAsDouble() * DrivetrainConstants.MAX_SPEED) // Drive left with negative X (left)
                                .withRotationalRate(rot.getAsDouble() * DrivetrainConstants.MAX_ANGULAR_RATE)
                                .withDeadband(DrivetrainConstants.MAX_SPEED * 0.1).withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE * 0.1) // Add a 10% deadband
                                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Drive counterclockwise with negative X (left)
                            
            }

            public static Supplier<SwerveRequest> getSlow(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
                return () -> SLOW
                                .withVelocityX(y.getAsDouble() * DrivetrainConstants.MAX_SPEED * SLOW_MODE_MULT) // Drive forward with negative Y (forward)
                                .withVelocityY(x.getAsDouble() * DrivetrainConstants.MAX_SPEED * SLOW_MODE_MULT) // Drive left with negative X (left)
                                .withRotationalRate(rot.getAsDouble() * DrivetrainConstants.MAX_ANGULAR_RATE * SLOW_MODE_MULT)
                                .withDeadband(DrivetrainConstants.MAX_SPEED * 0.1).withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE * 0.1) // Add a 10% deadband
                                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Drive counterclockwise with negative X (left)
                            
            }

            public static Supplier<SwerveRequest> getRobotCentric(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
                return () -> ROBO_CENTRIC
                                .withVelocityX(y.getAsDouble() * DrivetrainConstants.MAX_SPEED) // Drive forward with negative Y (forward)
                                .withVelocityY(x.getAsDouble() * DrivetrainConstants.MAX_SPEED) // Drive left with negative X (left)
                                .withRotationalRate(rot.getAsDouble() * DrivetrainConstants.MAX_ANGULAR_RATE)
                                .withDeadband(DrivetrainConstants.MAX_SPEED * 0.1).withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE * 0.1) // Add a 10% deadband
                                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Drive counterclockwise with negative X (left)
            }


            public static Supplier<SwerveRequest> getBrake() {
                return () -> BRAKE;
            }


            //auto request exists in Swerve.java
        }

    }

    public static class AutonomousConstants {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(10, 0, 0);
        public static final PIDConstants ROTATION_PID = new PIDConstants(5, 0, 0);

        private static final double TRACK_WIDTH = Units.inchesToMeters(27); //TODO: make more accurate
        private static final Mass ROBOT_MASS = Pounds.of(147);
        private static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(5.2268411); //TODO: this assumes even weight distribution; should be calculated w/ SYSID or CAD
        private static final ModuleConfig MODULE_CONFIG = new ModuleConfig(TunerConstants.kWheelRadius, TunerConstants.kSpeedAt12Volts, 1.916, DCMotor.getKrakenX60Foc(1).withReduction(TunerConstants.kDriveGearRatio), Amps.of(120), 1);

        public static final RobotConfig CONFIG = new RobotConfig(ROBOT_MASS, ROBOT_MOI, MODULE_CONFIG, new Translation2d[]{new Translation2d(TRACK_WIDTH/2, TRACK_WIDTH/2), new Translation2d(TRACK_WIDTH/2, -TRACK_WIDTH/2), new Translation2d(-TRACK_WIDTH/2, TRACK_WIDTH/2), new Translation2d(-TRACK_WIDTH/2, -TRACK_WIDTH/2)});
    }

    public static class VisionConstants {
        public static final String camera1Name = "cam1";
        public static final TargetModel targetModel = TargetModel.kAprilTag36h11;
        public static final Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
        public static final VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);
        public static final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
        public static final SimCameraProperties cameraProp = new SimCameraProperties();
        public static final Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
        public static final Rotation3d robotToCameraRot = new Rotation3d(0, 0, 0);
        public static final Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);


    }

    public static class PoseConstants {
        public static final Translation2d FIELD_LIMIT = new Translation2d(Units.feetToMeters(54.0),
                Units.feetToMeters(26.0));
    }


    public class TunerConstants {
        // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100).withKI(0).withKD(0.5)
            .withKS(0.1).withKV(2.66).withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(0.1).withKI(0).withKD(0)
            .withKS(0).withKV(0.124);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The type of motor used for the drive motor
        private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        // The type of motor used for the drive motor
        private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

        // The remote sensor feedback type to use for the steer motors;
        // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
        private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final Current kSlipCurrent = Amps.of(120.0);

        // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
        // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
        private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a relatively low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(60))
                    .withStatorCurrentLimitEnable(true)
            );
        private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
        // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
        private static final Pigeon2Configuration pigeonConfigs = null;

        // CAN bus that the devices are located on;
        // All swerve devices must share the same CAN bus
        public static final CANBus kCANBus = new CANBus("Canivore", "./logs/example.hoot");

        // Theoretical free speed (m/s) at 12 V applied output;
        // This needs to be tuned to your individual robot
        public static final LinearVelocity kSpeedAt12Volts = FeetPerSecond.of(15.01);

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 3.5714285714285716;

        private static final double kDriveGearRatio = 6.746031746031747;
        private static final double kSteerGearRatio = 21.428571428571427;
        private static final Distance kWheelRadius = Inches.of(2);

        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;

        private static final int kPigeonId = 23;

        // These are only used for simulation
        private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
        private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
        // Simulated voltage necessary to overcome friction
        private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
        private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

        public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName(kCANBus.getName())
                .withPigeon2Id(kPigeonId)
                .withPigeon2Configs(pigeonConfigs);

        private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withCouplingGearRatio(kCoupleRatio)
                .withWheelRadius(kWheelRadius)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                .withSlipCurrent(kSlipCurrent)
                .withSpeedAt12Volts(kSpeedAt12Volts)
                .withDriveMotorType(kDriveMotorType)
                .withSteerMotorType(kSteerMotorType)
                .withFeedbackSource(kSteerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(encoderInitialConfigs)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage);


        // Front Left
        private static final int kFrontLeftDriveMotorId = 1;
        private static final int kFrontLeftSteerMotorId = 2;
        private static final int kFrontLeftEncoderId = 31;
        private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.2353515625);
        private static final boolean kFrontLeftSteerMotorInverted = true;
        private static final boolean kFrontLeftEncoderInverted = false;

        private static final Distance kFrontLeftXPos = Inches.of(13.5);
        private static final Distance kFrontLeftYPos = Inches.of(13.5);

        // Front Right
        private static final int kFrontRightDriveMotorId = 3;
        private static final int kFrontRightSteerMotorId = 4;
        private static final int kFrontRightEncoderId = 32;
        private static final Angle kFrontRightEncoderOffset = Rotations.of(0.391357421875);
        private static final boolean kFrontRightSteerMotorInverted = true;
        private static final boolean kFrontRightEncoderInverted = false;

        private static final Distance kFrontRightXPos = Inches.of(13.5);
        private static final Distance kFrontRightYPos = Inches.of(-13.5);

        // Back Left
        private static final int kBackLeftDriveMotorId = 5;
        private static final int kBackLeftSteerMotorId = 6;
        private static final int kBackLeftEncoderId = 33;
        private static final Angle kBackLeftEncoderOffset = Rotations.of(0.14404296875);
        private static final boolean kBackLeftSteerMotorInverted = true;
        private static final boolean kBackLeftEncoderInverted = false;

        private static final Distance kBackLeftXPos = Inches.of(-13.5);
        private static final Distance kBackLeftYPos = Inches.of(13.5);

        // Back Right
        private static final int kBackRightDriveMotorId = 7;
        private static final int kBackRightSteerMotorId = 8;
        private static final int kBackRightEncoderId = 34;
        private static final Angle kBackRightEncoderOffset = Rotations.of(0.0126953125);
        private static final boolean kBackRightSteerMotorInverted = true;
        private static final boolean kBackRightEncoderInverted = false;

        private static final Distance kBackRightXPos = Inches.of(-13.5);
        private static final Distance kBackRightYPos = Inches.of(-13.5);


        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
            ConstantCreator.createModuleConstants(
                kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
                kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
            ConstantCreator.createModuleConstants(
                kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
                kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
            ConstantCreator.createModuleConstants(
                kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
                kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
            ConstantCreator.createModuleConstants(
                kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
                kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
            );

        /**
         * Creates a CommandSwerveDrivetrain instance.
         * This should only be called once in your robot program,.
         */
        public static Swerve createDrivetrain() {
            return new Swerve(
                DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
            );
        }
    }
}
