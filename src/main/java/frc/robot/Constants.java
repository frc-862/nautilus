package frc.robot;

import java.nio.file.Paths;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionTargetSim;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.Constants.FishingRodConstants.states;
import frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.util.function.Supplier;
import java.util.HashMap;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionTargetSim;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import frc.thunder.hardware.ThunderBird;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

public class Constants {

    public static final String TRITON_IDENTIFIER = "/home/lvuser/triton"; // Differentiate between Triton and Nautlius
    public static final boolean IS_TRITON = Paths.get(TRITON_IDENTIFIER).toFile().exists();

    public static class RobotMap {
        public static final int FL_DRIVE = 1;
        public static final int FL_TURN = 2;
        public static final int FL_ENCODER = 31;

        public static final int FR_DRIVE = 3;
        public static final int FR_TURN = 4;
        public static final int FR_ENCODER = 32;

        public static final int BL_DRIVE = 5;
        public static final int BL_TURN = 6;
        public static final int BL_ENCODER = 33;

        public static final int BR_DRIVE = 7;
        public static final int BR_TURN = 8;
        public static final int BR_ENCODER = 34;

        public static final int L_ELEVATOR = 9; // temp
        public static final int R_ELEVATOR = 10; // temp
        public static final int ELEVATOR_CANRANGE = 41; // temp

        public static final int WRIST = 11; // temp
        public static final int WRIST_ENCODER = 35; // temp

        public static final int COLLECTOR = 12; // temp
        public static final int COLLECTOR_ENCODER = 36; // temp


        public static final int PIGEON = 23;

        public static final String CANIVORE_CAN_NAME = "Canivore";

        // 20ms default loop time
        public static final double UPDATE_FREQ = 0.020;

        public static class ButtonBox {
            public static final int GRAY_TOPLEFT = 5;
            public static final int PINK = 3;
            public static final int GREEN = 4;
            public static final int GRAY_TOPRIGHT = 6;
            public static final int GRAY_BOTTOMLEFT = 2; // AXIS
            public static final int PURPLE = 1;
            public static final int RED = 2;
            public static final int GRAY_BOTTOMRIGHT = 3; // AXIS
            public static final int SHARE = 7;
            public static final int OPTIONS = 8;
            public static final int L3_SL = 9;
            public static final int R3_SL = 10;
        }
    }

    public static class ElevatorConstants {
        public static final boolean BRAKE_MODE = true;
        public static final double STATOR_CURRENT_LIMIT = 0d; // temp
        public static final boolean L_INVERTED = false; // temp
        public static final boolean R_INVERTED = true; // temp

        public static final double GEAR_RATIO = 1 / 4d; // temp
        public static final double ROTOR_TO_SENSOR_RATIO = 1; // temp
        public static final double ENCODER_TO_MECHANISM_RATIO = GEAR_RATIO * Math.PI * 2 * (7 / 11); // TODO: i dont
                                                                                                     // know why the
                                                                                                     // 7/11 made it
                                                                                                     // work, but it
                                                                                                     // did. this is
                                                                                                     // temporary.

        public static final double MOTORS_KP = 0; // temp
        public static final double MOTORS_KI = 0; // temp
        public static final double MOTORS_KD = 0; // temp
        public static final double MOTORS_KF = 0; // temp
        public static final double MOTORS_KS = 0; // temp
        public static final double MOTORS_KV = 0; // temp
        public static final double MOTORS_KA = 0; // temp
        public static final double MOTORS_KG = 0; // temp

        public static final double TOLERANCE = 0.1; // temp

        public static final Distance MIN_EXTENSION = Inches.of(0);
        public static final Distance MAX_EXTENSION = Inches.of(82);

        // SIM
        public static final Mass CARRIAGE_WEIGHT = Pounds.of(7); // temp
        public static final Distance DRUM_RADIUS = Inches.of(0.94); // TODO: ask mr hurley abt this because i have no
                                                                    // clue
        public static final double CUSHION_METERS = 0.05; // stages don't line up perfectly
        public static final double STAGE_LEN_METERS = MAX_EXTENSION.in(Meters) / 3; 
    }

    public static class FishingRodConstants {
        public enum states {
            STOW, L1, L2, L3, L4, SOURCE
        }

        public static final HashMap<states, Double> WRIST_MAP = new HashMap<states, Double>() {
            {
                put(states.STOW, 0d);
                put(states.L1, 0d);
                put(states.L2, -20d);
                put(states.L3, -20d);
                put(states.L4, -80d);
                put(states.SOURCE, 0d);
            }
        };

        public static final HashMap<states, Double> ELEVATOR_MAP = new HashMap<states, Double>() {
            {
                put(states.STOW, 1d);
                put(states.L1, 17.88d);
                put(states.L2, 31.72d);
                put(states.L3, 47.59d);
                put(states.L4, 71.87d);
                put(states.SOURCE, 36.5d);
            }
        };
    }

    public static class RobotMotors {
        public static final ThunderBird wristMotor = new ThunderBird(RobotMap.WRIST, RobotMap.CANIVORE_CAN_NAME,
            WristConstants.INVERTED, WristConstants.STATOR_CURRENT_LIMIT, WristConstants.BRAKE_MODE);
        public static final ThunderBird leftElevatorMotor = new ThunderBird(RobotMap.L_ELEVATOR, RobotMap.CANIVORE_CAN_NAME, ElevatorConstants.L_INVERTED,
                    ElevatorConstants.STATOR_CURRENT_LIMIT, ElevatorConstants.BRAKE_MODE);
        public static final ThunderBird rightElevatorMotor = new ThunderBird(RobotMap.R_ELEVATOR, RobotMap.CANIVORE_CAN_NAME, ElevatorConstants.R_INVERTED,
            ElevatorConstants.STATOR_CURRENT_LIMIT, ElevatorConstants.BRAKE_MODE);

    }   

    public static class WristConstants {
        public static final boolean BRAKE_MODE = true;
        public static final double STATOR_CURRENT_LIMIT = 100d; // temp
        public static final boolean INVERTED = false; // temp

        public static final double ROTOR_TO_ENCODER_RATIO = 10d; // temp
        public static final double ENCODER_TO_MECHANISM_RATIO = 1d;

        public static final double MOTORS_KP = 3; // temp
        public static final double MOTORS_KI = 0; // temp
        public static final double MOTORS_KD = 0; // temp
        public static final double MOTORS_KF = 0; // temp
        public static final double MOTORS_KS = 0; // temp
        public static final double MOTORS_KV = 0; // temp
        public static final double MOTORS_KA = 0; // temp
        public static final double MOTORS_KG = 1.523; // temp

        public static final Angle MIN_ANGLE = Degrees.of(-85);
        public static final Angle MAX_ANGLE = Degrees.of(85);

        public static final double TOLERANCE = 3d;

        //sim stuff
        public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.1096); // 5lb, 2.5in rad, 9in height
        public static final Distance LENGTH = Meters.of(0.18); // TODO: ask mr hurley abt this because i have no clue

    }
    public static class CollectorConstants{
        public static final boolean BRAKE_MODE = true;
        public static final double STATOR_CURRENT_LIMIT = 0d; // temp
        public static final boolean INVERTED = false; // temp

        public static final double GEAR_RATIO = 1d / 2d; // output shaft gear reduction / Motor gear reduction
        public static final double ROTOR_TO_ENCODER_RATIO = GEAR_RATIO * 360; // temp
        public static final double ENCODER_TO_MECHANISM_RATIO = 1d;

    }

    public static class ControllerConstants {
        public static final int DRIVER_CONTROLLER = 0;
        public static final int COPILOT_CONTROLLER = 1;

        public static final double JOYSTICK_DEADBAND = 0.1;
        public static final double TRIGGER_DEADBAND = 0.05;

    }

    public class DrivetrainConstants {
        public static final double MAX_SPEED = TunerConstants.TritonTunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts
                                                                                                                        // desired
                                                                                                                        // top
                                                                                                                        // speed
        public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a
                                                                                                        // rotation per
                                                                                                        // second max
                                                                                                        // angular
                                                                                                        // velocity
        public static final double SLOW_MODE_MULT = 0.3; // 3/4 of a rotation per second max angular velocity

        public class DriveRequests {
            private static final SwerveRequest.FieldCentric DRIVE = new SwerveRequest.FieldCentric();
            private static final SwerveRequest.FieldCentric SLOW = new SwerveRequest.FieldCentric();
            private static final SwerveRequest.RobotCentric ROBO_CENTRIC = new SwerveRequest.RobotCentric();
            private static final SwerveRequest.SwerveDriveBrake BRAKE = new SwerveRequest.SwerveDriveBrake();

            public static Supplier<SwerveRequest> getDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
                return () -> DRIVE
                        .withVelocityX(y.getAsDouble() * DrivetrainConstants.MAX_SPEED) // Drive forward with negative Y
                                                                                        // (forward)
                        .withVelocityY(x.getAsDouble() * DrivetrainConstants.MAX_SPEED) // Drive left with negative X
                                                                                        // (left)
                        .withRotationalRate(rot.getAsDouble() * DrivetrainConstants.MAX_ANGULAR_RATE)
                        .withDeadband(DrivetrainConstants.MAX_SPEED * 0.1)
                        .withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Drive counterclockwise with negative
                                                                                 // X (left)

            }

            public static Supplier<SwerveRequest> getSlow(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
                return () -> SLOW
                        .withVelocityX(y.getAsDouble() * DrivetrainConstants.MAX_SPEED * SLOW_MODE_MULT) // Drive
                                                                                                         // forward with
                                                                                                         // negative Y
                                                                                                         // (forward)
                        .withVelocityY(x.getAsDouble() * DrivetrainConstants.MAX_SPEED * SLOW_MODE_MULT) // Drive left
                                                                                                         // with
                                                                                                         // negative X
                                                                                                         // (left)
                        .withRotationalRate(rot.getAsDouble() * DrivetrainConstants.MAX_ANGULAR_RATE * SLOW_MODE_MULT)
                        .withDeadband(DrivetrainConstants.MAX_SPEED * 0.1)
                        .withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Drive counterclockwise with negative
                                                                                 // X (left)

            }

            public static Supplier<SwerveRequest> getRobotCentric(DoubleSupplier x, DoubleSupplier y,
                    DoubleSupplier rot) {
                return () -> ROBO_CENTRIC
                        .withVelocityX(y.getAsDouble() * DrivetrainConstants.MAX_SPEED) // Drive forward with negative Y
                                                                                        // (forward)
                        .withVelocityY(x.getAsDouble() * DrivetrainConstants.MAX_SPEED) // Drive left with negative X
                                                                                        // (left)
                        .withRotationalRate(rot.getAsDouble() * DrivetrainConstants.MAX_ANGULAR_RATE)
                        .withDeadband(DrivetrainConstants.MAX_SPEED * 0.1)
                        .withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Drive counterclockwise with negative
                                                                                 // X (left)
            }

            public static Supplier<SwerveRequest> getBrake() {
                return () -> BRAKE;
            }

            // auto request exists in Swerve.java
        }

    }

    public static class AutonomousConstants {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(10, 0, 0);
        public static final PIDConstants ROTATION_PID = new PIDConstants(5, 0, 0);

        private static final double TRACK_WIDTH = Units.inchesToMeters(27); // TODO: make more accurate
        private static final Mass ROBOT_MASS = Pounds.of(147);
        private static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(5.2268411); // TODO: this assumes even
                                                                                             // weight distribution;
                                                                                             // should be calculated w/
                                                                                             // SYSID or CAD
        private static final ModuleConfig MODULE_CONFIG = new ModuleConfig(
                TunerConstants.TritonTunerConstants.kWheelRadius, TunerConstants.TritonTunerConstants.kSpeedAt12Volts,
                1.916, DCMotor.getKrakenX60Foc(1).withReduction(TunerConstants.TritonTunerConstants.kDriveGearRatio),
                Amps.of(120), 1);

        public static final RobotConfig CONFIG = new RobotConfig(ROBOT_MASS, ROBOT_MOI, MODULE_CONFIG,
                new Translation2d[] { new Translation2d(TRACK_WIDTH / 2, TRACK_WIDTH / 2),
                        new Translation2d(TRACK_WIDTH / 2, -TRACK_WIDTH / 2),
                        new Translation2d(-TRACK_WIDTH / 2, TRACK_WIDTH / 2),
                        new Translation2d(-TRACK_WIDTH / 2, -TRACK_WIDTH / 2) });
    }

    public static class VisionConstants {
        public static final String camera1Name = "cam1";
        public static final TargetModel targetModel = TargetModel.kAprilTag36h11;
        public static final Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
        public static final VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);
        public static final AprilTagFieldLayout tagLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2024Crescendo);
        public static final SimCameraProperties cameraProp = new SimCameraProperties();
        public static final Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
        public static final Rotation3d robotToCameraRot = new Rotation3d(0, 0, 0);
        public static final Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

    }

    public static class PoseConstants {
        public static final Translation2d FIELD_LIMIT = new Translation2d(Units.feetToMeters(54.0),
                Units.feetToMeters(26.0));

        public static final Pose2d REEFSCORE1_1 = new Pose2d(4.864, 5.235, new Rotation2d(240));
        public static final Pose2d REEFSCORE1_2 = new Pose2d(5.283, 5.069, new Rotation2d(240));
        public static final Pose2d REEFSCORE2_1 = new Pose2d(5.837, 4.203, new Rotation2d(180));
        public static final Pose2d REEFSCORE2_2 = new Pose2d(5.89, 3.876, new Rotation2d(180));
        public static final Pose2d REEFSCORE3_1 = new Pose2d(5.327, 2.935, new Rotation2d(120));
        public static final Pose2d REEFSCORE3_2 = new Pose2d(5.019, 2.803, new Rotation2d(120));
        public static final Pose2d REEFSCORE4_1 = new Pose2d(3.881, 2.728, new Rotation2d(60));
        public static final Pose2d REEFSCORE4_2 = new Pose2d(3.639, 2.904, new Rotation2d(60));
        public static final Pose2d REEFSCORE5_1 = new Pose2d(3.072, 3.875, new Rotation2d(0));
        public static final Pose2d REEFSCORE5_2 = new Pose2d(3.101, 4.175, new Rotation2d(0));
        public static final Pose2d REEFSCORE6_1 = new Pose2d(3.656, 5.122, new Rotation2d(300));
        public static final Pose2d REEFSCORE6_2 = new Pose2d(3.949, 5.282, new Rotation2d(300));

        public enum ScoringPoses {
            REEFSCORE1_1, REEFSCORE1_2, REEFSCORE2_1, REEFSCORE2_2, REEFSCORE3_1, REEFSCORE3_2, 
            REEFSCORE4_1, REEFSCORE4_2, REEFSCORE5_1, REEFSCORE5_2, REEFSCORE6_1, REEFSCORE6_2
        }

        public static HashMap<ScoringPoses, Pose2d> poseHashMap = new HashMap<ScoringPoses, Pose2d>(){
            {
                put(ScoringPoses.REEFSCORE1_1, REEFSCORE1_1);
                put(ScoringPoses.REEFSCORE1_2, REEFSCORE1_2);
                put(ScoringPoses.REEFSCORE2_1, REEFSCORE2_1);
                put(ScoringPoses.REEFSCORE2_2, REEFSCORE2_2);
                put(ScoringPoses.REEFSCORE3_1, REEFSCORE3_1);
                put(ScoringPoses.REEFSCORE3_2, REEFSCORE3_2);
                put(ScoringPoses.REEFSCORE4_1, REEFSCORE4_1);
                put(ScoringPoses.REEFSCORE4_2, REEFSCORE4_2);
                put(ScoringPoses.REEFSCORE5_1, REEFSCORE5_1);
                put(ScoringPoses.REEFSCORE5_2, REEFSCORE5_2);
                put(ScoringPoses.REEFSCORE6_1, REEFSCORE6_1);
                put(ScoringPoses.REEFSCORE6_2, REEFSCORE6_2);

            }
        };

        public static final PathConstraints PATHFINDING_CONSTRAINTS = new PathConstraints(2.0, 1.0, 3.0, 1.5);

    }

    public class TunerConstants {

        public class NautliusTunerConstants {

        }

        public class TritonTunerConstants {
            // Both sets of gains need to be tuned to your individual robot.

            // The steer motor uses any SwerveModule.SteerRequestType control request with
            // the
            // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
            private static final Slot0Configs steerGains = new Slot0Configs()
                    .withKP(100).withKI(0).withKD(0.5)
                    .withKS(0.1).withKV(2.66).withKA(0)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
            // When using closed-loop control, the drive motor uses the control
            // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
            private static final Slot0Configs driveGains = new Slot0Configs()
                    .withKP(0.34807).withKI(0).withKD(0)
                    .withKS(0.18408).withKV(0.11928).withKA(0.0022307);

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
            // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to
            // RemoteCANcoder
            private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

            // The stator current at which the wheels start to slip;
            // This needs to be tuned to your individual robot
            private static final Current kSlipCurrent = Amps.of(120.0);

            // Initial configs for the drive and steer motors and the azimuth encoder; these
            // cannot be null.
            // Some configs will be overwritten; check the `with*InitialConfigs()` API
            // documentation.
            private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
            private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
                    .withCurrentLimits(
                            new CurrentLimitsConfigs()
                                    // Swerve azimuth does not require much torque output, so we can set a
                                    // relatively low
                                    // stator current limit to help avoid brownouts without impacting performance.
                                    .withStatorCurrentLimit(Amps.of(60))
                                    .withStatorCurrentLimitEnable(true));
            private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
            // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
            private static final Pigeon2Configuration pigeonConfigs = null;

            // CAN bus that the devices are located on;
            // All swerve devices must share the same CAN bus
            public static final CANBus kCANBus = new CANBus(RobotMap.CANIVORE_CAN_NAME, "./logs/example.hoot");

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

            private static final int kPigeonId = RobotMap.PIGEON;

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

            private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
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
            private static final int kFrontLeftDriveMotorId = RobotMap.FL_DRIVE;
            private static final int kFrontLeftSteerMotorId = RobotMap.FL_TURN;
            private static final int kFrontLeftEncoderId = RobotMap.FL_ENCODER;
            private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.0073);
            private static final boolean kFrontLeftSteerMotorInverted = true;
            private static final boolean kFrontLeftEncoderInverted = false;

            private static final Distance kFrontLeftXPos = Inches.of(13.5);
            private static final Distance kFrontLeftYPos = Inches.of(13.5);

            // Front Right
            private static final int kFrontRightDriveMotorId = RobotMap.FR_DRIVE;
            private static final int kFrontRightSteerMotorId = RobotMap.FR_TURN;
            private static final int kFrontRightEncoderId = RobotMap.FR_ENCODER;
            private static final Angle kFrontRightEncoderOffset = Rotations.of(0.033447);
            private static final boolean kFrontRightSteerMotorInverted = true;
            private static final boolean kFrontRightEncoderInverted = false;

            private static final Distance kFrontRightXPos = Inches.of(13.5);
            private static final Distance kFrontRightYPos = Inches.of(-13.5);

            // Back Left
            private static final int kBackLeftDriveMotorId = RobotMap.BL_DRIVE;
            private static final int kBackLeftSteerMotorId = RobotMap.BL_TURN;
            private static final int kBackLeftEncoderId = RobotMap.BL_ENCODER;
            private static final Angle kBackLeftEncoderOffset = Rotations.of(0.1350);
            private static final boolean kBackLeftSteerMotorInverted = true;
            private static final boolean kBackLeftEncoderInverted = false;

            private static final Distance kBackLeftXPos = Inches.of(-13.5);
            private static final Distance kBackLeftYPos = Inches.of(13.5);

            // Back Right
            private static final int kBackRightDriveMotorId = RobotMap.BR_DRIVE;
            private static final int kBackRightSteerMotorId = RobotMap.BR_TURN;
            private static final int kBackRightEncoderId = RobotMap.BR_ENCODER;
            private static final Angle kBackRightEncoderOffset = Rotations.of(0.129395);
            private static final boolean kBackRightSteerMotorInverted = true;
            private static final boolean kBackRightEncoderInverted = false;

            private static final Distance kBackRightXPos = Inches.of(-13.5);
            private static final Distance kBackRightYPos = Inches.of(-13.5);

            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft = ConstantCreator
                    .createModuleConstants(
                            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId,
                            kFrontLeftEncoderOffset,
                            kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted,
                            kFrontLeftEncoderInverted);
            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight = ConstantCreator
                    .createModuleConstants(
                            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
                            kFrontRightEncoderOffset,
                            kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted,
                            kFrontRightEncoderInverted);
            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft = ConstantCreator
                    .createModuleConstants(
                            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
                            kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted,
                            kBackLeftEncoderInverted);
            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight = ConstantCreator
                    .createModuleConstants(
                            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId,
                            kBackRightEncoderOffset,
                            kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted,
                            kBackRightEncoderInverted);

            /**
             * Creates a CommandSwerveDrivetrain instance.
             * This should only be called once in your robot program,.
             * 
             * @return Swerve
             */
            public static Swerve createDrivetrain() {
                return new Swerve(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
            }
        }

        public static Swerve createDrivetrain() {
            return IS_TRITON || Robot.isSimulation() ? TritonTunerConstants.createDrivetrain() : null;
        }
    }

    public static class LEDConstants {
        public static final int LED_PWM_PORT = 0;
        public static final int LED_LENGTH = 13;
        public static final int LED_BUFFER_TIME = 60;

        public static final int SWRIL_SEGMENT_SIZE = 5;

        public static final int RED_HUE = 0;
        public static final int ORANGE_HUE = 5;
        public static final int YELLOW_HUE = 15;
        public static final int GREEN_HUE = 240;
        public static final int LIGHT_BLUE_HUE = 195;
        public static final int BLUE_HUE = 120;
        public static final int PURPLE_HUE = 315;
        public static final int PINK_HUE = 355;

        public enum LED_STATES {
            DISABLED(),
            MIXER(),
            RAINBOW(),
            ALIGNING(),
            ALGAE_COLLECT(),
            ALGAE_SCORE(),
            CORAL_COLLECT(),
            CORAL_SCORE(),
            ROD_MOVING(),
            ROD_ON_TARGET(),
            OFF();
        }

    }
}
