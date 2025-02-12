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
import frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;

import frc.thunder.hardware.ThunderBird;

public class Constants {

    public static final String TRITON_IDENTIFIER = "/home/lvuser/triton"; // Differentiate between Triton and Nautlius
    public static final boolean IS_TRITON = Paths.get(TRITON_IDENTIFIER).toFile().exists();

    public enum RobotIdentifiers {
        NAUTILUS, TRITON, SIM
    };

    // note from kyle: wtf is this?? :sob:
    // yeah its a double inline ternary operator fight me
    // if(isTriton); else if(isSim); else, its nautilus
    public static final RobotIdentifiers ROBOT_IDENTIFIER = Paths.get(TRITON_IDENTIFIER).toFile().exists() ? RobotIdentifiers.TRITON
            : Robot.isSimulation() ? RobotIdentifiers.SIM : RobotIdentifiers.NAUTILUS;

    public static class EncoderConstants {
        // Nautilus values
        private static final Angle nautilusKFrontLeftEncoderOffset = Rotations.of(0.25732421875);
        private static final Angle nautilusKFrontRightEncoderOffset = Rotations.of(0.412353515625);
        private static final Angle nautilusKBackLeftEncoderOffset = Rotations.of(0.0693359375);
        private static final Angle nautilusKBackRightEncoderOffset = Rotations.of(-0.30517578125);

        // Triton values
        private static final Angle tritonKFrontLeftEncoderOffset = Rotations.of(0.0073);
        private static final Angle tritonKFrontRightEncoderOffset = Rotations.of(0.033447);
        private static final Angle tritonKBackLeftEncoderOffset = Rotations.of(0.1350);
        private static final Angle tritonKBackRightEncoderOffset = Rotations.of(0.129395);

        public static final double tritonWristOffset = -0.246;

        // Generic values
        public static final double frontLeftOffset = IS_TRITON ? tritonKFrontLeftEncoderOffset.in(Rotations)
                : nautilusKFrontLeftEncoderOffset.in(Rotations);
        public static final double frontRightOffset = IS_TRITON ? tritonKFrontRightEncoderOffset.in(Rotations)
                : nautilusKFrontRightEncoderOffset.in(Rotations);
        public static final double backLeftOffset = IS_TRITON ? tritonKBackLeftEncoderOffset.in(Rotations)
                : nautilusKBackLeftEncoderOffset.in(Rotations);
        public static final double backRightOffset = IS_TRITON ? tritonKBackRightEncoderOffset.in(Rotations)
                : nautilusKBackRightEncoderOffset.in(Rotations);

    }

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

        public static final int L_ELEVATOR = 10;
        public static final int R_ELEVATOR = 9;
        public static final int ELEVATOR_CANRANGE = 41;

        public static final int WRIST = 11;
        public static final int WRIST_ENCODER = 35;

        public static final int CORAL_COLLECTOR = 12;
        public static final int CORAL_COLLECTOR_ENCODER = 36;
        public static final int CORAL_COLLECTOR_BEAM_BREAK_DIO = 0; // temp

        public static final int ALGAE_COLLECTOR_ROLLER = 13; // temp
        public static final int ALGAE_COLLECTOR_PIVOT = 14; // temp

        public static final int CLIMBER = 15;

        public static final int PIGEON = 23;

        public static final String CANIVORE_CAN_NAME = "Canivore";
        // 20ms default loop time
        public static final double UPDATE_FREQ = 0.020;
    }

    // TODO: maybe put these in their own subsystem constants?
    public static class RobotMotors {
        public static final ThunderBird wristMotor = new ThunderBird(RobotMap.WRIST, RobotMap.CANIVORE_CAN_NAME,
                WristConstants.INVERTED, WristConstants.STATOR_CURRENT_LIMIT, WristConstants.BRAKE_MODE);
        public static final ThunderBird leftElevatorMotor = new ThunderBird(RobotMap.L_ELEVATOR,
                RobotMap.CANIVORE_CAN_NAME, ElevatorConstants.L_INVERTED,
                ElevatorConstants.STATOR_CURRENT_LIMIT, ElevatorConstants.BRAKE_MODE);
        public static final ThunderBird rightElevatorMotor = new ThunderBird(RobotMap.R_ELEVATOR,
                RobotMap.CANIVORE_CAN_NAME, ElevatorConstants.R_INVERTED,
                ElevatorConstants.STATOR_CURRENT_LIMIT, ElevatorConstants.BRAKE_MODE);
        public static final ThunderBird coralCollectorMotor = new ThunderBird(RobotMap.CORAL_COLLECTOR,
                RobotMap.CANIVORE_CAN_NAME, CoralCollectorConstants.INVERTED,
                CoralCollectorConstants.STATOR_CURRENT_LIMIT, CoralCollectorConstants.BRAKE_MODE);
        public static final ThunderBird algaeCollectorPivotMotor = new ThunderBird(RobotMap.ALGAE_COLLECTOR_PIVOT,
                RobotMap.CANIVORE_CAN_NAME, AlgaeCollectorConstants.PIVOT_INVERTED,
                AlgaeCollectorConstants.PIVOT_STATOR_CURRENT_LIMIT, AlgaeCollectorConstants.PIVOT_BRAKE_MODE);
        public static final ThunderBird algaeCollectorRollerMotor = new ThunderBird(RobotMap.ALGAE_COLLECTOR_ROLLER,
                RobotMap.CANIVORE_CAN_NAME, AlgaeCollectorConstants.ROLLER_INVERTED,
                AlgaeCollectorConstants.ROLLER_STATOR_CURRENT_LIMIT, AlgaeCollectorConstants.ROLLER_BRAKE_MODE);
        public static final ThunderBird climberMotor = new ThunderBird(RobotMap.CLIMBER, RobotMap.CANIVORE_CAN_NAME,
                ClimberConstants.INVERTED,
                ClimberConstants.STATOR_CURRENT_LIMIT, ClimberConstants.BREAK_MODE);
    }

    public static class FishingRodConstants {
        public enum RodStates {
            STOW, L1, L2, L3, L4, SOURCE, LOW, HIGH, ALGAE_SCORE
        }

        public enum RodTransitionStates {
            DEFAULT, // default travel state
            X_SCORE, // any state to L4
            SCORE_X, // L4 to any state
            TRITON // specific state to deal with triton's loose belt
        }

        public static final HashMap<RodStates, Double> WRIST_MAP = new HashMap<RodStates, Double>() {
            {
                // put(ROD_STATES.STOW, 0d);
                // put(ROD_STATES.L1, 0d);
                // put(ROD_STATES.L2, -20d);
                // put(ROD_STATES.L3, -20d);
                // put(ROD_STATES.L4, -80d);
                // put(ROD_STATES.SOURCE, 0d);

                put(RodStates.STOW, 81d);
                put(RodStates.L1, 0d);
                put(RodStates.L2, -35d);
                put(RodStates.L3, -35d);
                put(RodStates.L4, -47d);
                put(RodStates.SOURCE, 65d);
            }
        };

        public static final HashMap<RodStates, Double> ELEVATOR_MAP = new HashMap<RodStates, Double>() {
            {
                // put(ROD_STATES.STOW, 1d);
                // put(ROD_STATES.L1, 17.88d);
                // put(ROD_STATES.L2, 31.72d);
                // put(ROD_STATES.L3, 47.59d);
                // put(ROD_STATES.L4, 71.87d);
                // put(ROD_STATES.SOURCE, 36.5d);

                put(RodStates.STOW, 3d);
                put(RodStates.L1, 10d);
                put(RodStates.L2, 13d);
                put(RodStates.L3, 26d);
                put(RodStates.L4, 46.5d);
                put(RodStates.SOURCE, 6d);
            }
        };
    }

    public static class ElevatorConstants {
        public static final boolean BRAKE_MODE = true;
        // both motors are - to go up
        public static final boolean L_INVERTED = false;
        public static final boolean R_INVERTED = true;
        
        public static final double STATOR_CURRENT_LIMIT = 120d; // temp

        public static final double GEAR_RATIO = 4d;
        public static final Distance DRUM_RADIUS = Millimeter.of(15);
        public static final double DRUM_CIRCUMFERENCE = Math.PI * 2 * DRUM_RADIUS.in(Inches);
        public static final double ROTOR_TO_SENSOR_RATIO = 1; // temp
        public static final double ENCODER_TO_MECHANISM_RATIO = DRUM_CIRCUMFERENCE / GEAR_RATIO;

        public static final double MOTORS_KP = 6.5; // temp
        public static final double MOTORS_KI = 0; // temp
        public static final double MOTORS_KD = 0; // temp
        public static final double MOTORS_KF = 0; // temp
        public static final double MOTORS_KS = 1; // temp
        public static final double MOTORS_KV = 0.18;// temp
        public static final double MOTORS_KA = 0.01; // temp
        public static final double MOTORS_KG = 0d; // temp

        public static final double VELOC = 80d; // 80
        public static final double ACCEL = 400d; // 200
        public static final double JERK = 1600d; // temp

        public static final double TOLERANCE = 0.1; // temp

        // kind of guessing the numbers here (didn't do a proper test)
        public static final Distance MIN_EXTENSION = Inches.of(0);
        public static final Distance MAX_EXTENSION = Inches.of(47);

        // SIM
        public static final Mass CARRIAGE_WEIGHT = Pounds.of(30); // temp
        public static final double CUSHION_METERS = 0.05; // stages don't line up perfectly
        public static final double STAGE_LEN_METERS = MAX_EXTENSION.in(Meters) / 3;
    }

    public static class WristConstants {
        public static final boolean BRAKE_MODE = true;
        public static final double STATOR_CURRENT_LIMIT = 100d; // temp
        public static final boolean INVERTED = false; // temp

        public static final double ROTOR_TO_ENCODER_RATIO = 36d; // temp
        public static final double ENCODER_TO_MECHANISM_RATIO = 1d;

        public static final double MOTORS_KP = 15; // temp
        public static final double MOTORS_KI = 0; // temp
        public static final double MOTORS_KD = 0; // temp
        public static final double MOTORS_KF = 0; // temp
        public static final double MOTORS_KS = 0; // temp
        public static final double MOTORS_KV = 0; // temp
        public static final double MOTORS_KA = 0; // temp
        public static final double MOTORS_KG = 0.3; // temp

        public static final Angle MIN_ANGLE = Degrees.of(-85);
        public static final Angle MAX_ANGLE = Degrees.of(85);

        public static final double TOLERANCE = 10d;

        // sim stuff
        public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.086); // 5lb, 2.5in rad, 9in height
        public static final Distance LENGTH = Meters.of(0.18); // TODO: ask mr hurley abt this because i have no clue

    }

    public static class CoralCollectorConstants {
        public static final boolean BRAKE_MODE = true;
        public static final double STATOR_CURRENT_LIMIT = 100d; // temp
        public static final boolean INVERTED = false; // temp
        public static final double CORAL_ROLLER_SPEED = 1;
        public static final double DEBOUNCE_TIME = 0.1;

        public static final double GEAR_RATIO = 1d / 2d; // output shaft gear reduction / Motor gear reduction
        public static final double ROTOR_TO_ENCODER_RATIO = GEAR_RATIO * 360; // temp
        public static final double ENCODER_TO_MECHANISM_RATIO = 1d;

        public static final double KV = 0.24; // temp
        public static final double KA = 0.8; // temp

        public static final double BEAMBREAK_DEBOUNCE = 0.1;

        public static final double COLLECTED_CURRENT = 80d;
    }

    public class DrivetrainConstants {
        public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts
                                                                                                   // desired
                                                                                                   // top
                                                                                                   // speed
        public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a
                                                                                                        // rotation per
                                                                                                        // second max
                                                                                                        // angular
                                                                                                        // velocity
        public static final double SLOW_MODE_MULT = 0.3; // 3/4 of a rotation per second max angular velocity

        public static final double SLOW_SPEED_MULT = 0.4; // temp
        public static final double SLOW_TURN_MULT = 0.7; // temp

        public enum SysIdTestType {
            DRIVE,
            STEER,
            ROTATE
        }

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

            public static SwerveRequest getDrive(double x, double y, double rot) {
                return DRIVE
                        .withVelocityX(y * DrivetrainConstants.MAX_SPEED) // Drive forward with negative Y (forward)
                        .withVelocityY(x * DrivetrainConstants.MAX_SPEED) // Drive left with negative X (left)
                        .withRotationalRate(rot * DrivetrainConstants.MAX_ANGULAR_RATE)
                        .withDeadband(DrivetrainConstants.MAX_SPEED * 0.1)
                        .withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Drive counterclockwise with negative
                                                                                 // X
                                                                                 // (left)
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

            public static SwerveRequest getRobotCentric(double x, double y, double rot) {
                return ROBO_CENTRIC
                        .withVelocityX(y * DrivetrainConstants.MAX_SPEED) // Drive forward with negative Y (forward)
                        .withVelocityY(x * DrivetrainConstants.MAX_SPEED) // Drive left with negative X (left)
                        .withRotationalRate(rot * DrivetrainConstants.MAX_ANGULAR_RATE)
                        .withDeadband(DrivetrainConstants.MAX_SPEED * 0.1)
                        .withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Drive counterclockwise with negative
                                                                                 // X
                                                                                 // (left)
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
                TunerConstants.kWheelRadius, TunerConstants.kSpeedAt12Volts,
                1.916, DCMotor.getKrakenX60Foc(1).withReduction(TunerConstants.kDriveGearRatio),
                Amps.of(120), 1);

        public static final RobotConfig CONFIG = new RobotConfig(ROBOT_MASS, ROBOT_MOI, MODULE_CONFIG,
                new Translation2d[] { new Translation2d(TRACK_WIDTH / 2, TRACK_WIDTH / 2),
                        new Translation2d(TRACK_WIDTH / 2, -TRACK_WIDTH / 2),
                        new Translation2d(-TRACK_WIDTH / 2, TRACK_WIDTH / 2),
                        new Translation2d(-TRACK_WIDTH / 2, -TRACK_WIDTH / 2) });
    }

    public static class VisionConstants {
        public static final String leftCamName = "cam1";
        public static final String rightCamName = "cam2";
        public static final TargetModel targetModel = TargetModel.kAprilTag36h11;
        public static final Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
        public static final VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);
        public static final AprilTagFieldLayout tagLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025Reefscape);
        public static final SimCameraProperties cameraProp = new SimCameraProperties();
        public static final Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
        public static final Rotation3d robotToCameraRot = new Rotation3d(0, 0, Math.PI);
        public static final Transform3d robotLeftToCamera = new Transform3d(new Translation3d(-5.772, 11.281, 12).times(0.0254), robotToCameraRot);
        public static final Transform3d robotRightToCamera = new Transform3d(new Translation3d(-5.772, -11.281, 12).times(0.0254), robotToCameraRot);

        // 
        //

        public static final double VISION_X_STDEV = 1;
        public static final double VISION_Y_STDEV = 1;
        public static final double VISION_THETA_STDEV = 1;
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

        public static HashMap<ScoringPoses, Pose2d> poseHashMap = new HashMap<ScoringPoses, Pose2d>() {
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
            // Both sets of gains need to be tuned to your individual robot.

            // The steer motor uses any SwerveModule.SteerRequestType control request with
            // the
            // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
            private static final Slot0Configs steerGains = new Slot0Configs()
                    .withKP(20).withKI(0).withKD(0.5)
                    .withKS(0.224).withKV(2.6).withKA(0)
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
            public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.73);

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
            private static final Angle kFrontLeftEncoderOffset = EncoderConstants.nautilusKFrontLeftEncoderOffset;
            private static final boolean kFrontLeftSteerMotorInverted = true;
            private static final boolean kFrontLeftEncoderInverted = false;

            private static final Distance kFrontLeftXPos = Inches.of(11);
            private static final Distance kFrontLeftYPos = Inches.of(11);

            // Front Right
            private static final int kFrontRightDriveMotorId = RobotMap.FR_DRIVE;
            private static final int kFrontRightSteerMotorId = RobotMap.FR_TURN;
            private static final int kFrontRightEncoderId = RobotMap.FR_ENCODER;
            private static final Angle kFrontRightEncoderOffset = EncoderConstants.nautilusKFrontRightEncoderOffset;
            private static final boolean kFrontRightSteerMotorInverted = true;
            private static final boolean kFrontRightEncoderInverted = false;

            private static final Distance kFrontRightXPos = Inches.of(11);
            private static final Distance kFrontRightYPos = Inches.of(-11);

            // Back Left
            private static final int kBackLeftDriveMotorId = RobotMap.BL_DRIVE;
            private static final int kBackLeftSteerMotorId = RobotMap.BL_TURN;
            private static final int kBackLeftEncoderId = RobotMap.BL_ENCODER;
            private static final Angle kBackLeftEncoderOffset = EncoderConstants.nautilusKBackLeftEncoderOffset;
            private static final boolean kBackLeftSteerMotorInverted = true;
            private static final boolean kBackLeftEncoderInverted = false;

            private static final Distance kBackLeftXPos = Inches.of(-11);
            private static final Distance kBackLeftYPos = Inches.of(11);

            // Back Right
            private static final int kBackRightDriveMotorId = RobotMap.BR_DRIVE;
            private static final int kBackRightSteerMotorId = RobotMap.BR_DRIVE;
            private static final int kBackRightEncoderId = RobotMap.BR_ENCODER;
            private static final Angle kBackRightEncoderOffset = EncoderConstants.nautilusKBackRightEncoderOffset;
            private static final boolean kBackRightSteerMotorInverted = true;
            private static final boolean kBackRightEncoderInverted = false;

            private static final Distance kBackRightXPos = Inches.of(-11);
            private static final Distance kBackRightYPos = Inches.of(-11);

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
                return new Swerve(
                        DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
            }
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
                    .withKS(0.23986).withKV(0.12318).withKA(0.0059707);

            // The closed-loop output type to use for the steer motors;
            // This affects the PID/FF gains for the steer motors
            private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
            // The closed-loop output type to use for the drive motors;
            // This affects the PID/FF gains for the drive motors
            private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;

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
            private static final Angle kFrontLeftEncoderOffset = EncoderConstants.tritonKFrontLeftEncoderOffset;
            private static final boolean kFrontLeftSteerMotorInverted = true;
            private static final boolean kFrontLeftEncoderInverted = false;

            private static final Distance kFrontLeftXPos = Inches.of(11);
            private static final Distance kFrontLeftYPos = Inches.of(11);

            // Front Right
            private static final int kFrontRightDriveMotorId = RobotMap.FR_DRIVE;
            private static final int kFrontRightSteerMotorId = RobotMap.FR_TURN;
            private static final int kFrontRightEncoderId = RobotMap.FR_ENCODER;
            private static final Angle kFrontRightEncoderOffset = EncoderConstants.tritonKFrontRightEncoderOffset;
            private static final boolean kFrontRightSteerMotorInverted = true;
            private static final boolean kFrontRightEncoderInverted = false;

            private static final Distance kFrontRightXPos = Inches.of(11);
            private static final Distance kFrontRightYPos = Inches.of(-11);

            // Back Left
            private static final int kBackLeftDriveMotorId = RobotMap.BL_DRIVE;
            private static final int kBackLeftSteerMotorId = RobotMap.BL_TURN;
            private static final int kBackLeftEncoderId = RobotMap.BL_ENCODER;
            private static final Angle kBackLeftEncoderOffset = EncoderConstants.tritonKBackLeftEncoderOffset;
            private static final boolean kBackLeftSteerMotorInverted = true;
            private static final boolean kBackLeftEncoderInverted = false;

            private static final Distance kBackLeftXPos = Inches.of(-11);
            private static final Distance kBackLeftYPos = Inches.of(11);

            // Back Right
            private static final int kBackRightDriveMotorId = RobotMap.BR_DRIVE;
            private static final int kBackRightSteerMotorId = RobotMap.BR_TURN;
            private static final int kBackRightEncoderId = RobotMap.BR_ENCODER;
            private static final Angle kBackRightEncoderOffset = EncoderConstants.tritonKBackRightEncoderOffset;
            private static final boolean kBackRightSteerMotorInverted = true;
            private static final boolean kBackRightEncoderInverted = false;

            private static final Distance kBackRightXPos = Inches.of(-11);
            private static final Distance kBackRightYPos = Inches.of(-11);

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

        // Combined Nautilus and Triton Tuner Constants

        public static final Current kSlipCurrent = IS_TRITON
                ? TritonTunerConstants.kSlipCurrent
                : NautliusTunerConstants.kSlipCurrent;

        public static final LinearVelocity kSpeedAt12Volts = IS_TRITON
                ? TritonTunerConstants.kSpeedAt12Volts
                : NautliusTunerConstants.kSpeedAt12Volts;

        public static final double kCoupleRatio = IS_TRITON
                ? TritonTunerConstants.kCoupleRatio
                : NautliusTunerConstants.kCoupleRatio;

        public static final double kDriveGearRatio = IS_TRITON
                ? TritonTunerConstants.kDriveGearRatio
                : NautliusTunerConstants.kDriveGearRatio;
        public static final double kSteerGearRatio = IS_TRITON
                ? TritonTunerConstants.kSteerGearRatio
                : NautliusTunerConstants.kSteerGearRatio;

        public static final Distance kWheelRadius = IS_TRITON
                ? TritonTunerConstants.kWheelRadius
                : NautliusTunerConstants.kWheelRadius;

        public static final boolean kInvertLeftSide = IS_TRITON
                ? TritonTunerConstants.kInvertLeftSide
                : NautliusTunerConstants.kInvertLeftSide;
        public static final boolean kInvertRightSide = IS_TRITON
                ? TritonTunerConstants.kInvertRightSide
                : NautliusTunerConstants.kInvertRightSide;

        public static Swerve createDrivetrain() {
            return IS_TRITON ? TritonTunerConstants.createDrivetrain() : NautliusTunerConstants.createDrivetrain();
        }
    }

    public static class ControllerConstants {
        public static final int DRIVER_CONTROLLER = 0;
        public static final int COPILOT_CONTROLLER = 1;

        public static final double JOYSTICK_DEADBAND = 0.1;
        public static final double TRIGGER_DEADBAND = 0.05;

        public static class ButtonBoxBindings {
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

    public static class LEDConstants {
        public static final int PWM_PORT = 0;
        public static final int LENGTH = 60;

        public static final int SWRIL_SEGMENT_SIZE = 5;

        public enum LEDStates {
            DISABLED(),
            MIXER(),
            RAINBOW(),
            ALIGNING(),
            ALGAE_COLLECT(),
            ALGAE_SCORE(),
            CORAL_COLLECT(),
            CORAL_SCORE(),
            ROD_MOVING(),
            OFF();
        }
    }

    public class AutoAlignConstants {
        public static final double AutoAlignTolerance = 0.02d;

        // X PID
        public static final double X_Kp = 0.035d;
        public static final double X_Ki = 0d;
        public static final double X_Kd = 0d;

        // Y PID
        public static final double Y_Kp = 0.1d;
        public static final double Y_Ki = 0d;
        public static final double Y_Kd = 0d;

        public static final double targetTX = 720d;

        public static final HashMap<Integer, Double> tagAngles = new HashMap<Integer, Double>() {
            {
                put(6, 0d);
                put(9, 0d);
            }
        };

    }

    public class SimGamePeicesConstants {
        public static final Pose3d A1B = new Pose3d(1.216, 5.850, 0.5, new Rotation3d(0, 0, 0));
        public static final Pose3d A2B = new Pose3d(1.216, 4.020, 0.5, new Rotation3d(0, 0, 0));
        public static final Pose3d A3B = new Pose3d(1.216, 2.190, 0.5, new Rotation3d(0, 0, 0));
        public static final Pose3d A1R = new Pose3d(16.330, 2.190, 0.5,  new Rotation3d(0, 0, 0));
        public static final Pose3d A2R = new Pose3d(16.330, 4.020, 0.5, new Rotation3d(0, 0, 0));
        public static final Pose3d A3R = new Pose3d(16.330, 5.850, 0.5, new Rotation3d(0, 0, 0));

        public static final Pose3d DEFAULT_POSE = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

        public static final double ELEATOR_ROOT_HEIGHT = 0.5;
        public static final double COLLECTION_TOLERANCE = 0.5;
        public static final double ALGAE_COLLECTOR_ROOT_HEIGHT = 0.33;
        public static final double COLECTOR_SPEED_THRESHHOLD = 5;
    }
    public class AlgaeCollectorConstants {
        public static final double PIVOT_TOLERANCE = 5; // temp
        public static final double PIVOT_GEAR_RATIO = 1; // temp
        public static final double PIVOT_MOI = 0.01096; // temp
        public static final double PIVOT_MIN_ANGLE = 0; // temp
        public static final double PIVOT_MAX_ANGLE = 90; // temp
        public static final double PIVOT_LENGTH = 0.5; // temp
        public static final double PIVOT_START_ANGLE = 0; // temp
        public static final double ALGAE_ROLLER_SPEED = 1;

        public static final double ROLLER_KV = 0.24; // temp
        public static final double ROLLER_KA = 0.8; // temp

        public static final boolean PIVOT_INVERTED = false; // temp
        public static final double PIVOT_STATOR_CURRENT_LIMIT = 100d; // temp
        public static final boolean PIVOT_BRAKE_MODE = false; // temp

        public static final boolean ROLLER_INVERTED = false; // temp
        public static final double ROLLER_STATOR_CURRENT_LIMIT = 100d; // temp
        public static final boolean ROLLER_BRAKE_MODE = false; // temp

        public static final double PIVOT_KP = 3; // temp
        public static final double PIVOT_KI = 0; // temp
        public static final double PIVOT_KD = 0; // temp

        public static final double DEPLOY_ANGLE = 90;
        public static final double STOW_ANGLE = 0;

        public static final double COLLECTED_CURRENT = 80d; // temp

        public enum AlgaePivotStates {
            DEPLOYED, STOWED
        }

    }

    public class ClimberConstants {
        public static final double GEAR_RATIO = 1.0; // temp
        public static final double DRUM_RADIUS = 0.015; // temp
        public static final double CARRIAGE_MASS = 18.75; // temp
        public static final double MIN_EXTENSION = -0.5; // temp
        public static final double MAX_EXTENSION = 0; // temp
        public static final double TOLERANCE = 1; // temp
        public static final double ROTOR_TO_MECHANISM_RATIO = 1d;

        public static final double KP = 0.05d; // temp
        public static final double KI = 0.0; // temp
        public static final double KD = 0; // temp

        public static final boolean INVERTED = false; // temp
        public static final double STATOR_CURRENT_LIMIT = 100d; // temp
        public static final boolean BREAK_MODE = true;

    }
}
