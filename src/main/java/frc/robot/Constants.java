package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.nio.file.Paths;
import java.util.HashMap;
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
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.Constants.VisionConstants.ReefPose;
import frc.robot.subsystems.FishingRod;
import frc.robot.subsystems.Swerve;
import frc.thunder.hardware.ThunderBird;
import frc.thunder.math.InterpolationMap;
import frc.thunder.util.Tuple;

public class Constants {

    public static final String TRITON_IDENTIFIER = "/home/lvuser/triton"; // Differentiate between Triton and Nautlius
    public static final boolean IS_TRITON = Paths.get(TRITON_IDENTIFIER).toFile().exists();

    public enum RobotIdentifiers {
        NAUTILUS, TRITON, SIM
    };

    // note from kyle: wtf is this?? :sob:
    // yeah its a double inline ternary operator fight me
    // if(isTriton); else if(isSim); else, its nautilus
    public static final RobotIdentifiers ROBOT_IDENTIFIER = Paths.get(TRITON_IDENTIFIER).toFile().exists()
        ? RobotIdentifiers.TRITON
        : Robot.isSimulation() ? RobotIdentifiers.SIM : RobotIdentifiers.NAUTILUS;

    public static class EncoderConstants {
        // Nautilus values
        private static final Angle nautilusKFrontLeftEncoderOffset = Rotations.of(0.252685546875);
        private static final Angle nautilusKFrontRightEncoderOffset = Rotations.of(0.408203125);
        private static final Angle nautilusKBackLeftEncoderOffset = Rotations.of(-0.031494140625);
        private static final Angle nautilusKBackRightEncoderOffset = Rotations.of(-0.427978515625);

        // Triton values
        private static final Angle tritonKFrontLeftEncoderOffset = Rotations.of(0.0073);
        private static final Angle tritonKFrontRightEncoderOffset = Rotations.of(0.033447);
        private static final Angle tritonKBackLeftEncoderOffset = Rotations.of(0.1350);
        private static final Angle tritonKBackRightEncoderOffset = Rotations.of(0.129395);

        public static final double tritonWristOffset = -0.227;
        public static final double nautilusWristOffset = 0.75586;

        // Generic values
        public static final double frontLeftOffset = IS_TRITON ? tritonKFrontLeftEncoderOffset.in(Rotations)
            : nautilusKFrontLeftEncoderOffset.in(Rotations);
        public static final double frontRightOffset = IS_TRITON ? tritonKFrontRightEncoderOffset.in(Rotations)
            : nautilusKFrontRightEncoderOffset.in(Rotations);
        public static final double backLeftOffset = IS_TRITON ? tritonKBackLeftEncoderOffset.in(Rotations)
            : nautilusKBackLeftEncoderOffset.in(Rotations);
        public static final double backRightOffset = IS_TRITON ? tritonKBackRightEncoderOffset.in(Rotations)
            : nautilusKBackRightEncoderOffset.in(Rotations);

        public static final double wristOffset = IS_TRITON ? tritonWristOffset : nautilusWristOffset;
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
        public static final int CORAL_COLLECTOR_BEAM_BREAK_DIO = 1; // temp

        public static final int ALGAE_COLLECTOR_ROLLER = 13; // temp
        public static final int ALGAE_COLLECTOR_PIVOT = 14; // temp

        public static final int CLIMBER = 15;
        public static final int CLIMBER_LIMIT_SWITCH_DIO = 0;

        public static final int PIGEON = 23;

        public static final String CANIVORE_CAN_NAME = "Canivore";

        public static final int PDH = 2; // RIO LOOP
        public static final ModuleType PDH_MODULE_TYPE = ModuleType.kRev;

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

        //keep in mind that this is when the other mechanism will START moving, not necessarily the angle it will intersect the reef at
        public static final double L2_SAFEZONE_ELE = 8d; //elevator height to move wrist
        public static final double L3_SAFEZONE_ELE = 18d; // TUNE THIS!!!!
        public static final double L4_SAFEZONE_ELE = 31d; //elevator height to move wrist
        public static final double LOW_SAFEZONE_ELE = 10d; // 17 is setpoint
        public static final double HIGH_SAFEZONE_ELE = 20d; // 28 is setpoint

        public static final double STOW_SAFEZONE_ANGLE = -30d; //wrist angle to move elevator

        public enum RodStates {
            STOW(false), INVERSE_STOW(false), L1(true), L2(true), L3(true), L4(true), SOURCE(false), LOW(true), HIGH(true),
            BARGE(true), PROCESSOR(false), DEFAULT(false), UNKNOWN(false);

            private boolean scoring;

            RodStates(boolean scoring) {
                this.scoring = scoring;
            }

            public boolean isScoring() {
                return scoring;
            }
        }

        public enum RodTransitionStates {
            DEFAULT, // default travel state
            WRIST_DOWN_THEN_ELE, // any state to L4
            WRIST_UP_THEN_ELE, // L4 to any state
            WITH_WRIST_SLOW,
            TRITON, // specific state to deal with triton's loose belt,
            TRANSITIONING,

            CORAL_SAFE_ZONE, // transition to any coral score setpoint, moves wrist earlier than onTarget()
            STOW_SAFE_ZONE // transition to stow, moves elevator earlier than onTarget()
        }

        public static final HashMap<RodStates, Double> WRIST_MAP = new HashMap<RodStates, Double>() {
            {
                put(RodStates.STOW, IS_TRITON ? 80d : 75d); // Lower angle is safer for nautilus
                put(RodStates.INVERSE_STOW, -70d); // Lower angle is safer for nautilus
                put(RodStates.L1, 6d);
                put(RodStates.L2, -30d); // -30
                put(RodStates.L3, -36d); // -36
                put(RodStates.L4, -42d);
                put(RodStates.LOW, -29d);
                put(RodStates.HIGH, -29d);
                put(RodStates.SOURCE, 42d);
                put(RodStates.PROCESSOR, -25.5d);
                put(RodStates.BARGE, 57d);
            }
        };

        public static final HashMap<RodStates, Double> ELEVATOR_MAP = new HashMap<RodStates, Double>() {
            {
                put(RodStates.STOW, 2d);
                put(RodStates.INVERSE_STOW, 4d);
                put(RodStates.L1, 2d);
                put(RodStates.L2, 14d);
                put(RodStates.L3, 27d); // 26.5
                put(RodStates.L4, 46.25d);
                put(RodStates.LOW, 17d); // 15
                put(RodStates.HIGH, 28d);
                put(RodStates.SOURCE, 9.6d); // 9.1
                put(RodStates.BARGE, 47d);
                put(RodStates.PROCESSOR, 1.75d);
            }
        };
    }

    public static class ElevatorConstants {
        public static final double OVERHEAT_TEMP = 95;
        public static final double OVERHEAT_TEMP_DIFFERENCE = 20;

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

        public static final double VELOC = 72d; // 80
        public static final double ACCEL = 250d; // 200
        public static final double JERK = 1600d; // temp

        public static final double POSITION_TOLERANCE = 0.1; // temp
        public static final double CANRANGE_TOLERANCE = 0.25; // temp

        public static final double BOTTOM_CURRENT = 75d; // our current to check when our elevator bottoms out
        public static final double BOTTOM_RAW_POWER = -0.25d; // raw dutyCycle out to go to the bottom

        // kind of guessing the numbers here (didn't do a proper test)
        public static final Distance MIN_EXTENSION = Inches.of(0);
        public static final Distance MAX_EXTENSION = Inches.of(49.7);

        // SIM
        public static final Mass CARRIAGE_WEIGHT = Pounds.of(30); // temp
        public static final double CUSHION_METERS = 0.05; // stages don't line up perfectly
        public static final double STAGE_LEN_METERS = MAX_EXTENSION.in(Meters) / 3;

        public static final double SLOW_MODE_HEIGHT_LIMIT = 29d;

        public static final double TRITON_INTERPOLATION_SLOPE = 0.692679;
        public static final double TRITON_INTERPOLATION_INTERCEPT = -2.53443;

        public static final double NATUILUS_INTERPOLATION_SLOPE = 20.49174;
        public static final double NAUTILUS_INTERPOLATION_INTERCEPT = -0.689164;


        public static final InterpolationMap CANRANGE_MAP = new InterpolationMap() {
            {
                put(-10000d, 0d);
                put(0.124, 0d);
                put(0.20684, 2.927);
                put(0.22491, 3.5959);
                put(0.24941, 4.48193);
                put(0.26504, 4.85571);
                // added 3/11/25
                put(0.295, 5.207763);
                put(0.312, 5.698242);
                put(0.341, 6.163085);
                put(0.365, 6.664795);
                put(0.384, 7.416503);
                put(0.414, 8.211181);
                put(0.445, 8.841309);
                put(0.432, 9.000244);
                put(0.453, 9.314941);
                put(0.464, 9.496093);
            }
        };
    }

    public static class WristConstants {
        public static final boolean BRAKE_MODE = true;
        public static final double STATOR_CURRENT_LIMIT = 100d;
        public static final boolean INVERTED = false;

        public static final double ROTOR_TO_ENCODER_RATIO = 74;
        public static final double ENCODER_TO_MECHANISM_RATIO = 1d;

        public static final double MOTORS_KP = 60;
        public static final double MOTORS_KI = 0;
        public static final double MOTORS_KD = 0;
        public static final double MOTORS_KF = 0;
        public static final double MOTORS_KS = 0;
        public static final double MOTORS_KV = 0;
        public static final double MOTORS_KA = 0;
        public static final double MOTORS_KG = 0.1;

        public static final double MOTORS_KP_SLOW = 33;

        public static final Angle MIN_ANGLE = Degrees.of(-85);
        public static final Angle MAX_ANGLE = Degrees.of(85);

        public static final double TOLERANCE = 5d;


        // sim stuff
        public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.086); // 5lb, 2.5in rad, 9in height
        public static final Distance LENGTH = Meters.of(0.18); // TODO: ask mr hurley abt this because i have no clue
    }

    public static class CoralCollectorConstants {
        // public static final boolean INVERTED = false; // OLD - check below for proper invert
        public static final boolean BRAKE_MODE = false;
        public static final double STATOR_CURRENT_LIMIT = 100d; // temp
        public static final double CORAL_ROLLER_SPEED = 1;
        public static final double DEBOUNCE_TIME = 0.1; // unused

        public static final double GEAR_RATIO = 1d / 2d; // output shaft gear reduction / Motor gear reduction
        public static final double ROTOR_TO_ENCODER_RATIO = GEAR_RATIO * 360; // temp
        public static final double ENCODER_TO_MECHANISM_RATIO = 1d;

        public static final double KV = 0.24; // temp
        public static final double KA = 0.8; // temp

        public static final double BEAMBREAK_DEBOUNCE = 0.1; // unused

        // public static final double COLLECTED_CURRENT = 35d;
        public static final double COLLECTOR_DEADBAND = 0.1;

        // 2.5 constants
        public static final boolean INVERTED = true;

        public static final double LOW_SPIT_POWER_MULT = 0.75;
        public static final double CORAL_COLLECTED_CURRENT = 40d; //40
        public static final double ALGAE_COLLECTED_CURRENT = 70d; //40
        public static final double CORAL_HOLD_POWER = 0.07d;
        public static final double ALGAE_HOLD_POWER = 0.2d;
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
            private static final SwerveRequest.FieldCentric AUTO_ALIGN = new SwerveRequest.FieldCentric();
            private static final SwerveRequest.FieldCentric SLOW = new SwerveRequest.FieldCentric();
            private static final SwerveRequest.RobotCentric ROBO_CENTRIC = new SwerveRequest.RobotCentric();
            private static final SwerveRequest.SwerveDriveBrake BRAKE = new SwerveRequest.SwerveDriveBrake();

            public static Supplier<SwerveRequest> getDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
                return () -> DRIVE
                        .withVelocityX(x.getAsDouble() * DrivetrainConstants.MAX_SPEED) // Drive forward with negative Y
                                                                                        // (forward)
                        .withVelocityY(y.getAsDouble() * DrivetrainConstants.MAX_SPEED) // Drive left with negative X
                                                                                        // (left)
                        .withRotationalRate(rot.getAsDouble() * DrivetrainConstants.MAX_ANGULAR_RATE)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective); // Drive counterclockwise with negative
                                                                                 // X (left)

            }

            public static SwerveRequest getAutoAlign(double x, double y, double rot) {
                return AUTO_ALIGN
                        .withVelocityX(x * DrivetrainConstants.MAX_SPEED) // Drive forward with negative Y (forward)
                        .withVelocityY(y * DrivetrainConstants.MAX_SPEED) // Drive left with negative X (left)
                        .withRotationalRate(rot * DrivetrainConstants.MAX_ANGULAR_RATE)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance); // Drive counterclockwise with negative
                        // X
                                                                                 // (left)
            }

            // public static Supplier<SwerveRequest> getSlow(DoubleSupplier x,
            // DoubleSupplier y, DoubleSupplier rot) {
            // return () -> SLOW
            // .withVelocityX(y.getAsDouble() * DrivetrainConstants.MAX_SPEED *
            // SLOW_MODE_MULT) // Drive
            // // forward with
            // // negative Y
            // // (forward)
            // .withVelocityY(x.getAsDouble() * DrivetrainConstants.MAX_SPEED *
            // SLOW_MODE_MULT) // Drive left
            // // with
            // // negative X
            // // (left)
            // .withRotationalRate(rot.getAsDouble() * DrivetrainConstants.MAX_ANGULAR_RATE
            // * SLOW_MODE_MULT)
            // .withDeadband(DrivetrainConstants.MAX_SPEED * 0.1)
            // .withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE * 0.1) // Add a
            // 10% deadband
            // .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Drive
            // counterclockwise with negative
            // // X (left)

            // }

            public static Supplier<SwerveRequest> getRobotCentric(DoubleSupplier x, DoubleSupplier y,
                    DoubleSupplier rot) {
                return () -> ROBO_CENTRIC
                        .withVelocityX(y.getAsDouble() * DrivetrainConstants.MAX_SPEED) // Drive forward with negative Y
                                                                                        // (forward)
                        .withVelocityY(x.getAsDouble() * DrivetrainConstants.MAX_SPEED) // Drive left with negative X
                                                                                        // (left)
                        .withRotationalRate(rot.getAsDouble() * DrivetrainConstants.MAX_ANGULAR_RATE)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Drive counterclockwise with negative
                                                                                 // X (left)
            }

            public static SwerveRequest getRobotCentric(double x, double y, double rot) {
                return ROBO_CENTRIC
                        .withVelocityX(y * DrivetrainConstants.MAX_SPEED) // Drive forward with negative Y (forward)
                        .withVelocityY(x * DrivetrainConstants.MAX_SPEED) // Drive left with negative X (left)
                        .withRotationalRate(rot * DrivetrainConstants.MAX_ANGULAR_RATE)
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
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(50, 0, 0);
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
        public static final String leftCamName = "leftCam";
        public static final String rightCamName = "rightCam";
        public static final String middleCamName = "backCam";
        public static final TargetModel targetModel = TargetModel.kAprilTag36h11;
        public static final Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
        public static final VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);
        public static final AprilTagFieldLayout tagLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.k2025ReefscapeWelded);
        public static final SimCameraProperties cameraProp = new SimCameraProperties();
        public static final Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
        public static final Rotation3d robotToCameraRot = new Rotation3d(0, 0, Math.PI);
        public static final Transform3d robotLeftToCamera = new Transform3d(
                new Translation3d(-5.772, 11.281, 12).times(0.0254), robotToCameraRot);
        public static final Transform3d robotRightToCamera = new Transform3d(
                new Translation3d(-5.772, -11.281, 12).times(0.0254), robotToCameraRot);
            public static final Transform3d robotMiddleToCamera = new Transform3d(
                        new Translation3d(5, -6.35, 12).times(0.0254), robotToCameraRot);

        // MIDDLE is only used for algae on the reef!!
        public enum ReefPose {
            LEFT, RIGHT, MIDDLE
        }

        public static final double VISION_X_STDEV = 1;
        public static final double VISION_Y_STDEV = 1;
        public static final double VISION_THETA_STDEV = 1;
    }

    public static class PoseConstants {
        //temporary variable until red poses are properly tuned
        static final double blueRedTransform = 8.575;

        public static final Translation2d FIELD_LIMIT = new Translation2d(Units.feetToMeters(54.0),
                Units.feetToMeters(26.0)); // This is WRONG!!!! This should be 57 feet! but im not going to change it now :)

        public static final Pose2d RED_BARGE = new Pose2d(9.982, 1.621, new Rotation2d(Degrees.of(0)));
        public static final Pose2d BLUE_BARGE = new Pose2d(7.625, 6.201, new Rotation2d(Degrees.of(-180)));

        public static HashMap<Tuple<VisionConstants.ReefPose, Integer>, Pose2d> poseHashMap = new HashMap<Tuple<VisionConstants.ReefPose, Integer>, Pose2d>() {
            {
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 6), new Pose2d(13.5635436668043, 2.82659631425915, new Rotation2d(Degrees.of(-60))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 6), new Pose2d(13.8453483331957, 2.98929631425915, new Rotation2d(Degrees.of(-60))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 7), new Pose2d(14.350498, 3.8632, new Rotation2d(Degrees.of(0))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 7), new Pose2d(14.350498, 4.1886, new Rotation2d(Degrees.of(0))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 8), new Pose2d(13.8453483331957, 5.06250368574084, new Rotation2d(Degrees.of(60))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 8), new Pose2d(13.5635436668043, 5.22520368574084, new Rotation2d(Degrees.of(60))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 9), new Pose2d(12.5542603331957, 5.22520368574084, new Rotation2d(Degrees.of(120))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 9), new Pose2d(12.2724556668043, 5.06250368574084, new Rotation2d(Degrees.of(120))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 10), new Pose2d(11.7673059999999, 4.1886, new Rotation2d(Degrees.of(180))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 10), new Pose2d(11.7673059999999, 3.8632, new Rotation2d(Degrees.of(180))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 11), new Pose2d(12.2724556668043, 2.98929631425915, new Rotation2d(Degrees.of(-120))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 11), new Pose2d(12.5542603331957, 2.82659631425915, new Rotation2d(Degrees.of(-120))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 17), new Pose2d(3.98480833319572, 2.82659631425915, new Rotation2d(Degrees.of(-120))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 17), new Pose2d(3.70300366680426, 2.98929631425915, new Rotation2d(Degrees.of(-120))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 18), new Pose2d(3.1976, 3.8632, new Rotation2d(Degrees.of(180))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 18), new Pose2d(3.1976, 4.1886, new Rotation2d(Degrees.of(180))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 19), new Pose2d(3.70300366680426, 5.06250368574084, new Rotation2d(Degrees.of(120))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 19), new Pose2d(3.98480833319572, 5.22520368574084, new Rotation2d(Degrees.of(120))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 20), new Pose2d(4.99383766680426, 5.22520368574084, new Rotation2d(Degrees.of(60))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 20), new Pose2d(5.27564233319572, 5.06250368574084, new Rotation2d(Degrees.of(60))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 21), new Pose2d(5.781046, 4.1886, new Rotation2d(Degrees.of(0))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 21), new Pose2d(5.781046, 3.8632, new Rotation2d(Degrees.of(0))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 22), new Pose2d(5.27564233319572, 2.98929631425915, new Rotation2d(Degrees.of(-60))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 22), new Pose2d(4.99383766680426, 2.82659631425915, new Rotation2d(Degrees.of(-60))));

                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.MIDDLE, 6), new Pose2d(13.689446, 2.93392707637268, new Rotation2d(Degrees.of(-60))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.MIDDLE, 7), new Pose2d(14.320498, 4.0259, new Rotation2d(Degrees.of(0))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.MIDDLE, 8), new Pose2d(13.689446, 5.11787292362731, new Rotation2d(Degrees.of(60))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.MIDDLE, 9), new Pose2d(12.428358, 5.11787292362731, new Rotation2d(Degrees.of(120))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.MIDDLE, 10), new Pose2d(11.7973059999999, 4.0259, new Rotation2d(Degrees.of(180))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.MIDDLE, 11), new Pose2d(12.428358, 2.93392707637268, new Rotation2d(Degrees.of(-120))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.MIDDLE, 17), new Pose2d(3.85890599999999, 2.93392707637268, new Rotation2d(Degrees.of(-120))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.MIDDLE, 18), new Pose2d(3.2276, 4.0259, new Rotation2d(Degrees.of(180))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.MIDDLE, 19), new Pose2d(3.85890599999999, 5.11787292362731, new Rotation2d(Degrees.of(120))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.MIDDLE, 20), new Pose2d(5.11973999999999, 5.11787292362731, new Rotation2d(Degrees.of(60))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.MIDDLE, 21), new Pose2d(5.751046, 4.0259, new Rotation2d(Degrees.of(0))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.MIDDLE, 22), new Pose2d(5.11973999999999, 2.93392707637268, new Rotation2d(Degrees.of(-60))));

                
                //red right source
                put(new Tuple<>(VisionConstants.ReefPose.RIGHT, 2),
                        new Pose2d(16.664, 7.411, new Rotation2d(Degrees.of(-126))));
                //red left source
                put(new Tuple<>(VisionConstants.ReefPose.RIGHT, 1),
                        new Pose2d(16.653, 0.626, new Rotation2d(Degrees.of(126))));

                //blue left source
                put(new Tuple<>(VisionConstants.ReefPose.RIGHT, 13),
                        new Pose2d(1.026, 7.372, new Rotation2d(Degrees.of(-54))));
                //blue right source
                put(new Tuple<>(VisionConstants.ReefPose.RIGHT, 12),
                        new Pose2d(1.004, 0.665, new Rotation2d(Degrees.of(54))));

                // red barge front
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 5),
                    new Pose2d(9.982, 1.621, new Rotation2d(Degrees.of(0))));
                // blue barge front
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 14),
                    new Pose2d(7.625, 5.5, new Rotation2d(Degrees.of(-180))));

                // red barge back
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 15),
                    new Pose2d(7.564, 2.005, new Rotation2d(Degrees.of(-180))));
                // blue barge back
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 4),
                    new Pose2d(10.034, 6.009, new Rotation2d(Degrees.of(0))));
            }

        };

        public static HashMap<Tuple<VisionConstants.ReefPose, Integer>, Pose2d> l1PoseHashMap = new HashMap<Tuple<VisionConstants.ReefPose, Integer>, Pose2d>() {
        {
                // mathematically deduced ones
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 6), new Pose2d(13.7168434596216, 2.68647328430983, new Rotation2d(Degrees.of(-90))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 6), new Pose2d(13.8900485403784, 2.78647328430983, new Rotation2d(Degrees.of(-30))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 7), new Pose2d(14.548498, 3.9259, new Rotation2d(Degrees.of(-30))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 7), new Pose2d(14.548498, 4.1259, new Rotation2d(Degrees.of(30))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 8), new Pose2d(13.8900485403784, 5.26532671569016, new Rotation2d(Degrees.of(30))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 8), new Pose2d(13.7168434596216, 5.36532671569016, new Rotation2d(Degrees.of(90))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 9), new Pose2d(12.4009605403784, 5.36532671569016, new Rotation2d(Degrees.of(90))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 9), new Pose2d(12.2277554596216, 5.26532671569016, new Rotation2d(Degrees.of(150))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 10), new Pose2d(11.5693059999999, 4.1259, new Rotation2d(Degrees.of(150))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 10), new Pose2d(11.5693059999999, 3.9259, new Rotation2d(Degrees.of(210))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 11), new Pose2d(12.2277554596216, 2.78647328430983, new Rotation2d(Degrees.of(-150))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 11), new Pose2d(12.4009605403784, 2.68647328430983, new Rotation2d(Degrees.of(-90))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 17), new Pose2d(3.65830345962155, 2.78647328430983, new Rotation2d(Degrees.of(-150))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 17), new Pose2d(3.83150854037843, 2.68647328430983, new Rotation2d(Degrees.of(-90))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 18), new Pose2d(2.9996, 4.1259, new Rotation2d(Degrees.of(150))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 18), new Pose2d(2.9996, 3.9259, new Rotation2d(Degrees.of(210))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 19), new Pose2d(3.83150854037843, 5.36532671569016, new Rotation2d(Degrees.of(90))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 19), new Pose2d(3.65830345962155, 5.26532671569016, new Rotation2d(Degrees.of(150))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 20), new Pose2d(5.32034254037843, 5.26532671569016, new Rotation2d(Degrees.of(30))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 20), new Pose2d(5.14713745962155, 5.36532671569016, new Rotation2d(Degrees.of(90))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 21), new Pose2d(5.979046, 3.9259, new Rotation2d(Degrees.of(-30))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 21), new Pose2d(5.979046, 4.1259, new Rotation2d(Degrees.of(30))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.LEFT, 22), new Pose2d(5.14713745962155, 2.68647328430983, new Rotation2d(Degrees.of(-90))));
                put(new Tuple<ReefPose, Integer>(VisionConstants.ReefPose.RIGHT, 22), new Pose2d(5.32034254037843, 2.78647328430983, new Rotation2d(Degrees.of(-30))));
            }
        };

        public enum LightningTagID {
            // reef poses
            One(7, 18),
            Two(8, 17),
            Three(9, 22),
            Four(10, 21),
            Five(11, 20),
            Six(6, 19),

            RightSource(2, 12),
            LeftSource(1, 13),

            Barge(5, 14),
            BargeBack(15, 4);

            public final int redID;
            public final int blueID;
            LightningTagID(int redID, int blueID) {
                this.redID = redID;
                this.blueID = blueID;
            }
        }

        public static final Pose2d CENTER_REEF_RED = new Pose2d(13.058, 4.025, new Rotation2d(0));
        public static final Pose2d CENTER_REEF_BLUE = new Pose2d(4.482, 4.025, new Rotation2d(0));

        public enum StowZone {
            REEF(1.75d),
            SAFE(4.2d),
            SOURCE(10d);

            public final double radius; // In Meters
            StowZone(double radius) {
                this.radius = radius;
            }

        }

        public static final PathConstraints PATHFINDING_CONSTRAINTS = new PathConstraints(2.0, 1.0, 3.0, 1.5);

        // POPULATE WITH REAL VALUES
        public static HashMap<Rectangle2d, Integer> aprilTagRegions = new HashMap<Rectangle2d, Integer>() {
            {
                put(new Rectangle2d(new Translation2d(3.321, 3.324), new Translation2d(3.512, 1.885)), 17);
                put(new Rectangle2d(new Translation2d(3.297, 4.642), new Translation2d(2.206, 3.360)), 18);
                put(new Rectangle2d(new Translation2d(4.495, 5.362), new Translation2d(2.997, 5.949)), 19);
                put(new Rectangle2d(new Translation2d(5.658, 4.678), new Translation2d(5.370, 6.429)), 20);
                put(new Rectangle2d(new Translation2d(5.658, 3.372), new Translation2d(6.773, 4.798)), 21);
                put(new Rectangle2d(new Translation2d(4.483, 2.676), new Translation2d(6.365, 2.449)), 22);
            }
        };

        /**
         * Returns the RodState for algae based on the tagID
         * @param tagId to reference
         * @return HIGH/LOW depending on tagId
         */
        public static FishingRodConstants.RodStates getAlgaeScoreState(double tagId) {
            final DriverStation.Alliance blue = DriverStation.Alliance.Blue;
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(blue);

            if (tagId % 2 == 0) {
                // even: algae on odd sides are high for blue reef, low for red
                return alliance == blue ? FishingRodConstants.RodStates.HIGH : FishingRodConstants.RodStates.LOW;
            } else {
                // odd: algae on odd sides are low for blue reef, high for red
                return alliance == blue ? FishingRodConstants.RodStates.LOW : FishingRodConstants.RodStates.HIGH;
            }
        }

        /**
         * Returns the RodState for algae based on the drivetrain position
         * @param drivetrain
         * @param rod
         * @return HIGH/LOW depending on pos
         */
        public static FishingRodConstants.RodStates getAlgaeScoreState(Swerve drivetrain, FishingRod rod) {
            double tagId = getScorePose(drivetrain.getPose());

            if (tagId == 0) {
                return rod.getState();
            }

            return getAlgaeScoreState(tagId);
        }

        public static int getScorePose(Pose2d robotPose){


            final DriverStation.Alliance blue = DriverStation.Alliance.Blue;
            DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(blue);


            Translation2d robotToReef = alliance == blue ? robotPose.getTranslation().minus(new Translation2d(4.5, 4.031))
                : robotPose.getTranslation().minus(new Translation2d(13.055, 4.031));

            double theta = MathUtil.inputModulus(robotToReef.getAngle().getRadians(), 0, Math.PI * 2);
            double r = robotToReef.getDistance(new Translation2d());

            if (r > 3){
                return 0;
            }


            if (theta >= 0 && theta <= Math.PI/6){
                return alliance == blue ? 21 : 7;
            } else if (theta > Math.PI/6 && theta <= 3 * Math.PI/6){
                return alliance == blue ? 20 : 8;
            } else if (theta > 3 * Math.PI/6 && theta <= 5 * Math.PI/6){
                return alliance == blue ? 19 : 9;
            } else if (theta > 5 * Math.PI/6 && theta <= 7 * Math.PI/6){
                return alliance == blue ? 18 : 10;
            } else if (theta > 7 * Math.PI/6 && theta <= 9 * Math.PI/6){
                return alliance == blue ? 17 : 11;
            } else if (theta > 9 * Math.PI/6 && theta <= 11 * Math.PI/6){
                return alliance == blue ? 22 : 6;
            } else if (theta > 11 * Math.PI/6 && theta <= 12 * Math.PI/6){
                return alliance == blue ? 21 : 7;
            } else {
                return 0;
            }
        }
    }

    public class TunerConstants {

        public class NautliusTunerConstants {
            // Both sets of gains need to be tuned to your individual robot.

            // The steer motor uses any SwerveModule.SteerRequestType control request with
            // the
            // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
            private static final Slot0Configs steerGains = new Slot0Configs()
                    .withKP(50).withKI(0).withKD(0.5)
                    .withKS(0.224).withKV(2.6).withKA(0)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
            // When using closed-loop control, the drive motor uses the control
            // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
            private static final Slot0Configs driveGains = new Slot0Configs()
                .withKP(0.1751).withKI(0).withKD(0)
                .withKS(0.057834).withKV(0.12065).withKA(0.020309);
                // .withKP(0.6507).withKI(0).withKD(0)
                // .withKS(0.33986).withKV(0.12318).withKA(0.0059707);

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
            private static final int kBackRightSteerMotorId = RobotMap.BR_TURN;
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
                    .withKP(50).withKI(0).withKD(0.5)
                    .withKS(0.27425).withKV(2.4308).withKA(0.094343)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
            // When using closed-loop control, the drive motor uses the control
            // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
            private static final Slot0Configs driveGains = new Slot0Configs()
                .withKP(0.6507).withKI(0).withKD(0)
                .withKS(0.33986).withKV(0.12318).withKA(0.0059707);

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
        public static final int LENGTH = 200;

        public static final int PULSE_TIME = 1;

        public static final int SWRIL_SEGMENT_SIZE = 5;

        public static final double PDH_LED_POWEROFF_VOLTAGE = 9d;

        public enum LEDStates {
            MIXER(),
            ERROR(),
            CLIMBED(),
            COLLECTED(),
            ALIGNED(),
            ALGAE_MODE(),
            ALIGNING(),
            COLLECTING(),
            SCORING(),
            ROD_MOVING(),
            READY_TO_ALIGN(),
            UPDATING_POSE(),
            POSE_BAD()
        }
    }

    public class AutoAlignConstants {
        
        // auton gains for posebased (if needed)
        public static final double AUTON_DRIVE_P = 1.5d; //0.8
        public static final double AUTON_DRIVE_I = 0;
        public static final double AUTON_DRIVE_D = 0.08; // 0.035
        public static final double AUTON_DRIVE_KS = 0;//0.1;

        public static final double AUTON_ROT_P = 0.03d;
        public static final double AUTON_ROT_I = 0;
        public static final double AUTON_ROT_D = 0;
        public static final double AUTON_ROT_KS = 0;

        // tele gains
        public static final double TELE_DRIVE_P = 1.5d;
        public static final double TELE_DRIVE_I = 0;
        public static final double TELE_DRIVE_D = 0.08;
        public static final double TELE_DRIVE_TOLERANCE = 0.025;
        public static final double TELE_DRIVE_KS = 0;//0.08;

        public static final double TELE_ROT_P = 0.03;
        public static final double TELE_ROT_I = 0;
        public static final double TELE_ROT_D = 0;
        public static final double TELE_ROT_TOLERANCE = 1.5;
        public static final double TELE_ROT_KS = 0; // 0.01 NOT APPLIED

        public static final double DEPLOY_VEL = 0.2; // 0.45
        public static final double BARGE_DEPLY_VEL = 0.25;
    }

    public class SimGamePeicesConstants {
        public static final Pose3d A1B = new Pose3d(1.216, 5.850, 0.5, new Rotation3d(0, 0, 0));
        public static final Pose3d A2B = new Pose3d(1.216, 4.020, 0.5, new Rotation3d(0, 0, 0));
        public static final Pose3d A3B = new Pose3d(1.216, 2.190, 0.5, new Rotation3d(0, 0, 0));
        public static final Pose3d A1R = new Pose3d(16.330, 2.190, 0.5, new Rotation3d(0, 0, 0));
        public static final Pose3d A2R = new Pose3d(16.330, 4.020, 0.5, new Rotation3d(0, 0, 0));
        public static final Pose3d A3R = new Pose3d(16.330, 5.850, 0.5, new Rotation3d(0, 0, 0));

        public static final Pose3d DEFAULT_POSE = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

        public static final double COLLECTION_TOLERANCE = 0.5;
        public static final double COLECTOR_SPEED_THRESHHOLD = 3;

        public static final Translation3d ELEVATOR_OFFSET = new Translation3d(0, -0.15, 0.35); // temp, robot relative
        public static final Translation3d ALGAE_COLLECTOR_OFFSET = new Translation3d(0, 0.15, 0.33); // temp, robot
                                                                                                     // relative
    }

    public class AlgaeCollectorConstants {
        public static final double PIVOT_TOLERANCE = 5; // temp
        public static final double PIVOT_GEAR_RATIO = 1; // temp
        public static final double PIVOT_MOI = 0.01096; // temp
        public static final double PIVOT_MIN_ANGLE = 0; // temp
        public static final double PIVOT_MAX_ANGLE = 90; // temp
        public static final double PIVOT_LENGTH = 0.33; // temp
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

        public static final boolean INVERTED = false; // temp
        public static final double STATOR_CURRENT_LIMIT = 100d; // temp
        public static final boolean BREAK_MODE = true;
    }
}

