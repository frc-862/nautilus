package frc.robot.commands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.PoseConstants.LightningTagID;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.subsystems.Swerve;
import frc.robot.util.GeomUtil;
import frc.robot.util.NetworkNumber;
import frc.robot.util.NetworkPose;
import frc.robot.util.TunableNumber;
import frc.thunder.util.Tuple;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveToPose extends Command {
    private static final TunableNumber drivekP = new TunableNumber("DriveToPose/DrivekP");
    private static final TunableNumber drivekD = new TunableNumber("DriveToPose/DrivekD");
    private static final TunableNumber thetakP = new TunableNumber("DriveToPose/ThetakP");
    private static final TunableNumber thetakD = new TunableNumber("DriveToPose/ThetakD");
    private static final TunableNumber driveMaxVelocity = new TunableNumber("DriveToPose/DriveMaxVelocity");
    private static final TunableNumber driveMaxVelocitySlow = new TunableNumber("DriveToPose/DriveMaxVelocitySlow");
    private static final TunableNumber driveMaxAcceleration = new TunableNumber("DriveToPose/DriveMaxAcceleration");
    private static final TunableNumber thetaMaxVelocity = new TunableNumber("DriveToPose/ThetaMaxVelocity");
    private static final TunableNumber thetaMaxAcceleration = new TunableNumber("DriveToPose/ThetaMaxAcceleration");
    private static final TunableNumber driveTolerance = new TunableNumber("DriveToPose/DriveTolerance");
    private static final TunableNumber thetaTolerance = new TunableNumber("DriveToPose/ThetaTolerance");
    private static final TunableNumber ffMinRadius = new TunableNumber("DriveToPose/FFMinRadius");
    private static final TunableNumber ffMaxRadius = new TunableNumber("DriveToPose/FFMaxRadius");
    private static final NetworkPose targetPose = new NetworkPose("DriveToPose/TargetPose", new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
    private static final NetworkPose currentPose = new NetworkPose("DriveToPose/CurrentPose", new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
    private static final NetworkNumber distance = new NetworkNumber("DriveToPose/Distance", 0.0);

    static {
        drivekP.initDefault(1.8);
        drivekD.initDefault(0.0);
        thetakP.initDefault(0.0);
        thetakD.initDefault(0.0);
        driveMaxVelocity.initDefault(DrivetrainConstants.MAX_SPEED);
        driveMaxAcceleration.initDefault(3.0);
        thetaMaxVelocity.initDefault(Units.degreesToRadians(360.0));
        thetaMaxAcceleration.initDefault(8.0);
        driveTolerance.initDefault(0.01);
        thetaTolerance.initDefault(Units.degreesToRadians(1.0));
        ffMinRadius.initDefault(0.05);
        ffMaxRadius.initDefault(0.1);
    }

    private final Swerve drive;
    private final Supplier<Pose2d> target;

    private final ProfiledPIDController driveController = new ProfiledPIDController(
            0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);

    private Translation2d lastSetpointTranslation = Translation2d.kZero;
    private double driveErrorAbs = 0.0;
    private double thetaErrorAbs = 0.0;
    private boolean running = false;
    private Supplier<Pose2d> robot;

    private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
    private DoubleSupplier omegaFF = () -> 0.0;

    public DriveToPose(Swerve drive, Camera camera, LightningTagID tagID) {
        this(drive, () -> PoseConstants.poseHashMap.get(new Tuple<Camera, Integer>(camera, DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red ? tagID.redID : tagID.blueID)));
    }

    public DriveToPose(Swerve drive, Supplier<Pose2d> target) {
        this.drive = drive;
        robot = drive::getPose;
        this.target = target;

        // Enable continuous input for theta controller
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    public DriveToPose(Swerve drive, Supplier<Pose2d> target, Supplier<Pose2d> robot) {
        this(drive, target);
        this.robot = robot;
    }

    public DriveToPose(
            Swerve drive,
            Supplier<Pose2d> target,
            Supplier<Pose2d> robot,
            Supplier<Translation2d> linearFF,
            DoubleSupplier omegaFF) {
        this(drive, target, robot);
        this.linearFF = linearFF;
        this.omegaFF = omegaFF;
    }

    @Override
    public void initialize() {
        Pose2d currentPose = robot.get();
        ChassisSpeeds fieldVelocity = drive.getFieldVelocity();
        Translation2d linearFieldVelocity = new Translation2d(fieldVelocity.vxMetersPerSecond,
                fieldVelocity.vyMetersPerSecond);
        driveController.reset(
                currentPose.getTranslation().getDistance(target.get().getTranslation()),
                Math.min(
                        0.0,
                        -linearFieldVelocity
                                .rotateBy(
                                        target
                                                .get()
                                                .getTranslation()
                                                .minus(currentPose.getTranslation())
                                                .getAngle()
                                                .unaryMinus())
                                .getX()));
        thetaController.reset(
                currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
        lastSetpointTranslation = currentPose.getTranslation();
    }

    @Override
    public void execute() {
        running = true;

        // Update from tunable numbers
        if (driveMaxVelocity.hasChanged(hashCode())
                || driveMaxVelocitySlow.hasChanged(hashCode())
                || driveMaxAcceleration.hasChanged(hashCode())
                || driveTolerance.hasChanged(hashCode())
                || thetaMaxVelocity.hasChanged(hashCode())
                || thetaMaxAcceleration.hasChanged(hashCode())
                || thetaTolerance.hasChanged(hashCode())
                || drivekP.hasChanged(hashCode())
                || drivekD.hasChanged(hashCode())
                || thetakP.hasChanged(hashCode())
                || thetakD.hasChanged(hashCode())) {
            driveController.setP(drivekP.get());
            driveController.setD(drivekD.get());
            driveController.setConstraints(
                    new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
            driveController.setTolerance(driveTolerance.get());
            thetaController.setP(thetakP.get());
            thetaController.setD(thetakD.get());
            thetaController.setConstraints(
                    new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
            thetaController.setTolerance(thetaTolerance.get());
        }

        // Get current pose and target pose
        Pose2d currentPose = robot.get();
        Pose2d targetPose = target.get();

        // Calculate drive speed
        double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double ffScaler = 1;//MathUtil.clamp(
                // (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
                // 0.0,
                // 1.0);

        driveErrorAbs = currentDistance;
        driveController.reset(
                lastSetpointTranslation.getDistance(targetPose.getTranslation()),
                driveController.getSetpoint().velocity);

        double driveVelocityScalar = driveController.getSetpoint().velocity * ffScaler
                + driveController.calculate(driveErrorAbs, 0.0);

        if (currentDistance < driveController.getPositionTolerance()) {
            driveVelocityScalar = 0.0;
        }

        lastSetpointTranslation = new Pose2d(
                targetPose.getTranslation(),
                new Rotation2d(
                        Math.atan2(
                                currentPose.getTranslation().getY() - targetPose.getTranslation().getY(),
                                currentPose.getTranslation().getX() - targetPose.getTranslation().getX())))
                .transformBy(GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0))
                .getTranslation();

        // Calculate theta speed
        double thetaVelocity = thetaController.getSetpoint().velocity * ffScaler
                + thetaController.calculate(
                        currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        thetaErrorAbs = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());

        if (thetaErrorAbs < thetaController.getPositionTolerance()) {
            thetaVelocity = 0.0;
        }

        // Why do I need to rotate the drive velocity by -45 degrees? Field centric should just work?
        final var adjust = new Rotation2d(-45 * 0.0174533);
        Translation2d driveVelocity = new Pose2d(
                Translation2d.kZero,
                new Rotation2d(
                        Math.atan2(
                                currentPose.getY() - targetPose.getY(),
                                currentPose.getX() - targetPose.getX())).rotateBy(adjust))
                .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
                .getTranslation();

        // Scale feedback velocities by input ff
        final double linearS = linearFF.get().getNorm() * 3.0;
        final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
        driveVelocity = driveVelocity.interpolate(linearFF.get().times(DrivetrainConstants.MAX_SPEED), linearS);
        thetaVelocity = MathUtil.interpolate(
                thetaVelocity, omegaFF.getAsDouble() * DrivetrainConstants.MAX_ANGULAR_RATE, thetaS);

        // Command speeds
        drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));

        // Log data
        DriveToPose.currentPose.set(currentPose);
        DriveToPose.targetPose.set(targetPose);
        DriveToPose.distance.set(currentDistance);

        // Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
        // Logger.recordOutput("DriveToPose/DistanceSetpoint",
        // driveController.getSetpoint().position);
        // Logger.recordOutput("DriveToPose/ThetaMeasured",
        // currentPose.getRotation().getRadians());
        // Logger.recordOutput("DriveToPose/ThetaSetpoint",
        // thetaController.getSetpoint().position);
        // Logger.recordOutput(
        // "DriveToPose/Setpoint",
        // new Pose2d[] {
        // new Pose2d(
        // lastSetpointTranslation,
        // Rotation2d.fromRadians(thetaController.getSetpoint().position))
        // });
        // Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {targetPose});
    }

    /**
     * Called when the command ends or is interrupted.
     * * Stops the drive and clears logs.
     * 
     * @param interrupted true if the command was interrupted, false if it completed
     *                    normally
     */
    @Override
    public void end(boolean interrupted) {
        drive.stop();
        running = false;
        // Clear logs
        // Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
        // Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
    }

    /**
     * Checks if the robot is stopped at the final pose.
     * 
     * @return true if the robot is at the goal pose, false otherwise
     *         * Note: This method checks if the command is running and if both the
     *         drive and theta controllers are at their goals.
     *         It does not check if the robot is within tolerance, which can be done
     *         using the withinTolerance method.
     */
    public boolean atGoal() {
        return running && driveController.atGoal() && thetaController.atGoal();
    }

    /**
     * Checks if the robot pose is within the allowed drive and theta tolerances.
     * 
     * @param driveTolerance the allowed distance tolerance for the drive controller
     * @param thetaTolerance the allowed angle tolerance for the theta controller
     * @return true if the robot is within the specified tolerances, false otherwise
     *         * Note: This method checks if the absolute errors for both the drive
     *         and theta controllers are within the specified tolerances.
     */
    public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
        return running
                && Math.abs(driveErrorAbs) < driveTolerance
                && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
    }
}
