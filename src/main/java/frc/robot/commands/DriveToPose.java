package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.PoseConstants.LightningTagID;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.subsystems.Swerve;
import frc.robot.util.GeomUtil;
import frc.robot.util.TunableNumber;
import frc.thunder.util.Tuple;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveToPose extends Command {
  private static final TunableNumber drivekP = new TunableNumber("DriveToPose/DrivekP");
  private static final TunableNumber drivekD = new TunableNumber("DriveToPose/DrivekD");
  private static final TunableNumber thetakP = new TunableNumber("DriveToPose/ThetakP");
  private static final TunableNumber thetakD = new TunableNumber("DriveToPose/ThetakD");
  private static final TunableNumber driveMaxVelocity =
      new TunableNumber("DriveToPose/DriveMaxVelocity");
  private static final TunableNumber driveMaxVelocitySlow =
      new TunableNumber("DriveToPose/DriveMaxVelocitySlow");
  private static final TunableNumber driveMaxAcceleration =
      new TunableNumber("DriveToPose/DriveMaxAcceleration");
  private static final TunableNumber thetaMaxVelocity =
      new TunableNumber("DriveToPose/ThetaMaxVelocity");
  private static final TunableNumber thetaMaxAcceleration =
      new TunableNumber("DriveToPose/ThetaMaxAcceleration");
  private static final TunableNumber driveTolerance =
      new TunableNumber("DriveToPose/DriveTolerance");
  private static final TunableNumber thetaTolerance =
      new TunableNumber("DriveToPose/ThetaTolerance");
  private static final TunableNumber ffMinRadius =
      new TunableNumber("DriveToPose/FFMinRadius");
  private static final TunableNumber ffMaxRadius =
      new TunableNumber("DriveToPose/FFMaxRadius");

  static {
    drivekP.initDefault(6.0);
    drivekD.initDefault(0.0);
    thetakP.initDefault(4.0);
    thetakD.initDefault(0.0);
    driveMaxVelocity.initDefault(3.8);
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

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);

  private Translation2d lastSetpointTranslation = Translation2d.kZero;
  private Rotation2d lastSetpointRotation = Rotation2d.kZero;
  private double lastTime = 0.0;
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  private boolean running = false;
  private Supplier<Pose2d> robot;

  private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
  private DoubleSupplier omegaFF = () -> 0.0;

  public DriveToPose(Swerve drive, Supplier<Pose2d> target) {
    this.drive = drive;
    this.target = target;
    this.robot = () -> drive.getPose(); // Default to drive pose

    // Enable continuous input for theta controller
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  public DriveToPose(Swerve drive, Supplier<Pose2d> target, Supplier<Pose2d> robot) {
    this(drive, target);
    this.robot = robot;
  }

  public DriveToPose(Swerve drive, Pose2d target) {
    this(drive, () -> target);
  }

  public DriveToPose(Swerve drive, Camera camera, LightningTagID tagID) {
    this(drive, 
        () -> PoseConstants.poseHashMap.get(new Tuple<Camera, Integer>(camera, DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red 
            ? tagID.redID : tagID.blueID)));
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
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
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
    lastSetpointRotation = target.get().getRotation();
    lastTime = Timer.getTimestamp();
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
    double ffScaler =
        MathUtil.clamp(
            (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
            0.0,
            1.0);
    driveErrorAbs = currentDistance;
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.calculate(driveErrorAbs, 0.0)
            + driveController.getSetpoint().velocity * ffScaler;
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;

    var angleToTarget = new Rotation2d(
        Math.atan2(
            currentPose.getTranslation().getY() - targetPose.getTranslation().getY(),
            currentPose.getTranslation().getX() - targetPose.getTranslation().getX()));

    lastSetpointTranslation =
        new Pose2d(targetPose.getTranslation(), angleToTarget)
            .transformBy(GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.calculate(
                currentPose.getRotation().getRadians(),
                new TrapezoidProfile.State(
                    targetPose.getRotation().getRadians(),
                    (targetPose.getRotation().minus(lastSetpointRotation)).getRadians()
                        / (Timer.getTimestamp() - lastTime)))
            + thetaController.getSetpoint().velocity * ffScaler;
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;
    lastSetpointRotation = targetPose.getRotation();
    Translation2d driveVelocity =
        new Pose2d(Translation2d.kZero, angleToTarget)
            .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
            .getTranslation();
    lastTime = Timer.getTimestamp();

    // Scale feedback velocities by input ff
    final double linearS = linearFF.get().getNorm() * 3.0;
    final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
    driveVelocity =
        driveVelocity.interpolate(linearFF.get().times(DrivetrainConstants.MAX_SPEED), linearS);
    thetaVelocity =
        MathUtil.interpolate(
            thetaVelocity, omegaFF.getAsDouble() * DrivetrainConstants.MAX_ANGULAR_RATE, thetaS);

    // Command speeds
    drive.runVelocity(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity);

    // TODO: Log data
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    running = false;
    // Clear logs
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }
}
