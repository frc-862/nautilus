// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDStates;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.PoseConstants.LightningTagID;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.util.Tuple;

public class PoseBasedAutoAlign extends Command {

    private Swerve drivetrain;
    private Camera camera;
    private LEDs leds;

    private double driveTolerance = AutoAlignConstants.POSEBASED_DRIVE_TOLERANCE;

    private PIDController xPID = new PIDController(AutoAlignConstants.POSEBASED_DRIVE_P,
            AutoAlignConstants.POSEBASED_DRIVE_I, AutoAlignConstants.POSEBASED_DRIVE_D);
    private PIDController yPID = new PIDController(AutoAlignConstants.POSEBASED_DRIVE_P,
            AutoAlignConstants.POSEBASED_DRIVE_I, AutoAlignConstants.POSEBASED_DRIVE_D);
    private PIDController rPID = new PIDController(AutoAlignConstants.POSEBASED_ROT_P,
            AutoAlignConstants.POSEBASED_ROT_I, AutoAlignConstants.POSEBASED_ROT_D);

    private double driveKS = AutoAlignConstants.POSEBASED_DRIVE_KS;
    private double rotKS = AutoAlignConstants.POSEBASED_ROT_KS;

    private Pose2d targetPose = new Pose2d();

    private int tagID = 0;
    private boolean customTagSet = false;
    private boolean invokeCancel = false;
    private boolean isL1 = false;

    private boolean overrideYPID = false;

    private LightningTagID codeID = LightningTagID.One;

    /**
     * Used to align to Tag
     * will always use PID Controllers
     *
     * @param drivetrain
     * @param camera
     * @param leds
     * @param codeID     the Lightning-specific ID code for the tag
     */
    public PoseBasedAutoAlign(Swerve drivetrain, Camera camera, LEDs leds, LightningTagID codeID) {
        this(drivetrain, camera, leds);

        customTagSet = true;
        this.codeID = codeID;
    }

    /**
     * Used to align to Tag
     * will always use PID Controllers
     *
     * @param drivetrain
     * @param camera
     * @param leds
     * @param isL1       whether to use L1 pose hashmap
     * @param codeID     the Lightning-specific ID code for the tag
     */
    public PoseBasedAutoAlign(Swerve drivetrain, Camera camera, boolean isL1, LEDs leds) {
        this(drivetrain, camera, leds);
        this.isL1 = isL1;
    }

    /**
     * Used to align to Tag
     * will always use PID Controllers
     *
     * @param drivetrain
     * @param camera
     * @param leds
     */
    public PoseBasedAutoAlign(Swerve drivetrain, Camera camera, LEDs leds) {
        this.drivetrain = drivetrain;
        this.camera = camera;
        this.leds = leds;

        tagID = 0;
        customTagSet = false;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        invokeCancel = false;
        // Get tagID from codeID
        if (codeID != null) {
            tagID = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red
                    ? codeID.redID
                    : codeID.blueID;
        }

        // Get the tag in front of the robot
        if (!customTagSet) {
            tagID = PoseConstants.getScorePose(drivetrain.getPose());
        }

        // If we have a target tag, set target pose
        if (tagID == 0) {
            invokeCancel = true;
            CommandScheduler.getInstance().cancel(this);
        } else {
            if (isL1) {
                targetPose = PoseConstants.l1PoseHashMap.get(new Tuple<>(camera, tagID));
            } else {
                targetPose = PoseConstants.poseHashMap.get(new Tuple<>(camera, tagID));
            }

            LightningShuffleboard.setDouble("TestAutoAlign", "Tag", tagID);

            if (targetPose != null) {
                LightningShuffleboard.setString("TestAutoAlign", "Target Pose", targetPose.toString());
            }
        }

        xPID.setTolerance(driveTolerance);

        yPID.setTolerance(driveTolerance);

        rPID.setTolerance(AutoAlignConstants.POSEBASED_ROT_TOLERANCE);
        rPID.enableContinuousInput(0, 360);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();

        double xVeloc = xPID.calculate(currentPose.getX(), targetPose.getX())
                + (Math.signum(xPID.getError()) * (!xPID.atSetpoint() ? driveKS : 0));
        double yVeloc = yPID.calculate(currentPose.getY(), targetPose.getY())
                + (Math.signum(yPID.getError()) * (!yPID.atSetpoint() ? driveKS : 0));
        double rotationVeloc = rPID.calculate(currentPose.getRotation().getDegrees(),
                targetPose.getRotation().getDegrees());// + Math.signum(controllerY.getError()) * rKs;

        drivetrain.setControl(DriveRequests.getAutoAlign(xVeloc, yVeloc, rotationVeloc));

        LightningShuffleboard.setDouble("TestAutoAlign", "X veloc", xVeloc);
        LightningShuffleboard.setDouble("TestAutoAlign", "Y veloc", yVeloc);
        LightningShuffleboard.setDouble("TestAutoAlign", "R veloc", rotationVeloc);

        // setDriveGains();
        // setRotGains();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            leds.strip.enableState(LEDStates.ALIGNED).withDeadline(new WaitCommand(LEDConstants.PULSE_TIME)).schedule();
        }
        if (!DriverStation.isAutonomous() && onTarget()) {
            RobotContainer.hapticDriverCommand().schedule();
        }

        // Ensure we are not moving after we stop
        drivetrain.setControl(DriveRequests.getBrake().get());
    }

    @Override
    public boolean isFinished() {
        return onTarget() || invokeCancel;
    }

    public boolean onTarget() {
        return xPID.atSetpoint() && (!overrideYPID ? yPID.atSetpoint() : true) && rPID.atSetpoint();
    }

    private void setDriveGains() {
        String key = "DRV";
        xPID.setPID(
                LightningShuffleboard.getDouble("TestAutoAlign", key + " Kp", AutoAlignConstants.POSEBASED_DRIVE_P),
                LightningShuffleboard.getDouble("TestAutoAlign", key + " Ki", AutoAlignConstants.POSEBASED_DRIVE_I),
                LightningShuffleboard.getDouble("TestAutoAlign", key + " Kd", AutoAlignConstants.POSEBASED_DRIVE_D));
        driveKS = LightningShuffleboard.getDouble("TestAutoAlign", key + " KStatic",
                AutoAlignConstants.POSEBASED_DRIVE_KS);

        yPID.setPID(xPID.getP(), xPID.getI(), xPID.getD());

        xPID.setTolerance(LightningShuffleboard.getDouble("TestAutoAlign", key + " tolerance", driveTolerance));
    }

    private void setRotGains() {
        String key = "ROT";

        rPID.setPID(
                LightningShuffleboard.getDouble("TestAutoAlign", key + " Kp", AutoAlignConstants.POSEBASED_ROT_P),
                LightningShuffleboard.getDouble("TestAutoAlign", key + " Ki", AutoAlignConstants.POSEBASED_ROT_I),
                LightningShuffleboard.getDouble("TestAutoAlign", key + " Kd", AutoAlignConstants.POSEBASED_ROT_D));
        rotKS = LightningShuffleboard.getDouble("TestAutoAlign", key + "KStatic", AutoAlignConstants.POSEBASED_ROT_KS);

        rPID.setTolerance(LightningShuffleboard.getDouble("TestAutoAlign", key + " tolerance",
                AutoAlignConstants.POSEBASED_ROT_TOLERANCE));
    }
}
