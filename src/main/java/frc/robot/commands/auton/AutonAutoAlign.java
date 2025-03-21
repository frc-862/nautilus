// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.Constants.FishingRodConstants.RodStates;
import frc.robot.Constants.FishingRodConstants.RodTransitionStates;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDStates;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.PoseConstants.LightningTagID;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.subsystems.FishingRod;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.util.Tuple;

public class AutonAutoAlign extends Command {

    private Swerve drivetrain;
    private Camera camera;
    private LEDs leds;
    private FishingRod rod;

    private double driveTolerance = AutoAlignConstants.TELE_DRIVE_TOLERANCE;

    private PIDController xPID = new PIDController(AutoAlignConstants.TELE_DRIVE_P,
            AutoAlignConstants.TELE_DRIVE_I, AutoAlignConstants.TELE_DRIVE_D);
    private PIDController yPID = new PIDController(AutoAlignConstants.TELE_DRIVE_P,
            AutoAlignConstants.TELE_DRIVE_I, AutoAlignConstants.TELE_DRIVE_D);
    private PIDController rPID = new PIDController(AutoAlignConstants.POSEBASED_ROT_P,
            AutoAlignConstants.POSEBASED_ROT_I, AutoAlignConstants.POSEBASED_ROT_D);

    private double driveKS = AutoAlignConstants.TELE_DRIVE_KS;
    private double rotKS = AutoAlignConstants.POSEBASED_ROT_KS;

    private Pose2d targetPose = new Pose2d();

    private int tagID = 0;
    private boolean customTagSet = false;
    private boolean invokeCancel = false;

    private LightningTagID codeID = LightningTagID.One;

    // AUTON AUTO ALIGN SPECIFIC VARIABLES
    private boolean hasDeployed = false;
    private boolean reachedDeployVelOnce = false;

    private Supplier<RodStates> targetRodState;

    /**
     * Used to align to a given Tag
     * will always use PID Controllers
     *
     * @param drivetrain
     * @param camera
     * @param leds
     * @param rod
     * @param codeID     the Lightning-specific ID code for the tag
     * @param targetRodState supplier for the rod state to go to when ready
     */
    public AutonAutoAlign(Swerve drivetrain, Camera camera, LEDs leds, FishingRod rod, LightningTagID codeID, Supplier<RodStates> targetRodState) {
        this(drivetrain, camera, leds, rod, targetRodState);

        customTagSet = true;
        this.codeID = codeID;
    }

    /**
     * Used to align to Tag based on current pose
     * will always use PID Controllers
     *
     * @param drivetrain
     * @param camera
     * @param leds
     * @param rod
     * @param targetRodState supplier for the rod state to go to when ready
     */
    public AutonAutoAlign(Swerve drivetrain, Camera camera, LEDs leds, FishingRod rod, Supplier<RodStates> targetRodState) {
        this.drivetrain = drivetrain;
        this.camera = camera;
        this.leds = leds;
        this.rod = rod;

        tagID = 0;
        customTagSet = false;

        this.targetRodState = targetRodState;

        if (targetRodState.get() != RodStates.L4) {
            addRequirements(drivetrain);
        } else {
            addRequirements(drivetrain, rod);
        }
    }

    @Override
    public void initialize() {
        hasDeployed = false;
        reachedDeployVelOnce = false;

        invokeCancel = false;
        // Get tagID from codeID
        if (codeID != null) {
            tagID = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red
                    ? codeID.redID
                    : codeID.blueID;
        } else {
            customTagSet = false;
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
            targetPose = PoseConstants.poseHashMap.get(new Tuple<Camera, Integer>(camera, tagID));

            LightningShuffleboard.setDouble("TestAutoAlign", "Tag", tagID);

            if (targetPose != null) {
                LightningShuffleboard.setString("TestAutoAlign", "Target Pose", targetPose.toString());
            }
        }

        if (DriverStation.isAutonomous()) {
            xPID.setPID(AutoAlignConstants.AUTON_DRIVE_P, AutoAlignConstants.AUTON_DRIVE_I, AutoAlignConstants.AUTON_DRIVE_D);
            yPID.setPID(xPID.getP(), xPID.getI(), xPID.getD());
        } else {
            xPID.setPID(AutoAlignConstants.TELE_DRIVE_P,
                AutoAlignConstants.TELE_DRIVE_I, AutoAlignConstants.TELE_DRIVE_D);
            yPID.setPID(xPID.getP(), xPID.getI(), xPID.getD());
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

        // if speed goes above threshold, start checking if we go back below threshold
        // if (Math.abs(xVeloc) > deployVeloc && Math.abs(yVeloc) > deployVeloc) {
        // reachedDeployVelOnce = true;
        // }

        // if we've reached the threshold once, and we're below the threshold, deploy the ele
        if (Math.abs(xVeloc) < AutoAlignConstants.DEPLOY_VEL && Math.abs(yVeloc) < AutoAlignConstants.DEPLOY_VEL && !hasDeployed) {
            hasDeployed = true;
            invokeRod();
        }

        // once we've started moving the elvator, make sure we never go faster than
        // threshold (to be safe)
        // if (doEle) {
        // xVeloc = MathUtil.clamp(xVeloc, -deployVeloc, deployVeloc);
        // yVeloc = MathUtil.clamp(yVeloc, -deployVeloc, deployVeloc);
        // }

        drivetrain.setControl(DriveRequests.getAutoAlign(xVeloc, yVeloc, rotationVeloc));

        LightningShuffleboard.setDouble("TestAutoAlign", "X veloc", xVeloc);
        LightningShuffleboard.setDouble("TestAutoAlign", "Y veloc", yVeloc);
        LightningShuffleboard.setDouble("TestAutoAlign", "R veloc", rotationVeloc);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            leds.strip.enableState(LEDStates.ALIGNED).withDeadline(new WaitCommand(LEDConstants.PULSE_TIME)).schedule();

            if (rod.getState() != targetRodState.get()) {
                invokeRod();
            }
        }

        if (!DriverStation.isAutonomous() && onTarget()) {
            RobotContainer.hapticDriverCommand().schedule();
        }

        // Ensure we are not moving after we stop
        drivetrain.setControl(DriveRequests.getBrake().get());
    }

    @Override
    public boolean isFinished() {
        return (onTarget() && rod.onTarget()) || invokeCancel;
    }

    public boolean onTarget() {
        return xPID.atSetpoint() && yPID.atSetpoint() && rPID.atSetpoint();
    }

    private void invokeRod() {
        if (targetRodState.get() == RodStates.DEFAULT) {
            return;
        }

        if (targetRodState.get() == RodStates.L4) {
            rod.setState(RodStates.L4, RodTransitionStates.CORAL_SAFE_ZONE);
        } //else {
        //     rod.setState(targetRodState.get());
        // }
    }
}
