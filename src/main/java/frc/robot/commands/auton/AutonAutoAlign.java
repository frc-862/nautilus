// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.Constants.FishingRodConstants.RodStates;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDStates;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.PoseConstants.LightningTagID;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.subsystems.FishingRod;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.util.Tuple;

public class AutonAutoAlign extends Command {

    private Swerve drivetrain;
    private Camera camera;
    private LEDs leds;
    private FishingRod rod;

    private double tolerance = 0.025;

    private PIDController xPID = new PIDController(AutoAlignConstants.THREE_DEE_xP, AutoAlignConstants.THREE_DEE_xI,
            AutoAlignConstants.THREE_DEE_xD);

    private PIDController yPID = new PIDController(AutoAlignConstants.THREE_DEE_yP, AutoAlignConstants.THREE_DEE_yI,
            AutoAlignConstants.THREE_DEE_yD);

    private PIDController rPID = new PIDController(AutoAlignConstants.THREE_DEE_rP, AutoAlignConstants.THREE_DEE_rI,
            AutoAlignConstants.THREE_DEE_rD);

    private Pose2d targetPose = new Pose2d();

    private int tagID = 0;
    private boolean customTagSet = false;
    private boolean invokeCancel = false;

    private LightningTagID codeID = LightningTagID.One;

    private boolean doEle = false;
    private final double deployVeloc = 0.25;
    private boolean reachedDeployVelOnce = false;

    /**
     * Used to align to Tag
     * will always use PID Controllers
     * 
     * @param drivetrain
     * @param camera
     * @param leds
     * @param codeID     the Lightning-specific ID code for the tag
     */
    public AutonAutoAlign(Swerve drivetrain, Camera camera, LEDs leds, FishingRod rod, LightningTagID codeID) {
        this(drivetrain, camera, leds, rod);

        customTagSet = true;
        this.codeID = codeID;

        this.rod = rod;
    }

    /**
     * Used to align to Tag
     * will always use PID Controllers
     * 
     * @param drivetrain
     * @param camera
     * @param leds
     */
    public AutonAutoAlign(Swerve drivetrain, Camera camera, LEDs leds, FishingRod rod) {
        this.drivetrain = drivetrain;
        this.camera = camera;
        this.leds = leds;
        this.rod = rod;

        tagID = 0;
        customTagSet = false;

        addRequirements(drivetrain, rod);
    }

    @Override
    public void initialize() {
        doEle = false;
        reachedDeployVelOnce = false;

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
            targetPose = PoseConstants.poseHashMap.get(new Tuple<Camera, Integer>(camera, tagID));

            LightningShuffleboard.setDouble("TestAutoAlign", "Tag", tagID);

            if (targetPose != null) {
                LightningShuffleboard.setString("TestAutoAlign", "Target Pose", targetPose.toString());
            }
        }

        xPID.setTolerance(tolerance);

        yPID.setTolerance(tolerance);

        rPID.setTolerance(2.5);
        rPID.enableContinuousInput(0, 360);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();

        // double kS = 0.025;
        double kS = 0.01;
        // double rKs = LightningShuffleboard.getDouble("TestAutoAlign", "R static",
        // 0d);

        double xVeloc = xPID.calculate(currentPose.getX(), targetPose.getX())
                + (Math.signum(xPID.getError()) * (!xPID.atSetpoint() ? kS : 0));
        double yVeloc = yPID.calculate(currentPose.getY(), targetPose.getY())
                + (Math.signum(yPID.getError()) * (!yPID.atSetpoint() ? kS : 0));
        double rotationVeloc = rPID.calculate(currentPose.getRotation().getDegrees(),
                targetPose.getRotation().getDegrees());// + Math.signum(controllerY.getError()) * rKs;

                
        //if speed goes above threshold, start checking if we go back below threshold
        if (Math.abs(xVeloc) > deployVeloc && Math.abs(yVeloc) > deployVeloc) {
            reachedDeployVelOnce = true;
        }

        //
        if (Math.abs(xVeloc) < deployVeloc && Math.abs(yVeloc) < deployVeloc && !doEle && reachedDeployVelOnce) {
            doEle = true;
            rod.setState(RodStates.L4);
        } 
        

        //once we've started moving the elvator, make sure we never go faster than threshold (to be safe)
        if (doEle) {
            xVeloc = MathUtil.clamp(xVeloc, -deployVeloc, deployVeloc);
            yVeloc = MathUtil.clamp(yVeloc, -deployVeloc, deployVeloc);
        }

        drivetrain.setControl(DriveRequests.getAutoAlign(xVeloc, yVeloc, rotationVeloc));

        LightningShuffleboard.setDouble("TestAutoAlign", "X veloc", xVeloc);
        LightningShuffleboard.setDouble("TestAutoAlign", "Y veloc", yVeloc);
        LightningShuffleboard.setDouble("TestAutoAlign", "R veloc", rotationVeloc);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            leds.strip.enableState(LEDStates.ALIGNED).withDeadline(new WaitCommand(LEDConstants.PULSE_TIME)).schedule();
        }
        if (!DriverStation.isAutonomous() && onTarget()) {
            RobotContainer.hapticDriverCommand().schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return (onTarget() && rod.onTarget()) || invokeCancel;
    }

    public boolean onTarget() {
        return xPID.atSetpoint() && yPID.atSetpoint() && rPID.atSetpoint();
    }
}
