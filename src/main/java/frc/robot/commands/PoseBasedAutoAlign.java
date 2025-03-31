// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.Constants.FishingRodConstants.RodStates;
import frc.robot.Constants.FishingRodConstants.RodTransitionStates;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDStates;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.PoseConstants.LightningTagID;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FishingRod;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.util.Tuple;

public class PoseBasedAutoAlign extends Command {
    private Swerve drivetrain;
    private LEDs leds;

    private double driveTolerance = AutoAlignConstants.TELE_DRIVE_TOLERANCE;

    private PIDController xPID = new PIDController(AutoAlignConstants.TELE_DRIVE_P, AutoAlignConstants.TELE_DRIVE_I, AutoAlignConstants.TELE_DRIVE_D);
    private PIDController yPID = new PIDController(AutoAlignConstants.TELE_DRIVE_P, AutoAlignConstants.TELE_DRIVE_I, AutoAlignConstants.TELE_DRIVE_D);
    private PIDController rPID = new PIDController(AutoAlignConstants.POSEBASED_ROT_P, AutoAlignConstants.POSEBASED_ROT_I, AutoAlignConstants.POSEBASED_ROT_D);

    private boolean overrideYPID = false;

    protected Pose2d targetPose = new Pose2d();

    protected boolean isWithY = true;
    protected DoubleSupplier yVelSupplier = () -> 0.0;
    
    private FishingRod rod;
    private Supplier<RodStates> targetRodState;
    private boolean isWithRodState = false;
    private boolean hasDeployedRod = false; // used to check if we have deployed the fishing rod yet

   
    /**
     * Used to align to Tag
     * will always use PID Controllers
     *
     * @param drivetrain
     * @param camera
     * @param leds
     */
    public PoseBasedAutoAlign(Swerve drivetrain, LEDs leds) {
        this.drivetrain = drivetrain;
        this.leds = leds;
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
       LightningShuffleboard.setPose2d("TestAutoAlign", "Target Pose", targetPose);

        xPID.setPID(AutoAlignConstants.AUTON_DRIVE_P, AutoAlignConstants.AUTON_DRIVE_I, AutoAlignConstants.AUTON_DRIVE_D);
        yPID.setPID(xPID.getP(), xPID.getI(), xPID.getD());

        xPID.setTolerance(driveTolerance);

        yPID.setTolerance(driveTolerance);

        rPID.setTolerance(AutoAlignConstants.POSEBASED_ROT_TOLERANCE);
        rPID.enableContinuousInput(0, 360);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();

        double xVeloc = xPID.calculate(currentPose.getX(), targetPose.getX())
                + (Math.signum(xPID.getError()) * (!xPID.atSetpoint() ? AutoAlignConstants.AUTON_DRIVE_KS : 0));
        double yVeloc = yPID.calculate(currentPose.getY(), targetPose.getY())
                + (Math.signum(yPID.getError()) * (!yPID.atSetpoint() ? AutoAlignConstants.AUTON_DRIVE_KS : 0));
        double rotationVeloc = rPID.calculate(currentPose.getRotation().getDegrees(),
                targetPose.getRotation().getDegrees());// + Math.signum(controllerY.getError()) * rKs;

        if (!isWithY && yVelSupplier != null) {
            yVeloc = yVelSupplier.getAsDouble();
        }

        if (isWithRodState) {
            // if we've reached the threshold once, and we're below the threshold, deploy the ele
            if (Math.abs(xVeloc) < AutoAlignConstants.DEPLOY_VEL && Math.abs(yVeloc) < AutoAlignConstants.DEPLOY_VEL && !hasDeployedRod) {
                hasDeployedRod = true;
                invokeRod();
            }
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

            if (rod.getState() != targetRodState.get() && isWithRodState) {
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
        return onTarget();
    }

    public boolean onTarget() {
        return xPID.atSetpoint() && (!overrideYPID ? yPID.atSetpoint() : true) && rPID.atSetpoint();
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

    private void setDriveGains() {
        String key = "DRV";
        xPID.setPID(
                LightningShuffleboard.getDouble("TestAutoAlign", key + " Kp", AutoAlignConstants.TELE_DRIVE_P),
                LightningShuffleboard.getDouble("TestAutoAlign", key + " Ki", AutoAlignConstants.TELE_DRIVE_I),
                LightningShuffleboard.getDouble("TestAutoAlign", key + " Kd", AutoAlignConstants.TELE_DRIVE_D));

        yPID.setPID(xPID.getP(), xPID.getI(), xPID.getD());

        xPID.setTolerance(LightningShuffleboard.getDouble("TestAutoAlign", key + " tolerance", driveTolerance));
    }

    private void setRotGains() {
        String key = "ROT";

        rPID.setPID(
                LightningShuffleboard.getDouble("TestAutoAlign", key + " Kp", AutoAlignConstants.POSEBASED_ROT_P),
                LightningShuffleboard.getDouble("TestAutoAlign", key + " Ki", AutoAlignConstants.POSEBASED_ROT_I),
                LightningShuffleboard.getDouble("TestAutoAlign", key + " Kd", AutoAlignConstants.POSEBASED_ROT_D));

        rPID.setTolerance(LightningShuffleboard.getDouble("TestAutoAlign", key + " tolerance",
                AutoAlignConstants.POSEBASED_ROT_TOLERANCE));
    }

    public static PoseBasedAutoAlign getLightningIDAutoAlign(Swerve drivetrain, Camera camera, LEDs leds, LightningTagID tagID) {
        return new PoseBasedAutoAlign(drivetrain, leds) 
        {
            @Override
            public void initialize() {
                int ID = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red ? tagID.redID : tagID.blueID;

                targetPose = PoseConstants.poseHashMap.get(new Tuple<Camera, Integer>(camera, ID));

                super.initialize();
            }
        };
    }

    public static PoseBasedAutoAlign getTagIDAutoAlign(Swerve drivetrain, Camera camera, LEDs leds, int ID) {
        return new PoseBasedAutoAlign(drivetrain, leds) 
        {
            @Override
            public void initialize() {
                targetPose = PoseConstants.poseHashMap.get(new Tuple<Camera, Integer>(camera, ID));

                super.initialize();
            }
        };
    }

    public static PoseBasedAutoAlign getPoseAutoAlign(Swerve drivetrain, Camera camera, LEDs leds) {
        return new PoseBasedAutoAlign(drivetrain, leds) 
        {
            @Override
            public void initialize() {
                int ID = PoseConstants.getScorePose(drivetrain.getPose());
                if (ID == 0) {
                    CommandScheduler.getInstance().cancel(this);
                    return;
                }

                targetPose = PoseConstants.poseHashMap.get(new Tuple<Camera, Integer>(camera, ID));

                super.initialize();
            }
        };
    }

    public static PoseBasedAutoAlign getBargeAutoAlign(Swerve drivetrain, LEDs leds, DoubleSupplier yVelSupplier) {
        return new PoseBasedAutoAlign(drivetrain, leds) 
        {
            @Override
            public void initialize() {
                this.yVelSupplier = yVelSupplier;
                this.isWithY = false;

                targetPose = DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? PoseConstants.RED_BARGE : PoseConstants.BLUE_BARGE;

                super.initialize();
            }
        };
    }

    public static PoseBasedAutoAlign getL1PoseAutoAlign(Swerve drivetrain, Camera camera, LEDs leds) {
        return new PoseBasedAutoAlign(drivetrain, leds) 
        {
            @Override
            public void initialize() {
                int ID = PoseConstants.getScorePose(drivetrain.getPose());
                if (ID == 0) {
                    CommandScheduler.getInstance().cancel(this);
                    return;
                }

                targetPose = PoseConstants.poseHashMap.get(new Tuple<Camera, Integer>(camera, ID));

                super.initialize();
            }
        };
    }

    public PoseBasedAutoAlign withRodState(FishingRod rod, Supplier<RodStates> state) {
        this.rod = rod; // set the fishing rod subsystem
        this.targetRodState = state; // set the target rod state to the one passed in

        isWithRodState = true; // set the flag to true to indicate we are using a rod state

        addRequirements(rod); // add the fishing rod subsystem as a requirement to this command

        return this; // return this to allow for method chaining
    }
}
