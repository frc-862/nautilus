package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.util.Tuple;

public class PoseBasedAutoAlign extends Command {

    private PhotonVision vision;
    private Swerve drivetrain;
    private Camera camera;

    private PIDController controllerX = new PIDController(AutoAlignConstants.THREE_DEE_xP, AutoAlignConstants.THREE_DEE_xI,
        AutoAlignConstants.THREE_DEE_xD);

    private PIDController controllerY = new PIDController(AutoAlignConstants.THREE_DEE_yP, AutoAlignConstants.THREE_DEE_yI,
        AutoAlignConstants.THREE_DEE_yD);

    private PIDController controllerR = new PIDController(AutoAlignConstants.THREE_DEE_rP, AutoAlignConstants.THREE_DEE_rI,
        AutoAlignConstants.THREE_DEE_rD);

    private Transform3d currentTransform = new Transform3d();
    private Pose2d targetPose = new Pose2d();

    private StructPublisher<Pose2d> publisher;

    private int tagID = 0;
    private boolean customTagSet = false;
    private boolean invokeCancel = false;

    /**
     * Used to align to Tag
     * will always use PID Controllers
     * @param vision
     * @param drivetrain
     * @param camera
     * @param tag
     */
    public PoseBasedAutoAlign(PhotonVision vision, Swerve drivetrain, Camera camera, int tag) {
        this(vision, drivetrain, camera);

        customTagSet = true;

        tagID = tag;
    }

    public PoseBasedAutoAlign(PhotonVision vision, Swerve drivetrain, Camera camera) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.camera = camera;

        tagID = 0;
        customTagSet = false;
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        publisher = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("TestAutoAlign").getStructTopic("TARGET POSE", Pose2d.struct).publish();
        publisher.accept(targetPose);

        if (!customTagSet) {
            tagID = PoseConstants.getScorePose(drivetrain.getPose());
        }

        if (tagID == 0) {
            invokeCancel = true;
            CommandScheduler.getInstance().cancel(this);
        } else {
            targetPose = PoseConstants.poseHashMap.get(new Tuple<Camera, Integer>(camera, tagID));
        }

        controllerX.setTolerance(0.02);

        controllerY.setTolerance(0.02);


        // controllerR.setSetpoint(0);
        controllerR.setTolerance(2);
        controllerR.enableContinuousInput(0, 360);

        // try {
        //     vision.getTagNum(camera);
        //     PoseConstants.poseHashMap.get(new Tuple<Camera, Integer>(camera, vision.getTagNum(camera)));
        // } catch (Exception e) {
        //     System.out.println("Error: Cannot See April Tag");
        //     cancel();
        // }
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getPose();

        // double xKs = LightningShuffleboard.getDouble("TestAutoAlign", "Y static", 0d);
        // double yKs = LightningShuffleboard.getDouble("TestAutoAlign", "X static", 0d);
        // double rKs = LightningShuffleboard.getDouble("TestAutoAlign", "R static", 0d);

        double xVeloc = controllerX.calculate(currentPose.getX(), targetPose.getX());// + Math.signum(controllerY.getError()) * yKs;
        double yVeloc = controllerY.calculate(currentPose.getY(), targetPose.getY());// + Math.signum(controllerY.getError()) * xKs;
        double rotationVeloc = controllerR.calculate(currentPose.getRotation().getDegrees(), targetPose.getRotation().getDegrees());// + Math.signum(controllerY.getError()) * rKs;

        drivetrain.setControl(DriveRequests.getDrive(
            xVeloc,
            yVeloc,
            rotationVeloc));

        // setXGains();
        // setYGains();
        // setRGains();

        // LightningShuffleboard.setPose2d("TestAutoAlign", "current pose veloc", currentPose.toPose2d());
        // LightningShuffleboard.setPose2d("TestAutoAlign", "target pose", targetPose.toPose2d());

        LightningShuffleboard.setDouble("TestAutoAlign", "X veloc", xVeloc);
        LightningShuffleboard.setDouble("TestAutoAlign", "Y veloc", yVeloc);
        LightningShuffleboard.setDouble("TestAutoAlign", "R veloc", rotationVeloc);
        // LightningShuffleboard.setDouble("TestAutoAlign", "targ X", targetPose.getX());
        // LightningShuffleboard.setDouble("TestAutoAlign", "targ Y", targetPose.getY());
        // LightningShuffleboard.setDouble("TestAutoAlign", "targ R", targetPose.getRotation().getDegrees());
        // LightningShuffleboard.setPose2d("TestAutoAlign", "targg pose", targetPose);


        // LightningShuffleboard.setPose2d("TestAutoAlign", "target pose", targetPose);
    }

    @Override
    public void end(boolean interrupted) {
        publisher.close();
    }

    @Override
    public boolean isFinished() {
        return (controllerX.atSetpoint() && controllerY.atSetpoint() && controllerR.atSetpoint()) || invokeCancel;
    }

    private void setXGains() {
        String key = "x";
        controllerX.setPID(
            LightningShuffleboard.getDouble("TestAutoAlign", key + " Kp", AutoAlignConstants.THREE_DEE_xP),
            LightningShuffleboard.getDouble("TestAutoAlign", key + " Ki", AutoAlignConstants.THREE_DEE_xI),
            LightningShuffleboard.getDouble("TestAutoAlign", key + " Kd", AutoAlignConstants.THREE_DEE_xD));

        controllerX.setTolerance(LightningShuffleboard.getDouble("TestAutoAlign", key + " tolerance", 0));
    }

    private void setYGains() {
        String key = "y";

        controllerY.setPID(
            LightningShuffleboard.getDouble("TestAutoAlign", key + " Kp", AutoAlignConstants.THREE_DEE_yP),
            LightningShuffleboard.getDouble("TestAutoAlign", key + " Ki", AutoAlignConstants.THREE_DEE_yI),
            LightningShuffleboard.getDouble("TestAutoAlign", key + " Kd", AutoAlignConstants.THREE_DEE_yD));

        controllerY.setTolerance(LightningShuffleboard.getDouble("TestAutoAlign", key + " tolerance", 0));
    }
    private void setRGains() {
        String key = "r";

        controllerR.setPID(
            LightningShuffleboard.getDouble("TestAutoAlign", key + " Kp", AutoAlignConstants.THREE_DEE_rP),
            LightningShuffleboard.getDouble("TestAutoAlign", key + " Ki", AutoAlignConstants.THREE_DEE_rI),
            LightningShuffleboard.getDouble("TestAutoAlign", key + " Kd", AutoAlignConstants.THREE_DEE_rD));

        controllerR.setTolerance(LightningShuffleboard.getDouble("TestAutoAlign", key + " tolerance", 0));
    }
}