package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.Constants.PoseConstants.ScoringPoses;
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

    private PIDController controllerR = new PIDController(0, 0, 0);

    private Transform3d currentTransform = new Transform3d();
    private Pose2d targetPose = new Pose2d();


    /**
     * Used to align to Tag
     * will always use PID Controllers
     * @param vision
     * @param drivetrain
     */
    public PoseBasedAutoAlign(PhotonVision vision, Swerve drivetrain, Camera camera) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.camera = camera;
        // targetPose = PoseConstants.poseHashMap.get(new Tuple<Camera, Integer>(camera, 19));

        targetPose = new Pose2d(3.656, 5.122, new Rotation2d(300));


        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

        // controllerX.setSetpoint(0);
        controllerX.setTolerance(LightningShuffleboard.getDouble("TestAutoAlign", "x tolerance", 0));
        // controllerX.enableContinuousInput(0, 360);

        // controllerY.setSetpoint(Units.inchesToMeters(9.228));
        controllerY.setTolerance(LightningShuffleboard.getDouble("TestAutoAlign", "y tolerance", 0));


        // controllerR.setSetpoint(0);
        controllerY.setTolerance(LightningShuffleboard.getDouble("TestAutoAlign", "z tolerance", 0));
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

        double yVeloc = controllerY.calculate(currentPose.getX(), targetPose.getY());
        double xVeloc = controllerX.calculate(currentPose.getY(), targetPose.getX());
        double rotationVeloc = controllerR.calculate(currentPose.getRotation().getDegrees(), targetPose.getRotation().getDegrees());

        drivetrain.setControl(DriveRequests.getRobotCentric(
            xVeloc, 
            yVeloc, 
            rotationVeloc));

        // setGains(controllerX, "x");        
        // setGains(controllerY, "y");        
        // setGains(controllerR, "r");
        
        // LightningShuffleboard.setPose2d("TestAutoAlign", "current pose veloc", currentPose.toPose2d());
        // LightningShuffleboard.setPose2d("TestAutoAlign", "target pose", targetPose.toPose2d());

        // LightningShuffleboard.setDouble("TestAutoAlign", "Y", currentTransform.getX());
        // LightningShuffleboard.setDouble("TestAutoAlign", "X", currentTransform.getY());
        // LightningShuffleboard.setDouble("TestAutoAlign", "R", currentTransform.getRotation().toRotation2d().getDegrees());

        LightningShuffleboard.setDouble("TestAutoAlign", "X veloc", xVeloc);
        LightningShuffleboard.setDouble("TestAutoAlign", "Y veloc", yVeloc);
        LightningShuffleboard.setDouble("TestAutoAlign", "R veloc", rotationVeloc);

        // LightningShuffleboard.setPose2d("TestAutoAlign", "target pose", targetPose);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }

    private void setGains(PIDController controller, String key) {
        controller.setPID(
            LightningShuffleboard.getDouble("TestAutoAlign", key + " Kp", 0),
            LightningShuffleboard.getDouble("TestAutoAlign", key + " Ki", 0),
            LightningShuffleboard.getDouble("TestAutoAlign", key + " Kd", 0));

        controller.setTolerance(LightningShuffleboard.getDouble("TestAutoAlign", key + " tolerance", 0));
    }
}
