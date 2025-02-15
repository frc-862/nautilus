package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class ThreeDeeAutoAlign extends Command {
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
     * @param camera
     */
    public ThreeDeeAutoAlign(PhotonVision vision, Swerve drivetrain, Camera camera) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.camera = camera;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // controllerX = new PIDController(
        //     LightningShuffleboard.getDouble("TestAutoAlign", "x Kp", 0),
        //     LightningShuffleboard.getDouble("TestAutoAlign", "x Ki", 0),
        //     LightningShuffleboard.getDouble("TestAutoAlign", "x Kd", 0));

        // controllerY = new PIDController(
        //     LightningShuffleboard.getDouble("TestAutoAlign", "y Kp", 0),
        //     LightningShuffleboard.getDouble("TestAutoAlign", "y Ki", 0),
        //     LightningShuffleboard.getDouble("TestAutoAlign", "y Kd", 0));

        // controllerR = new PIDController(
        //     LightningShuffleboard.getDouble("TestAutoAlign", "r Kp", 0),
        //     LightningShuffleboard.getDouble("TestAutoAlign", "r Ki", 0),
        //     LightningShuffleboard.getDouble("TestAutoAlign", "r Kd", 0));



        // controllerX.setSetpoint(0);
        controllerX.setTolerance(LightningShuffleboard.getDouble("TestAutoAlign", "x tolerance", 0));
        // controllerX.enableContinuousInput(0, 360);

        // controllerY.setSetpoint(Units.inchesToMeters(9.228));
        controllerY.setTolerance(LightningShuffleboard.getDouble("TestAutoAlign", "y tolerance", 0));


        // controllerR.setSetpoint(0);
        controllerY.setTolerance(LightningShuffleboard.getDouble("TestAutoAlign", "z tolerance", 0));
        controllerR.enableContinuousInput(0, 360);
    }

    @Override
    public void execute() {
        try {
            // currentPose = new Pose3d(drivetrain.getState().Pose);
            // targetPose = currentPose.plus(vision.getTransformToTag(camera).plus(new Transform3d(-9.228, 0, 0, new Rotation3d())));
            currentTransform = vision.getTransformToTag(camera);
        } catch (Exception e) {
            System.out.println("Error: Cannot See April Tag");
            cancel();
        }
        // MathUtil.inputModulus(drivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 360);
        // double xVeloc = controllerX.calculate(currentPose.getX(), targetPose.getX());
        // double yVeloc = controllerY.calculate(currentPose.getY(), targetPose.getY());
        // double rotationVeloc = controllerR.calculate(currentPose.getRotation().toRotation2d().getDegrees(), targetPose.getRotation().toRotation2d().getDegrees());

        double yVeloc = controllerY.calculate(currentTransform.getX());
        double xVeloc = controllerX.calculate(currentTransform.getY());
        double rotationVeloc = controllerR.calculate(currentTransform.getRotation().toRotation2d().getDegrees());

        drivetrain.setControl(DriveRequests.getRobotCentric(
            xVeloc, 
            yVeloc, 
            rotationVeloc));

        setGains(controllerX, "x");        
        setGains(controllerY, "y");        
        setGains(controllerR, "r");
        
        // LightningShuffleboard.setPose2d("TestAutoAlign", "current pose veloc", currentPose.toPose2d());
        // LightningShuffleboard.setPose2d("TestAutoAlign", "target pose", targetPose.toPose2d());

        LightningShuffleboard.setDouble("TestAutoAlign", "Y", currentTransform.getX());
        LightningShuffleboard.setDouble("TestAutoAlign", "X", currentTransform.getY());
        LightningShuffleboard.setDouble("TestAutoAlign", "R", currentTransform.getRotation().toRotation2d().getDegrees());

        LightningShuffleboard.setDouble("TestAutoAlign", "X veloc", xVeloc);
        LightningShuffleboard.setDouble("TestAutoAlign", "Y veloc", yVeloc);
        LightningShuffleboard.setDouble("TestAutoAlign", "R veloc", rotationVeloc);
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
