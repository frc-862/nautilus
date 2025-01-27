package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class TagAutoAlign extends Command {
    private PhotonVision vision;
    private Swerve drivetrain;
  
    private PIDController controllerX = new PIDController(AutoAlignConstants.X_Kp, AutoAlignConstants.X_Ki, 
        AutoAlignConstants.X_Kd);

    private PIDController controllerY = new PIDController(AutoAlignConstants.Y_Kp, AutoAlignConstants.Y_Ki, 
        AutoAlignConstants.Y_Kd);

    private double dx_dt;
    private double dy_dt;

    private double pitch;
    private double yaw;

    private XboxController driver;

    /**
     * Used to align to Tag
     * will always use PID Controllers
     * @param vision
     * @param drivetrain
     */
    public TagAutoAlign(PhotonVision vision, Swerve drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    /**
     * used to align to tag
     * will use driver input for robot centric foreward movement in teleop
     * @param vision
     * @param drivetrain
     * @param driver
     */
    public TagAutoAlign (PhotonVision vision, Swerve drivetrain, XboxController driver){
        this(vision, drivetrain);
        this.driver = driver;
    }

    @Override
    public void initialize() {
        // zero velocity values

        controllerX = new PIDController(
            LightningShuffleboard.getDouble("TestAutoAlign", "X Kp", 0), 
            LightningShuffleboard.getDouble("TestAutoAlign", "X Ki", 0), 
            LightningShuffleboard.getDouble("TestAutoAlign", "X Kd", 0));

        controllerY = new PIDController(
            LightningShuffleboard.getDouble("TestAutoAlign", "Y Kp", 0), 
            LightningShuffleboard.getDouble("TestAutoAlign", "Y Ki", 0), 
            LightningShuffleboard.getDouble("TestAutoAlign", "Y Kd", 0));

        controllerX.setSetpoint(0);
        controllerX.setTolerance(AutoAlignConstants.AutoAlignTolerance);

        controllerY.setSetpoint(0);
        controllerY.setTolerance(AutoAlignConstants.AutoAlignTolerance);

    }

    @Override
    public void execute() {

        if(!vision.hasTarget()){

            System.out.println("Error: Cannot See April Tag");

            drivetrain.setControl(DriveRequests.getRobotCentric(0, 0, 0));
            return;
        }

        // update pitch and yaw values

        pitch = vision.getPitch2d();
        yaw = vision.getYaw2d();

        // use pitch and yaw to calculate velocity values

        dx_dt = controllerX.calculate(yaw);
        dy_dt = !DriverStation.isTeleop() ? -controllerY.calculate(pitch) : // if in teleop use driver input for foreward movement unless driver is null
            (driver == null ? -controllerY.calculate(pitch) : -driver.getLeftY());
        

        if (!DriverStation.isFMSAttached()){
            LightningShuffleboard.setDouble("TestAutoAlign", "X diffrence (yaw)", yaw);
            LightningShuffleboard.setDouble("TestAutoAlign", "Y diffrence (pitch)", pitch);

            LightningShuffleboard.setDouble("TestAutoAlign", "X speed", dx_dt);
            LightningShuffleboard.setDouble("TestAutoAlign", "Y speed", dy_dt);
        }

        // give the new velocity values to the drivetrain
        drivetrain.setControl(DriveRequests.getRobotCentric(dx_dt, dy_dt, 0));
    }



    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(DriveRequests.getRobotCentric(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(pitch) < AutoAlignConstants.AutoAlignTolerance && 
            Math.abs(yaw) < AutoAlignConstants.AutoAlignTolerance;
    }
}
