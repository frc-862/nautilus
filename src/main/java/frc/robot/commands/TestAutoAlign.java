package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class TestAutoAlign extends Command {
    PhotonVision vision;
    Swerve drivetrain;

    Transform3d diffFromTag;
  
    PIDController controllerX = new PIDController(AutoAlignConstants.XAutoAlignKp, AutoAlignConstants.XAutoAlignKi, 
        AutoAlignConstants.XAutoAlignKd);

    PIDController controllerY = new PIDController(AutoAlignConstants.YAutoAlignKp, AutoAlignConstants.YAutoAlignKi, 
        AutoAlignConstants.YAutoAlignKd);

    double dx_dt;
    double dy_dt;

    double pitch;
    double yaw;

    /**
     * Used to align to Tag
     * @param vision
     * @param drivetrain
     */
    public TestAutoAlign(PhotonVision vision, Swerve drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
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

        // update pitch and yaw values

        pitch = vision.getPitch();
        yaw = vision.getYaw();

        if(!vision.hasTarget()){

            System.out.println("Error: Cannot See April Tag");

            drivetrain.setControl(DriveRequests.getRobotCentric(0, 0, 0));
            return;
        }

        // use pitch and yaw to calculate velocity values

        dx_dt = controllerX.calculate(yaw);
        dy_dt = -controllerY.calculate(pitch);
        

        if (!DriverStation.isFMSAttached()){
            LightningShuffleboard.setDouble("TestAutoAlign", "X diffrence", yaw);
            LightningShuffleboard.setDouble("TestAutoAlign", "Y diffrence", pitch);

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
        return Math.abs(vision.getPitch()) < AutoAlignConstants.AutoAlignTolerance && 
            Math.abs(vision.getYaw()) < AutoAlignConstants.AutoAlignTolerance;
    }
}
