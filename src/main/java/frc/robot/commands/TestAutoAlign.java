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
    PIDController controllerRot = new PIDController(AutoAlignConstants.RotAutoAlignKp, 
        AutoAlignConstants.RotAutoAlignKi, AutoAlignConstants.RotAutoAlignKd);
    PIDController controllerX = new PIDController(AutoAlignConstants.XAutoAlignKp, 
        AutoAlignConstants.XAutoAlignKi, AutoAlignConstants.XAutoAlignKd);
    PIDController controllerY = new PIDController(AutoAlignConstants.YAutoAlignKp, 
        AutoAlignConstants.YAutoAlignKi, AutoAlignConstants.YAutoAlignKd);

    double dr_dt;
    double dx_dt;
    double dy_dt;

    public TestAutoAlign(PhotonVision vision, Swerve drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        dr_dt = 0;
        dx_dt = 0;
        dy_dt = 0;

        controllerRot.setSetpoint(0);
        controllerRot.setTolerance(AutoAlignConstants.AutoAlignTolerance);

        controllerX.setSetpoint(0);
        controllerX.setTolerance(AutoAlignConstants.AutoAlignTolerance);

        controllerY.setSetpoint(0);
        controllerY.setTolerance(AutoAlignConstants.AutoAlignTolerance);

        try{
            diffFromTag = vision.getTransformBestTarget();
        } catch (Exception e){
            System.out.println("Error in getting transform from tag");
            diffFromTag = new Transform3d();
            return;
        }
    }

    @Override
    public void execute() {
        dr_dt = 0;
        dx_dt = 0;
        dy_dt = 0;

        try{
            diffFromTag = vision.getTransformBestTarget();
        } catch (Exception e){
            System.out.println("Error in getting transform from tag");
            return;
        } 

        if (Math.abs(diffFromTag.getRotation().getZ()) > AutoAlignConstants.AutoAlignTolerance || 
            Math.abs(diffFromTag.getTranslation().getX()) > AutoAlignConstants.AutoAlignTolerance 
            || Math.abs(diffFromTag.getTranslation().getY()) > AutoAlignConstants.AutoAlignTolerance) {

                dy_dt = -controllerX.calculate(diffFromTag.getTranslation().getX());
                dx_dt = -controllerY.calculate(diffFromTag.getTranslation().getY()); // X and Y are intentionally flipped here
                dr_dt = controllerRot.calculate(diffFromTag.getRotation().getZ());
        }

        if (!DriverStation.isFMSAttached()){
            LightningShuffleboard.setDouble("TestAutoAlign", "X diffrence", 
                diffFromTag.getTranslation().getX());
            LightningShuffleboard.setDouble("TestAutoAlign", "Y diffrence", 
                diffFromTag.getTranslation().getY());
            LightningShuffleboard.setDouble("TestAutoAlign", "yaw diffrence", 
                diffFromTag.getRotation().getZ());

            LightningShuffleboard.setDouble("TestAutoAlign", "X speed", dx_dt);
            LightningShuffleboard.setDouble("TestAutoAlign", "Y speed", dy_dt);
            LightningShuffleboard.setDouble("TestAutoAlign", "yaw speed", dr_dt);
        }

        drivetrain.setControl(DriveRequests.getRobotCentric(dx_dt, dy_dt, dr_dt));
    }



    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(DriveRequests.getRobotCentric(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(diffFromTag.getRotation().getZ()) < AutoAlignConstants.AutoAlignTolerance && 
            Math.abs(diffFromTag.getTranslation().getX()) < AutoAlignConstants.AutoAlignTolerance && 
            Math.abs(diffFromTag.getTranslation().getY()) < AutoAlignConstants.AutoAlignTolerance;
    }
}
