package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class TestAutoAlign extends Command {
    PhotonVision vision;
    Swerve drivetrain;

    Transform3d diffFromTag;
    PIDController controllerRot = new PIDController(0.5d, 0, 0);
    PIDController controllerX = new PIDController(0.5d, 0, 0);
    PIDController controllerY = new PIDController(0.5d, 0, 0);

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
        controllerRot.setTolerance(0.2d);

        controllerX.setSetpoint(0);
        controllerX.setTolerance(0.2d);

        controllerY.setSetpoint(0);
        controllerY.setTolerance(0.2d);

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
        try{
            diffFromTag = vision.getTransformBestTarget();
        } catch (Exception e){
            System.out.println("Error in getting transform from tag");
            return;
        } 
              

        if (Math.abs(diffFromTag.getRotation().getZ()) > 0.2d || 
            Math.abs(diffFromTag.getTranslation().getX()) > 0.2d 
            || Math.abs(diffFromTag.getTranslation().getY()) > 0.2d) {

                dx_dt = -controllerX.calculate(diffFromTag.getTranslation().getX());
                dy_dt = -controllerY.calculate(diffFromTag.getTranslation().getY());
                dr_dt = -controllerRot.calculate(diffFromTag.getRotation().getZ());
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

            LightningShuffleboard.setBool("TestAutoAlign", "HasTarget", vision.hasTarget());

        }

        drivetrain.setControl(DriveRequests.getDrive(dx_dt, dy_dt, dr_dt));
        // drivetrain.setControl(DriveRequests.getDrive(0.7, 0.7, 0.7));

        System.out.println("CodeIsRunning" + "1:57");
    }



    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        // return Math.abs(diffFromTag.getRotation().getZ()) > 0.2d && 
        //     Math.abs(diffFromTag.getTranslation().getX()) > 0.2d && 
        //     Math.abs(diffFromTag.getTranslation().getY()) > 0.2d;
        return false;
    }
}
