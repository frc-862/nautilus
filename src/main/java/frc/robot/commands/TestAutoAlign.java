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
    PIDController controllerRot = new PIDController(0.05d, 0, 0);
    PIDController controllerXY = new PIDController(0.05d, 0, 0);

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
        controllerRot.setSetpoint(0);
        controllerRot.setTolerance(0.2d);

        controllerXY.setSetpoint(0);
        controllerXY.setTolerance(0.2d);
    }

    @Override
    public void execute() {
        diffFromTag = vision.getTransformBestTarget();
        if (Math.abs(diffFromTag.getRotation().getZ()) > 0.2d) {
            dr_dt = controllerRot.calculate(diffFromTag.getRotation().getZ());
        } 

        if (Math.abs(diffFromTag.getTranslation().getX()) > 0.2d 
            || Math.abs(diffFromTag.getTranslation().getY()) > 0.2d) {

                dx_dt = controllerXY.calculate(diffFromTag.getTranslation().getX());
                dy_dt = controllerXY.calculate(diffFromTag.getTranslation().getY());
        }

        drivetrain.applyRequest(DriveRequests.getRobotCentric(() -> dx_dt, () -> dy_dt, () -> dr_dt));

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

    }



    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return Math.abs(diffFromTag.getRotation().getZ()) > 0.2d && 
            Math.abs(diffFromTag.getTranslation().getX()) > 0.2d && 
            Math.abs(diffFromTag.getTranslation().getY()) > 0.2d;
    }
}
