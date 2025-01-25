package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class TestAutoAlign extends Command {
    PhotonVision vision;
    Swerve drivetrain;

    // get PID values from constants

    Transform3d diffFromTag;
    // PIDController controllerRot = RobotBase.isReal() ? new PIDController(AutoAlignConstants.RotAutoAlignKp, 
    //     AutoAlignConstants.RotAutoAlignKi, AutoAlignConstants.RotAutoAlignKd) : new PIDController(AutoAlignConstants.SimRotAutoAlignKp, 
    //     AutoAlignConstants.SimRotAutoAlignKi, AutoAlignConstants.SimRotAutoAlignKd);

    // PIDController controllerX = RobotBase.isReal() ? new PIDController(AutoAlignConstants.XAutoAlignKp, AutoAlignConstants.XAutoAlignKi, 
    //     AutoAlignConstants.XAutoAlignKd) : new PIDController(AutoAlignConstants.SimXAutoAlignKp, AutoAlignConstants.SimXAutoAlignKi,
    //     AutoAlignConstants.SimXAutoAlignKd);

    // PIDController controllerY = RobotBase.isReal() ? new PIDController(AutoAlignConstants.YAutoAlignKp, 
    //     AutoAlignConstants.YAutoAlignKi, AutoAlignConstants.YAutoAlignKd) : new PIDController(AutoAlignConstants.SimYAutoAlignKp,
    //     AutoAlignConstants.SimYAutoAlignKi, AutoAlignConstants.SimYAutoAlignKd);

    PIDController controllerX = new PIDController(
        LightningShuffleboard.getDouble("TestAutoAlign", "X Kp", 0), 
        LightningShuffleboard.getDouble("TestAutoAlign", "X Ki", 0), 
        LightningShuffleboard.getDouble("TestAutoAlign", "X Kd", 0));

    PIDController controllerY = new PIDController(
        LightningShuffleboard.getDouble("TestAutoAlign", "Y Kp", 0), 
        LightningShuffleboard.getDouble("TestAutoAlign", "Y Ki", 0), 
        LightningShuffleboard.getDouble("TestAutoAlign", "Y Kd", 0));

    double dr_dt;
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

        
        dx_dt = 0;
        dy_dt = 0;

        pitch = vision.getPitch();
        yaw = vision.getYaw();

        controllerX.setSetpoint(0);
        controllerX.setTolerance(AutoAlignConstants.AutoAlignTolerance);

        controllerY.setSetpoint(0);
        controllerY.setTolerance(AutoAlignConstants.AutoAlignTolerance);

    }

    @Override
    public void execute() {

        // zero velocity values
        dx_dt = 0;
        dy_dt = 0;

        pitch = vision.getPitch();
        yaw = vision.getYaw();

        if(!vision.hasTarget()){
            drivetrain.setControl(DriveRequests.getRobotCentric(0, 0, 0));
            return;
        }

        // if the difference is greater than the tolerance, calculate the new velocity values

        if (/*Math.abs(diffFromTag.getRotation().getZ()) > AutoAlignConstants.AutoAlignTolerance || 
            Math.abs(diffFromTag.getTranslation().getX()) > AutoAlignConstants.AutoAlignTolerance 
            || Math.abs(diffFromTag.getTranslation().getY()) > AutoAlignConstants.AutoAlignTolerance*/ true) {

                dx_dt = controllerX.calculate(yaw);
                dy_dt = -controllerY.calculate(pitch); // X and Y are intentionally flipped here
        }

        if (!DriverStation.isFMSAttached()){
            LightningShuffleboard.setDouble("TestAutoAlign", "X diffrence", 
                yaw);
            LightningShuffleboard.setDouble("TestAutoAlign", "Y diffrence", 
                pitch);

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

        return false;

        // return Math.abs(diffFromTag.getRotation().getZ()) < AutoAlignConstants.AutoAlignTolerance && 
        //     Math.abs(diffFromTag.getTranslation().getX()) < AutoAlignConstants.AutoAlignTolerance && 
        //     Math.abs(diffFromTag.getTranslation().getY()) < AutoAlignConstants.AutoAlignTolerance;
    }
}
