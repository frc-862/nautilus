package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class TestAutoAlign extends Command {
    PhotonVision vision;
    Swerve drivetrain;

    double tx = 0;
    PIDController controller = new PIDController(0.05d, 0, 0);
    double calculatedSpeed;

    public TestAutoAlign(PhotonVision vision, Swerve drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        controller.setSetpoint(0);
        controller.setTolerance(0.2d);
    }

    @Override
    public void execute() {
        tx = vision.getXBestTarget();
        if (Math.abs(tx) > 0.2d) {
            calculatedSpeed = controller.calculate(tx);
            drivetrain.applyRequest(DriveRequests.getRobotCentric(() -> calculatedSpeed, () -> 0, () -> 0));
        } else {
            DataLogManager.log("Auto Align Finished");

        }

        if (!DriverStation.isFMSAttached()){
            LightningShuffleboard.setDouble("TestAutoAlign", "tX", tx);
            LightningShuffleboard.setDouble("TestAutoAlign", "calculatedspeed", calculatedSpeed);
        }

    }



    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return Math.abs(tx) > 0.2d;
    }
}
