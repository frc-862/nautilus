
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.Constants.LEDConstants.LED_STATES;
import frc.robot.Constants.TunerConstants;
import frc.robot.commands.StandinCommands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.thunder.LightningContainer;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class RobotContainer extends LightningContainer {

    public Swerve drivetrain;
    public PhotonVision vision;
    private Telemetry logger;
    private LEDs leds;
    private SendableChooser<Command> autoChooser;

    private Elevator elevator;
    private Wrist wrist;

    private XboxController driver;

    @Override
    protected void initializeSubsystems() {
        drivetrain = TunerConstants.createDrivetrain();
        vision = new PhotonVision();
        logger = new Telemetry(TunerConstants.TritonTunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
        driver = new XboxController(ControllerConstants.DRIVER_CONTROLLER);

        leds = new LEDs();
    }

    @Override
    protected void configureDefaultCommands() {
        drivetrain.setDefaultCommand(drivetrain.applyRequest(
                DriveRequests.getDrive(() -> -driver.getLeftX(), () -> -driver.getLeftY(), () -> driver.getRightX())));
        drivetrain.registerTelemetry(logger::telemeterize);

        vision.setDefaultCommand(vision.updateOdometry(drivetrain));
    }

    @Override
    protected void configureButtonBindings() {
        new Trigger(driver::getAButton).whileTrue(drivetrain.applyRequest(
                DriveRequests.getSlow(() -> -driver.getLeftX(), () -> -driver.getLeftY(), () -> driver.getRightX())));
        new Trigger(driver::getBButton).whileTrue(drivetrain.applyRequest(DriveRequests
                .getRobotCentric(() -> -driver.getLeftX(), () -> -driver.getLeftY(), () -> driver.getRightX())));
        new Trigger(driver::getXButton).whileTrue(drivetrain.applyRequest(DriveRequests.getBrake()));

        new Trigger(() -> driver.getStartButton() && driver.getBackButton()).onTrue(drivetrain
				.runOnce(drivetrain::seedFieldCentric).andThen(new InstantCommand(() -> drivetrain
						.setOperatorPerspectiveForward(new Rotation2d(Math.toRadians(0))))));

        // // TODO: Remove Standin Command
        // new Trigger(() -> (elevator.isOnTarget() && wrist.isOnTarget()))
        //         .whileTrue(leds.enableState(LED_STATES.ROD_ON_TARGET));

    }

    @Override
    protected void initializeNamedCommands() {

        NamedCommands.registerCommand("ElevatorHome",
                StandinCommands.moveElevator(1).withDeadline(leds.enableState(LED_STATES.ROD_MOVING)));
        NamedCommands.registerCommand("AlgaeCollect",
                StandinCommands.moveAlgaeCollector().withDeadline(leds.enableState(LED_STATES.ALEGE_COLLECT)));
        NamedCommands.registerCommand("IntakeCoral",
                StandinCommands.intakeCoral().withDeadline(leds.enableState(LED_STATES.CORAL_COLLECT)));
        NamedCommands.registerCommand("MoveWrist",
                StandinCommands.moveWrist(1).withDeadline(leds.enableState(LED_STATES.ROD_MOVING)));

        autoChooser = AutoBuilder.buildAutoChooser();
        LightningShuffleboard.set("Auton", "Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
