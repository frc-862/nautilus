
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.Constants.FishingRodConstants.states;
import frc.robot.Constants.LEDConstants.LED_STATES;
import frc.robot.Constants.RobotMotors;
import frc.robot.Constants.TunerConstants;
import frc.robot.commands.SetRodState;
import frc.robot.commands.StandinCommands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FishingRod;
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
    private FishingRod rod;

    private XboxController driver;
    private XboxController copilot;

    @Override
    protected void initializeSubsystems() {
        drivetrain = TunerConstants.createDrivetrain();
        vision = new PhotonVision();
        logger = new Telemetry(TunerConstants.TritonTunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
        driver = new XboxController(ControllerConstants.DRIVER_CONTROLLER);
        copilot = new XboxController(ControllerConstants.COPILOT_CONTROLLER);

        //this is temporary
        if(Robot.isSimulation()) {
            elevator = new Elevator(RobotMotors.leftElevatorMotor, RobotMotors.rightElevatorMotor);
            wrist = new Wrist(RobotMotors.wristMotor);
            rod = new FishingRod(wrist, elevator);
        }

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

        new Trigger(() -> driver.getStartButton() && driver.getBackButton()).onTrue(
                new InstantCommand(() -> drivetrain.seedFieldCentric()));

                
        // // TODO: Remove Standin Command
        // new Trigger(() -> (elevator.isOnTarget() && wrist.isOnTarget()))
        //         .whileTrue(leds.enableState(LED_STATES.ROD_ON_TARGET));

        //sim stuff
        if(Robot.isSimulation()) {
            new Trigger(copilot::getLeftBumperButtonPressed).whileTrue(new InstantCommand((() -> wrist.setPower(-0.75)))).onFalse(new InstantCommand(wrist::stop));
            new Trigger(copilot::getRightBumperButton).whileTrue(new InstantCommand((() -> wrist.setPower(0.75)))).onFalse(new InstantCommand(wrist::stop));

            new Trigger(()-> copilot.getYButton()).whileTrue(new InstantCommand((() -> elevator.setPower(0.75)))).onFalse(new InstantCommand(elevator::stop));
            new Trigger(() -> copilot.getAButton()).whileTrue(new InstantCommand((() -> elevator.setPower(-0.75)))).onFalse(new InstantCommand(elevator::stop));
        }
    }

    @Override
    protected void initializeNamedCommands() {
        NamedCommands.registerCommand("AlgaeCollect",
                StandinCommands.moveAlgaeCollector().deadlineFor(leds.enableState(LED_STATES.ALGAE_COLLECT)));
        NamedCommands.registerCommand("IntakeCoral",
                StandinCommands.intakeCoral().deadlineFor(leds.enableState(LED_STATES.CORAL_COLLECT)));
        NamedCommands.registerCommand("ScoreCoral", 
                StandinCommands.scoreCoral().deadlineFor(leds.enableState(LED_STATES.CORAL_SCORE)));
        NamedCommands.registerCommand("MoveWrist",
                StandinCommands.moveWrist(1).deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
        if (Robot.isReal()){
                NamedCommands.registerCommand("ElevatorHome",
                        StandinCommands.elevatorStow().deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                NamedCommands.registerCommand("ElevatorL1",
                        StandinCommands.elevatorL1().deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                NamedCommands.registerCommand("ElevatorL2",
                        StandinCommands.elevatorL2().deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                NamedCommands.registerCommand("ElevatorL3",
                        StandinCommands.elevatorL3().deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                NamedCommands.registerCommand("ElevatorL4",
                        StandinCommands.elevatorL4().deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                NamedCommands.registerCommand("ElevatorSource",
                        StandinCommands.elevatorSource().deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
        } else if(Robot.isSimulation()){
                NamedCommands.registerCommand("ElevatorHome", new SetRodState(rod, states.STOW));
                NamedCommands.registerCommand("ElevatorL1", new SetRodState(rod, states.L1));
                NamedCommands.registerCommand("ElevatorL2", new SetRodState(rod, states.L2));
                NamedCommands.registerCommand("ElevatorL3", new SetRodState(rod, states.L3));
                NamedCommands.registerCommand("ElevatorL4", new SetRodState(rod, states.L4));
                NamedCommands.registerCommand("ElevatorSource", new SetRodState(rod, states.SOURCE));
        }

        autoChooser = AutoBuilder.buildAutoChooser();
        LightningShuffleboard.set("Auton", "Auto Chooser", autoChooser);
}

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
