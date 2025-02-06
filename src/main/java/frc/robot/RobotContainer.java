
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FishingRodConstants;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.Constants.FishingRodConstants.ROD_STATES;
import frc.robot.Constants.LEDConstants.LED_STATES;
import frc.robot.Constants.RobotMotors;
import frc.robot.Constants.TunerConstants;
import frc.robot.commands.CollectAlgae;
import frc.robot.commands.CollectCoral;
import frc.robot.commands.SetRodState;
import frc.robot.commands.StandinCommands;
import frc.robot.commands.TagAutoAlign;
import frc.robot.subsystems.AlgaeCollector;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralCollector;
import frc.robot.commands.auton.ScoreCoral;
import frc.robot.commands.TagAutoAlign;
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
    private CoralCollector coralCollector;
    private AlgaeCollector algaeCollector;
    private Climber climber;

    private Trigger algaeMode;

    private static XboxController driver;
    private static XboxController copilot;

    @Override
    protected void initializeSubsystems() {
        copilot = new XboxController(ControllerConstants.COPILOT_CONTROLLER);
        driver = new XboxController(ControllerConstants.DRIVER_CONTROLLER);
        drivetrain = TunerConstants.createDrivetrain();
        vision = new PhotonVision(drivetrain);
        logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

        driver = new XboxController(ControllerConstants.DRIVER_CONTROLLER);
        copilot = new XboxController(ControllerConstants.COPILOT_CONTROLLER);
        leds = new LEDs();
        elevator = new Elevator(RobotMotors.leftElevatorMotor, RobotMotors.rightElevatorMotor);
        wrist = new Wrist(RobotMotors.wristMotor);
        rod = new FishingRod(wrist, elevator);
        coralCollector = new CoralCollector(RobotMotors.coralCollectorMotor);
        climber = new Climber(RobotMotors.climberMotor);
        // algaeMode = new Trigger(() -> driver.getPOV() == 180 || copilot.getStartButtonPressed());

        switch (Constants.ROBOT_MODE) {
            case NAUTILUS:
                // nothing
                break;
            default: // (sim or> triton)
               
                break;
        }
    }

    @Override
    protected void configureDefaultCommands() {
        drivetrain.setDefaultCommand(drivetrain.applyRequest(
                DriveRequests.getDrive(
                        () -> MathUtil.applyDeadband(-driver.getLeftX(),
                                ControllerConstants.JOYSTICK_DEADBAND),
                        () -> MathUtil.applyDeadband(-driver.getLeftY(),
                                ControllerConstants.JOYSTICK_DEADBAND),
                        () -> MathUtil.applyDeadband(-driver.getRightX(),
                                ControllerConstants.JOYSTICK_DEADBAND))));
        drivetrain.registerTelemetry(logger::telemeterize);


        coralCollector.setDefaultCommand(new CollectCoral(coralCollector,
                () -> copilot.getRightTriggerAxis() - copilot.getLeftTriggerAxis()));

        // climber.setDefaultCommand(new RunCommand(() -> climber.setPower(MathUtil.applyDeadband(-copilot.getLeftY(), ControllerConstants.JOYSTICK_DEADBAND)), climber));

        switch (Constants.ROBOT_MODE) {
            case NAUTILUS:
                // nothing
                break;
            default:
                // stow
                rod.setDefaultCommand(new SetRodState(rod, ROD_STATES.STOW));
                break;
        }
    }

    @Override
    protected void configureButtonBindings() {
        new Trigger(() -> driver.getLeftTriggerAxis() > 0.25).whileTrue(drivetrain.applyRequest(DriveRequests
                .getRobotCentric(
                        () -> MathUtil.applyDeadband(-driver.getLeftX(),
                                ControllerConstants.JOYSTICK_DEADBAND),
                        () -> MathUtil.applyDeadband(-driver.getLeftY(),
                                ControllerConstants.JOYSTICK_DEADBAND),
                        () -> MathUtil.applyDeadband(-driver.getRightX(),
                                ControllerConstants.JOYSTICK_DEADBAND))));
        // sets slow mode
        new Trigger(() -> driver.getRightTriggerAxis() > 0.25)
                .onTrue(new InstantCommand(() -> drivetrain.setSlowMode(true)))
                .onFalse(new InstantCommand(() -> drivetrain.setSlowMode(false)));


        new Trigger(driver::getXButton).whileTrue(drivetrain.applyRequest(DriveRequests.getBrake()));

        new Trigger(() -> driver.getStartButton() && driver.getBackButton()).onTrue(
                new InstantCommand(() -> drivetrain.seedFieldCentric()));
                
        // // TODO: Remove Standin Command
        // new Trigger(() -> rod.onTarget()).whileTrue(leds.enableState(LED_STATES.ROD_ON_TARGET));

        //sim stuff
        if(Robot.isSimulation()) {
            // new Trigger(copilot::getLeftBumperButton).whileTrue(new InstantCommand((() -> wrist.setPower(-1)))).onFalse(new InstantCommand(wrist::stop));
            // new Trigger(copilot::getRightBumperButton).whileTrue(new InstantCommand((() -> wrist.setPower(1)))).onFalse(new InstantCommand(wrist::stop));

        //     new Trigger(()-> copilot.getYButton()).whileTrue(new InstantCommand((() -> elevator.setPower(0.75)))).onFalse(new InstantCommand(elevator::stop));
        //     new Trigger(() -> copilot.getAButton()).whileTrue(new InstantCommand((() -> elevator.setPower(-0.75)))).onFalse(new InstantCommand(elevator::stop));

            // new Trigger (()-> copilot.getXButton()).whileTrue(new InstantCommand((() -> coralCollector.setPower(0.75)))).onFalse(new InstantCommand(coralCollector::stop));
            // new Trigger(() -> copilot.getBButton()).whileTrue(new InstantCommand((() -> coralCollector.setPower(-0.5)))).onFalse(new InstantCommand(coralCollector::stop));
        }
    }

    @Override
    protected void initializeNamedCommands() {
        NamedCommands.registerCommand("IntakeCoral",
                StandinCommands.intakeCoral().deadlineFor(leds.enableState(LED_STATES.CORAL_COLLECT)));
        // NamedCommands.registerCommand("ScoreCoral",
        //         StandinCommands.scoreCoral().deadlineFor(leds.enableState(LED_STATES.CORAL_SCORE)));
        // TODO: Get actual offsets
        NamedCommands.registerCommand("ReefAlignLeft", 
                new TagAutoAlign(vision, drivetrain).deadlineFor(leds.enableState(LED_STATES.ALIGNING)));
        NamedCommands.registerCommand("ReefAlignRight", 
                new TagAutoAlign(vision, drivetrain).deadlineFor(leds.enableState(LED_STATES.ALIGNING)));
        NamedCommands.registerCommand("SourceAlignLeft", 
                new TagAutoAlign(vision, drivetrain).deadlineFor(leds.enableState(LED_STATES.ALIGNING)));
        NamedCommands.registerCommand("SourceAlignRight", 
                new TagAutoAlign(vision, drivetrain).deadlineFor(leds.enableState(LED_STATES.ALIGNING)));
        switch(Constants.ROBOT_MODE) {
                case SIM:
                        NamedCommands.registerCommand("RodHome",
                                StandinCommands.rodStow()
                                        .deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                        NamedCommands.registerCommand("RodL1",
                                StandinCommands.rodL1()
                                        .deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                        NamedCommands.registerCommand("RodL2",
                                StandinCommands.rodL2()
                                        .deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                        NamedCommands.registerCommand("RodL3",
                                StandinCommands.rodL3()
                                        .deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                        NamedCommands.registerCommand("RodL4",
                                StandinCommands.rodL4()
                                        .deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                        NamedCommands.registerCommand("RodSource",
                                StandinCommands.rodSource()
                                        .deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                break;
                default:
                        NamedCommands.registerCommand("RodHome",
                                new SetRodState(rod, ROD_STATES.STOW)
                                        .deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                        NamedCommands.registerCommand("RodL1",
                                new SetRodState(rod, ROD_STATES.L1)
                                        .deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                        NamedCommands.registerCommand("RodL2",
                                new SetRodState(rod, ROD_STATES.L2)
                                        .deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                        NamedCommands.registerCommand("RodL3",
                                new SetRodState(rod, ROD_STATES.L3)
                                        .deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                        NamedCommands.registerCommand("RodL4",
                                new SetRodState(rod, ROD_STATES.L4)
                                        .deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                        NamedCommands.registerCommand("RodSource",
                                new SetRodState(rod, ROD_STATES.SOURCE)
                                        .deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                        NamedCommands.registerCommand("ScoreCoral", new ScoreCoral(coralCollector));
                break;
        }

        autoChooser = AutoBuilder.buildAutoChooser();
        LightningShuffleboard.set("Auton", "Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public static Command hapticDriverCommand() {
        if (!DriverStation.isAutonomous()) {
            return new StartEndCommand(() -> {
                driver.setRumble(GenericHID.RumbleType.kRightRumble, 1d);
                driver.setRumble(GenericHID.RumbleType.kLeftRumble, 1d);
            }, () -> {
                driver.setRumble(GenericHID.RumbleType.kRightRumble, 0);
                driver.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            });
        } else {
            return new InstantCommand();
        }
    }

    public static Command hapticCopilotCommand() {
        if (!DriverStation.isAutonomous()) {
            return new StartEndCommand(() -> {
                copilot.setRumble(GenericHID.RumbleType.kRightRumble, 1d);
                copilot.setRumble(GenericHID.RumbleType.kLeftRumble, 1d);
            }, () -> {
                copilot.setRumble(GenericHID.RumbleType.kRightRumble, 0);
                copilot.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            });
        } else {
            return new InstantCommand();
        }
    }

}
