
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FishingRodConstants;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.Constants.FishingRodConstants.states;
import frc.robot.Constants.LEDConstants.LED_STATES;
import frc.robot.Constants.RobotMotors;
import frc.robot.Constants.TunerConstants;
import frc.robot.commands.SetRodState;
import frc.robot.commands.SmartCoralCollect;
import frc.robot.commands.StandinCommands;
import frc.robot.subsystems.Collector;
import frc.robot.commands.TestAutoAlign;
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
    private Collector collector;

    private XboxController driver;
    private XboxController copilot;

    @Override
    protected void initializeSubsystems() {
        copilot = new XboxController(ControllerConstants.COPILOT_CONTROLLER);
        driver = new XboxController(ControllerConstants.DRIVER_CONTROLLER);
        drivetrain = TunerConstants.createDrivetrain();
        vision = new PhotonVision();
        logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
        elevator = new Elevator(RobotMotors.leftElevatorMotor, RobotMotors.rightElevatorMotor);
        collector = new Collector(RobotMotors.collectorMotor);
        wrist = new Wrist(RobotMotors.wristMotor);
        rod = new FishingRod(wrist, elevator);

        //this is temporary
        if (Robot.isSimulation()) {
        }

        leds = new LEDs();
    }

    @Override
    protected void configureDefaultCommands() {
        drivetrain.setDefaultCommand(drivetrain.applyRequest(
                DriveRequests.getDrive(() -> MathUtil.applyDeadband(-driver.getLeftX(), ControllerConstants.JOYSTICK_DEADBAND), () -> MathUtil.applyDeadband(-driver.getLeftY(), ControllerConstants.JOYSTICK_DEADBAND), () -> MathUtil.applyDeadband(-driver.getRightX(), ControllerConstants.JOYSTICK_DEADBAND))));
        drivetrain.registerTelemetry(logger::telemeterize);

        vision.setDefaultCommand(vision.updateOdometry(drivetrain));

        collector.setDefaultCommand(new SmartCoralCollect(collector, () -> copilot.getRightTriggerAxis() - copilot.getLeftTriggerAxis()));

        // rod.setDefaultCommand(new InstantCommand(() -> rod.setState(states.STOW), rod));
    }

    @Override
    protected void configureButtonBindings() {
        new Trigger(() -> driver.getRightTriggerAxis() > 0.25).whileTrue(drivetrain.applyRequest(
                DriveRequests.getSlow(() -> MathUtil.applyDeadband(-driver.getLeftX(), ControllerConstants.JOYSTICK_DEADBAND), () -> MathUtil.applyDeadband(-driver.getLeftY(), ControllerConstants.JOYSTICK_DEADBAND), () -> MathUtil.applyDeadband(-driver.getRightX(), ControllerConstants.JOYSTICK_DEADBAND))));
        new Trigger(() -> driver.getLeftTriggerAxis() > 0.25).whileTrue(drivetrain.applyRequest(DriveRequests
                .getRobotCentric(() -> MathUtil.applyDeadband(-driver.getLeftX(), ControllerConstants.JOYSTICK_DEADBAND), () -> MathUtil.applyDeadband(-driver.getLeftY(), ControllerConstants.JOYSTICK_DEADBAND), () -> MathUtil.applyDeadband(-driver.getRightX(), ControllerConstants.JOYSTICK_DEADBAND))));
        new Trigger(driver::getXButton).whileTrue(drivetrain.applyRequest(DriveRequests.getBrake()));

        new Trigger(() -> driver.getStartButton() && driver.getBackButton()).onTrue(new InstantCommand(() -> drivetrain.seedFieldCentric()));

        new Trigger(() -> copilot.getAButton()).whileTrue(new RunCommand(() -> rod.setState(states.L1), rod));
        new Trigger(()-> copilot.getYButton()).whileTrue(new RunCommand(() -> rod.setState(states.L4), rod));

        new Trigger(()-> copilot.getXButton()).whileTrue(new RunCommand(() -> rod.setState(states.L2), rod));
        new Trigger(() -> copilot.getBButton()).whileTrue(new RunCommand(() -> rod.setState(states.L3), rod));

        new Trigger(() -> copilot.getRightBumperButton()).whileTrue(new RunCommand((() -> rod.setState(states.SOURCE)), rod));
        new Trigger(() -> copilot.getLeftBumperButton()).whileTrue(new RunCommand((() -> rod.setState(states.STOW)), rod));

        
        
        // new Trigger(() -> copilot.getAButton()).whileTrue(new InstantCommand(() -> elevator.setPosition(0)));
        // new Trigger(()-> copilot.getYButton()).whileTrue(new InstantCommand(() -> elevator.setPosition(13)));

        // new Trigger(()-> copilot.getXButton()).whileTrue(new InstantCommand(() -> elevator.setPosition(26)));
        // new Trigger(() -> copilot.getBButton()).whileTrue(new InstantCommand(() -> elevator.setPosition(43)));
        
        new Trigger(() -> copilot.getLeftY() > 0.2).onTrue(rod.addElevatorBias(5));
        new Trigger(() -> copilot.getLeftY() < -0.2).onTrue(rod.addElevatorBias(-5));
        
        new Trigger(() -> copilot.getRightY() < -0.2).onTrue(rod.addWristBias(5));
        new Trigger(() -> copilot.getRightY() > 0.2).onTrue(rod.addWristBias(-5));
        
        // new Trigger(() -> copilot.getRightBumperButton()).whileTrue(new InstantCommand((() -> wrist.setPosition(-20))));
        // new Trigger(()-> copilot.getLeftBumperButton()).whileTrue(new InstantCommand((() -> wrist.setPosition(-80))));
        // new Trigger(() -> copilot.getStartButton()).onTrue(new InstantCommand(() -> wrist.setPosition(-57)));


        // new Trigger(()-> copilot.getXButton()).whileTrue(new InstantCommand((() -> wrist.setPosition(0))));
        // new Trigger(() -> copilot.getBButton()).whileTrue(new InstantCommand((() -> wrist.setPosition(40))));
        

        // // TODO: Remove Standin Command
        // new Trigger(() -> rod.onTarget()).whileTrue(leds.enableState(LED_STATES.ROD_ON_TARGET));

        //sim stuff
        if(Robot.isSimulation()) {
            // new Trigger(copilot::getLeftBumperButton).whileTrue(new InstantCommand((() -> wrist.setPower(-1)))).onFalse(new InstantCommand(wrist::stop));
            // new Trigger(copilot::getRightBumperButton).whileTrue(new InstantCommand((() -> wrist.setPower(1)))).onFalse(new InstantCommand(wrist::stop));

        //     new Trigger(()-> copilot.getYButton()).whileTrue(new InstantCommand((() -> elevator.setPower(0.75)))).onFalse(new InstantCommand(elevator::stop));
        //     new Trigger(() -> copilot.getAButton()).whileTrue(new InstantCommand((() -> elevator.setPower(-0.75)))).onFalse(new InstantCommand(elevator::stop));

            // new Trigger (()-> copilot.getXButton()).whileTrue(new InstantCommand((() -> collector.setPower(0.75)))).onFalse(new InstantCommand(collector::stop));
            // new Trigger(() -> copilot.getBButton()).whileTrue(new InstantCommand((() -> collector.setPower(-0.5)))).onFalse(new InstantCommand(collector::stop));
        }
    }

    @Override
    protected void initializeNamedCommands() {
        NamedCommands.registerCommand("IntakeCoral",
                StandinCommands.intakeCoral().deadlineFor(leds.enableState(LED_STATES.CORAL_COLLECT)));
        NamedCommands.registerCommand("ScoreCoral", 
                StandinCommands.scoreCoral().deadlineFor(leds.enableState(LED_STATES.CORAL_SCORE)));
        // TODO: Get actual offsets
        NamedCommands.registerCommand("ReefAlignLeft", 
                new TestAutoAlign(vision, drivetrain, 0).deadlineFor(leds.enableState(LED_STATES.ALIGNING)));
        NamedCommands.registerCommand("ReefAlignRight", 
                new TestAutoAlign(vision, drivetrain, 0).deadlineFor(leds.enableState(LED_STATES.ALIGNING)));
        NamedCommands.registerCommand("SourceAlignLeft", 
                new TestAutoAlign(vision, drivetrain, 0).deadlineFor(leds.enableState(LED_STATES.ALIGNING)));
        NamedCommands.registerCommand("SourceAlignRight", 
                new TestAutoAlign(vision, drivetrain, 0).deadlineFor(leds.enableState(LED_STATES.ALIGNING)));
        if (Robot.isReal()){
                NamedCommands.registerCommand("RodHome",
                        StandinCommands.rodStow().deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                NamedCommands.registerCommand("RodL1",
                        StandinCommands.rodL1().deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                NamedCommands.registerCommand("RodL2",
                        StandinCommands.rodL2().deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                NamedCommands.registerCommand("RodL3",
                        StandinCommands.rodL3().deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                NamedCommands.registerCommand("RodL4",
                        StandinCommands.rodL4().deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                NamedCommands.registerCommand("RodSource",
                        StandinCommands.rodSource().deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
        } else if(Robot.isSimulation()){
                NamedCommands.registerCommand("RodHome", 
                        new SetRodState(rod, states.STOW).deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                NamedCommands.registerCommand("RodL1", 
                        new SetRodState(rod, states.L1).deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                NamedCommands.registerCommand("RodL2", 
                        new SetRodState(rod, states.L2).deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                NamedCommands.registerCommand("RodL3", 
                        new SetRodState(rod, states.L3).deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                NamedCommands.registerCommand("RodL4", 
                        new SetRodState(rod, states.L4).deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
                NamedCommands.registerCommand("RodSource", 
                        new SetRodState(rod, states.SOURCE).deadlineFor(leds.enableState(LED_STATES.ROD_MOVING)));
        }

        autoChooser = AutoBuilder.buildAutoChooser();
        LightningShuffleboard.set("Auton", "Auto Chooser", autoChooser);
}

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
