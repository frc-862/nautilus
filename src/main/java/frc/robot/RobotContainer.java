
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.CoralCollectorConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.RobotIdentifiers;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.Constants.FishingRodConstants.RodStates;
import frc.robot.Constants.LEDConstants.LEDStates;
import frc.robot.Constants.PoseConstants.LightningTagID;
import frc.robot.Constants.RobotMotors;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.VisionConstants.Camera;
import frc.robot.Constants.AlgaeCollectorConstants.AlgaePivotStates;
import frc.robot.Constants.LEDConstants.LEDStates;
import frc.robot.commands.CollectAlgae;
import frc.robot.commands.CollectCoral;
import frc.robot.commands.PoseBasedAutoAlign;
import frc.robot.commands.SetRodState;
import frc.robot.commands.SysIdSequence;
import frc.robot.commands.TagAutoAlign;
import frc.robot.commands.ThreeDeeAutoAlign;
import frc.robot.commands.auton.IntakeCoral;
import frc.robot.commands.auton.ScoreCoral;
import frc.robot.subsystems.AlgaeCollector;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralCollector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FishingRod;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SimGamePeices;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.thunder.LightningContainer;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.thunder.filter.XboxControllerFilter;
import frc.thunder.leds.LightningColors;

public class RobotContainer extends LightningContainer {

    public PowerDistribution pdh;
    public boolean allowPDHLeds = true;

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

    private SimGamePeices simGamePeices;

    private static XboxController driver;
    private static XboxController copilot;

    private boolean erroring = false;

    @Override
    protected void initializeSubsystems() {
        pdh = new PowerDistribution(RobotMap.PDH, RobotMap.PDH_MODULE_TYPE);

        driver = new XboxController(ControllerConstants.DRIVER_CONTROLLER);
        copilot = new XboxController(ControllerConstants.COPILOT_CONTROLLER);

        drivetrain = TunerConstants.createDrivetrain();
        vision = new PhotonVision(drivetrain);
        logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

        leds = new LEDs();

        elevator = new Elevator(RobotMotors.leftElevatorMotor, RobotMotors.rightElevatorMotor);
        wrist = new Wrist(RobotMotors.wristMotor);
        rod = new FishingRod(wrist, elevator);
        coralCollector = new CoralCollector(RobotMotors.coralCollectorMotor);

        climber = new Climber(RobotMotors.climberMotor);

        if (Constants.ROBOT_IDENTIFIER != RobotIdentifiers.NAUTILUS) {
            algaeCollector = new AlgaeCollector(RobotMotors.algaeCollectorRollerMotor,
                    RobotMotors.algaeCollectorPivotMotor);
        }

        if (Robot.isSimulation()) {
            // algae collector and climber are temp because not initialized above
            algaeCollector = new AlgaeCollector(RobotMotors.algaeCollectorRollerMotor,
                    RobotMotors.algaeCollectorPivotMotor);

            simGamePeices = new SimGamePeices(elevator, wrist, drivetrain, coralCollector, algaeCollector, climber);
        }
    }

    @Override
    protected void configureDefaultCommands() {
        // FIELD CENTRIC DRIVE
        drivetrain.setDefaultCommand(drivetrain.applyRequest(
                DriveRequests.getDrive(
                        () -> MathUtil.applyDeadband(-(driver.getLeftY() * drivetrain.getSpeedMult()),
                                ControllerConstants.JOYSTICK_DEADBAND),
                        () -> MathUtil.applyDeadband(-(driver.getLeftX() * drivetrain.getSpeedMult()),
                                ControllerConstants.JOYSTICK_DEADBAND),
                        () -> MathUtil.applyDeadband(-(driver.getRightX() * drivetrain.getTurnMult()),
                                ControllerConstants.JOYSTICK_DEADBAND))));
        drivetrain.registerTelemetry(logger::telemeterize);

        // CORAL INTAKE
        coralCollector.setDefaultCommand(new CollectCoral(coralCollector, leds,
                () -> MathUtil.applyDeadband(copilot.getRightTriggerAxis() - copilot.getLeftTriggerAxis(),
                        CoralCollectorConstants.COLLECTOR_DEADBAND)));

        climber.setDefaultCommand(new RunCommand(
                () -> climber.setPower(
                        MathUtil.applyDeadband(-copilot.getLeftY(), ControllerConstants.JOYSTICK_DEADBAND)),
                climber));

        // This should not be here, but is commented just to be safe if ever needed
        // again
        // rod.setDefaultCommand(new SetRodState(rod,
        // RodStates.STOW).onlyIf(DriverStation::isTeleop));

        // LED TRIGGERS
        new Trigger(() -> rod.onTarget()).whileFalse(leds.strip.enableState(LEDStates.ROD_MOVING));

        new Trigger(() -> elevator.isOverheating()).onTrue(new InstantCommand(() -> erroring = true));

        new Trigger(() -> erroring && DriverStation.isDisabled()).whileTrue(leds.strip.enableState(LEDStates.ERROR));

        new Trigger(() -> (coralCollector.getVelocity() > 0)).whileTrue(leds.strip.enableState(LEDStates.SCORING));
        new Trigger(() -> (coralCollector.getVelocity() < 0)).whileTrue(leds.strip.enableState(LEDStates.COLLECTING));

        new Trigger(() -> (drivetrain.poseZero() && DriverStation.isDisabled() && !vision.hasTarget()))
                .whileTrue(leds.strip.enableState(LEDStates.POSE_BAD));
        new Trigger(() -> (!drivetrain.poseStable() && DriverStation.isDisabled() && vision.hasTarget()))
                .whileTrue(leds.strip.enableState(LEDStates.UPDATING_POSE));

        /* PDH LED TRIGGERSS */
        if (Constants.ROBOT_IDENTIFIER == RobotIdentifiers.NAUTILUS) {
            // Allow the Rio userbutton to toggle the pdh leds
            new Trigger(RobotController::getUserButton).onTrue(new InstantCommand(() -> {
                allowPDHLeds = !allowPDHLeds;
                pdh.setSwitchableChannel(allowPDHLeds);
            }));

            // Turn off the PDH leds if the voltage ever dips below a certain value
            new Trigger(() -> pdh.getVoltage() < LEDConstants.PDH_LED_POWEROFF_VOLTAGE)
                    .onTrue(new InstantCommand(() -> {
                        allowPDHLeds = false;
                        pdh.setSwitchableChannel(false);
                    }));
        }
    }

    @Override
    protected void configureButtonBindings() {
        /* DRIVER BINDINGS */
        // robot centric driving
        new Trigger(() -> driver.getLeftTriggerAxis() > 0.25).whileTrue(drivetrain.applyRequest(DriveRequests
                .getRobotCentric(
                        () -> MathUtil.applyDeadband(-(driver.getLeftX() * drivetrain.getSpeedMult()),
                                ControllerConstants.JOYSTICK_DEADBAND),
                        () -> MathUtil.applyDeadband(-(driver.getLeftY() * drivetrain.getSpeedMult()),
                                ControllerConstants.JOYSTICK_DEADBAND),
                        () -> MathUtil.applyDeadband(-(driver.getRightX() * drivetrain.getTurnMult()),
                                ControllerConstants.JOYSTICK_DEADBAND))));

        // sets slow mode
        new Trigger(() -> driver.getRightTriggerAxis() > 0.25)
                .onTrue(new InstantCommand(() -> drivetrain.setSlowMode(true)))
                .onFalse(new InstantCommand(() -> drivetrain.setSlowMode(false)));

        // // sets slow mode if the elevator is above L3 (around 29 inches)
        // new Trigger(() -> (elevator.getPosition() >
        // ElevatorConstants.SLOW_MODE_HEIGHT_LIMIT) && (DriverStation.isTeleop()))
        // .onTrue(new InstantCommand(() -> drivetrain.setSlowMode(true)));

        // // stops slow mode if below L3 (around 29 inches)
        // new Trigger(() -> (!(elevator.getPosition() >
        // ElevatorConstants.SLOW_MODE_HEIGHT_LIMIT) && !(driver.getRightTriggerAxis() >
        // 0.25) && (DriverStation.isTeleop())))
        // .onTrue(new InstantCommand(() -> drivetrain.setSlowMode(false)));

        // drivetrain brake
        new Trigger(driver::getXButton).whileTrue(drivetrain.applyRequest(DriveRequests.getBrake()));

        // reset forward
        new Trigger(() -> driver.getStartButton() && driver.getBackButton()).onTrue(
                new InstantCommand(drivetrain::seedFieldCentric).alongWith(new InstantCommand(() -> drivetrain.setOperatorPerspectiveForward(new Rotation2d(Degrees.of(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 180 : 0))))));// (new
                                                                  // Rotation2d(Degrees.of(DriverStation.getAlliance().get()
                                                                  // == Alliance.Red ? 180 : 0)))));

        // new Trigger(driver::getLeftBumperButton)
        // .whileTrue(new PoseBasedAutoAlign(vision, drivetrain, Camera.RIGHT));

        new Trigger(driver::getLeftBumperButton)
                .whileTrue(new PoseBasedAutoAlign(vision, drivetrain, Camera.RIGHT, leds));
        new Trigger(driver::getRightBumperButton)
                .whileTrue(new PoseBasedAutoAlign(vision, drivetrain, Camera.LEFT, leds));

        new Trigger(() -> driver.getPOV() == 90)
                .whileTrue(new PoseBasedAutoAlign(vision, drivetrain, Camera.RIGHT, leds, LightningTagID.LeftSource));



        /* COPILOT BINDINGS */
        new Trigger(copilot::getRightBumperButton)
                .onTrue(new SetRodState(rod, RodStates.SOURCE));
        new Trigger(copilot::getLeftBumperButton)
                .onTrue(new SetRodState(rod, RodStates.STOW));


        new Trigger(copilot::getBackButton)
                .onTrue(new InstantCommand(() -> rod.setCoralMode(false)));
        new Trigger(copilot::getStartButton)
                .onTrue(new InstantCommand(() -> rod.setCoralMode(true)));
        

        // default
        (new Trigger(() -> rod.isCoralMode() && copilot.getAButton())).onTrue(new SetRodState(rod, RodStates.L1));
        (new Trigger(() -> rod.isCoralMode() && copilot.getBButton())).onTrue(new SetRodState(rod, RodStates.L2));
        (new Trigger(() -> rod.isCoralMode() && copilot.getXButton())).onTrue(new SetRodState(rod, RodStates.L3));
        (new Trigger(() -> rod.isCoralMode() && copilot.getYButton())).onTrue(new SetRodState(rod, RodStates.L4));

        //algae mode
        new Trigger(() -> !rod.isCoralMode() && copilot.getAButton()).onTrue(new SetRodState(rod, RodStates.PROCESSOR));
        new Trigger(() -> !rod.isCoralMode() && copilot.getBButton()).onTrue(new SetRodState(rod, RodStates.LOW));
        new Trigger(() -> !rod.isCoralMode() && copilot.getXButton()).onTrue(new SetRodState(rod, RodStates.HIGH));
        new Trigger(() -> !rod.isCoralMode() && copilot.getYButton()).onTrue(new SetRodState(rod, RodStates.BARGE));

        // biases
        new Trigger(() -> copilot.getPOV() == 0).onTrue(rod.addElevatorBias(0.5d));
        new Trigger(() -> copilot.getPOV() == 180).onTrue(rod.addElevatorBias(-0.5d));

        new Trigger(() -> copilot.getPOV() == 90).onTrue(rod.addWristBias(-2.5));
        new Trigger(() -> copilot.getPOV() == 270).onTrue(rod.addWristBias(2.5));

        // algae control
        // new Trigger(() -> copilot.getRightTriggerAxis() > -1)
        //     .whileTrue(new CollectAlgae(algaeCollector, copilot::getRightTriggerAxis).deadlineFor(leds.strip.enableState(LEDStates.COLLECTING))); 

        // sim stuff
        // if (Robot.isSimulation()) {
        // new Trigger(copilot::getLeftBumperButton).whileTrue(new InstantCommand((() ->
        // wrist.setRawPower(-1))))
        // .onFalse(new InstantCommand(wrist::stop));
        // new Trigger(copilot::getRightBumperButton).whileTrue(new InstantCommand((()
        // -> wrist.setRawPower(1))))
        // .onFalse(new InstantCommand(wrist::stop));

        // new Trigger(driver::getYButton).whileTrue(new TagAutoAlign(vision,
        // drivetrain));

        // new Trigger(() -> copilot.getXButton()).whileTrue(new InstantCommand((() ->
        // coralCollector.setPower(0.75))))
        // .onFalse(new InstantCommand(coralCollector::stop));
        // new Trigger(() -> copilot.getBButton()).whileTrue(new InstantCommand((() ->
        // coralCollector.setPower(-0.5))))
        // .onFalse(new InstantCommand(coralCollector::stop));
        // }

        // LED testing
        new Trigger(() -> leds.getTestState() != null).whileTrue(leds.strip.enableState(LEDStates.MIXER));
        
        // SYSID
        // new Trigger(driver::getStartButton).whileTrue(new InstantCommand(() -> SignalLogger.start()));
        // new Trigger(driver::getYButton)
        //         .whileTrue(new SysIdSequence(drivetrain, DrivetrainConstants.SysIdTestType.DRIVE));
        // new Trigger(driver::getBackButton).whileTrue(new InstantCommand(() -> SignalLogger.stop()));

    }

    @Override
    protected void initializeNamedCommands() {
        // NamedCommands.registerCommand("ScoreCoral",
        // StandinCommands.scoreCoral().deadlineFor(leds.elevatorStrip.enableState(LEDStates.CORAL_SCORE)));

        /**
         * 1 is the target facing the driver station, 2 is to the right of 1...6 is to
         * left of 1 (CCW+)
         * 7 is blue-barge source, 8 is red-barge source
         */
        for (LightningTagID ID : LightningTagID.values()) {
            switch (ID) {
                case LeftSource, RightSource:
                    NamedCommands.registerCommand("AlignTo" + ID.name(), new PoseBasedAutoAlign(vision, drivetrain, Camera.RIGHT, leds,
                        ID).deadlineFor(leds.strip.enableState(LEDStates.ALIGNING)));
                    break;

                default:
                    NamedCommands.registerCommand("AlignTo" + ID.name() + "Left", new PoseBasedAutoAlign(vision, drivetrain, Camera.LEFT, leds,
                            ID).deadlineFor(leds.strip.enableState(LEDStates.ALIGNING)));
                    NamedCommands.registerCommand("AlignTo" + ID.name() + "Right", new PoseBasedAutoAlign(vision, drivetrain, Camera.RIGHT, leds,
                            ID).deadlineFor(leds.strip.enableState(LEDStates.ALIGNING)));
                    break;
            }
        }

        NamedCommands.registerCommand("IntakeCoral",
                new IntakeCoral(coralCollector, 1).withDeadline(new WaitCommand(3)));

        NamedCommands.registerCommand("RodStow",
                new SetRodState(rod, RodStates.STOW)
                        .deadlineFor(leds.strip.enableState(LEDStates.ROD_MOVING)));
        NamedCommands.registerCommand("RodL1",
                new SetRodState(rod, RodStates.L1)
                        .deadlineFor(leds.strip.enableState(LEDStates.ROD_MOVING)));
        NamedCommands.registerCommand("RodL2",
                new SetRodState(rod, RodStates.L2)
                        .deadlineFor(leds.strip.enableState(LEDStates.ROD_MOVING)));
        NamedCommands.registerCommand("RodL3",
                new SetRodState(rod, RodStates.L3)
                        .deadlineFor(leds.strip.enableState(LEDStates.ROD_MOVING)));
        NamedCommands.registerCommand("RodL4",
                new SetRodState(rod, RodStates.L4)
                        .deadlineFor(leds.strip.enableState(LEDStates.ROD_MOVING)));
        NamedCommands.registerCommand("RodSource",
                new SetRodState(rod, RodStates.SOURCE)
                        .deadlineFor(leds.strip.enableState(LEDStates.ROD_MOVING)));
        NamedCommands.registerCommand("ScoreCoral", new ScoreCoral(coralCollector));

        autoChooser = AutoBuilder.buildAutoChooser();
        LightningShuffleboard.send("Auton", "Auto Chooser", autoChooser);
    }

    @Override
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
            }).withDeadline(new WaitCommand(1));
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
            }).withDeadline(new WaitCommand(1));
        } else {
            return new InstantCommand();
        }
    }

}
