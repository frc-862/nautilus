
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.CoralCollectorConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DrivetrainConstants.DriveRequests;
import frc.robot.Constants.FishingRodConstants.RodStates;
import frc.robot.Constants.FishingRodConstants.RodTransitionStates;
import frc.robot.Constants.LEDConstants.LEDStates;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.PoseConstants.LightningTagID;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.RobotMotors;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.VisionConstants.ReefPose;
import frc.robot.commands.CollectCoral;
import frc.robot.commands.DefaultRodStow;
import frc.robot.commands.ElevatorSyncStow;
import frc.robot.commands.PoseBasedAutoAlign;
import frc.robot.commands.SetRodState;
import frc.robot.commands.SetRodStateReefAlgae;
import frc.robot.commands.SysIdSequence;
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

public class RobotContainer extends LightningContainer {

    public PowerDistribution pdh;

    public Swerve drivetrain;
    public PhotonVision vision;
    private Telemetry logger;
    private LEDs leds;

    private SendableChooser<Command> autoChooser;

    public Elevator elevator;
    private Wrist wrist;
    public FishingRod rod;
    private CoralCollector coralCollector;
    private AlgaeCollector algaeCollector;
    private Climber climber;

    @SuppressWarnings("unused")
    private SimGamePeices simGamePeices;

    private static XboxController driver;
    private static XboxController copilot;

    private RodStates queuedRodState = RodStates.DEFAULT; // The queued state that the copilot can invoke for
                                                          // AutonAutoAlign

    private boolean erroring = false; // side note: nate wtf. side note from kyle: nate wtf

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

        // if (Constants.ROBOT_IDENTIFIER != RobotIdentifiers.NAUTILUS) {
        // algaeCollector = new AlgaeCollector(RobotMotors.algaeCollectorRollerMotor,
        // RobotMotors.algaeCollectorPivotMotor);
        // }

        if (Robot.isSimulation()) {
            // algae collector and climber are temp because not initialized above
            algaeCollector = new AlgaeCollector(RobotMotors.algaeCollectorRollerMotor,
                    RobotMotors.algaeCollectorPivotMotor);

            simGamePeices = new SimGamePeices(elevator, wrist, drivetrain, coralCollector, algaeCollector,
                    climber);
        }
    }

    @Override
    protected void configureDefaultCommands() {
        // FIELD CENTRIC DRIVE
        drivetrain.setDefaultCommand(drivetrain.applyRequest(
                DriveRequests.getDrive(
                        () -> MathUtil.applyDeadband(
                                -(driver.getLeftY() * drivetrain.getSpeedMult()),
                                ControllerConstants.JOYSTICK_DEADBAND),
                        () -> MathUtil.applyDeadband(
                                -(driver.getLeftX() * drivetrain.getSpeedMult()),
                                ControllerConstants.JOYSTICK_DEADBAND),
                        () -> MathUtil.applyDeadband(
                                -(driver.getRightX() * drivetrain.getTurnMult()),
                                ControllerConstants.JOYSTICK_DEADBAND))));
        drivetrain.registerTelemetry(logger::telemeterize);

        // Stow behavior based on the drivetrain's current stow zone
        rod.setDefaultCommand(new DefaultRodStow(drivetrain, rod));

        // COPILOT INTAKE
        coralCollector.setDefaultCommand(new CollectCoral(coralCollector, leds,
                () -> MathUtil.applyDeadband(
                        copilot.getRightTriggerAxis() - copilot.getLeftTriggerAxis(),
                        CoralCollectorConstants.COLLECTOR_DEADBAND),
                rod::isCoralMode, () -> rod.getState() == RodStates.L1));

        // COPILOT CLIMB
        climber.setDefaultCommand(new RunCommand(
                () -> climber.setPower(
                        MathUtil.applyDeadband(-copilot.getLeftY(),
                                ControllerConstants.JOYSTICK_DEADBAND)),
                climber));

        // Sync elevator while in stow
        new Trigger(() -> rod.getState() == RodStates.STOW).and(DriverStation::isTeleop).whileTrue(new ElevatorSyncStow(elevator, rod));

        /* LED TRIGGERS */
        new Trigger(() -> rod.onTarget()).whileFalse(leds.strip.enableState(LEDStates.ROD_MOVING));

        new Trigger(() -> elevator.isOverheating()).onTrue(new InstantCommand(() -> erroring = true));

        new Trigger(() -> erroring && DriverStation.isDisabled())
                .whileTrue(leds.strip.enableState(LEDStates.ERROR));

        new Trigger(() -> !rod.isCoralMode() && DriverStation.isEnabled()).whileTrue(leds.strip.enableState(LEDStates.ALGAE_MODE));

        new Trigger(() -> (coralCollector.getTargetPower() > (rod.isCoralMode() ? CoralCollectorConstants.CORAL_HOLD_POWER : CoralCollectorConstants.ALGAE_HOLD_POWER))).whileTrue(leds.strip.enableState(LEDStates.COLLECTING));

        leds.strip.setState(LEDStates.POSE_BAD, true).schedule();

        new Trigger(() -> (!drivetrain.poseStable() && DriverStation.isDisabled()))
                .onTrue(leds.strip.setState(LEDStates.POSE_BAD, false))
                .whileTrue(leds.strip.enableState(LEDStates.UPDATING_POSE));

        new Trigger(() -> (PoseConstants.getScorePose(drivetrain.getPose()) != 0 && DriverStation.isEnabled()))
                .whileTrue(leds.strip.enableState(LEDStates.READY_TO_ALIGN));

        new Trigger(() -> climber.getLimitSwitch()).whileTrue(leds.strip.enableState(LEDStates.CLIMBED));
    }

    @Override
    protected void configureButtonBindings() {
        /* DRIVER BINDINGS */
        // robot centric driving
        new Trigger(() -> driver.getLeftTriggerAxis() > 0.25)
                .whileTrue(drivetrain.applyRequest(DriveRequests.getRobotCentric(
                        () -> MathUtil.applyDeadband(
                                -(driver.getLeftX() * drivetrain.getSpeedMult()),
                                ControllerConstants.JOYSTICK_DEADBAND),
                        () -> MathUtil.applyDeadband(
                                -(driver.getLeftY() * drivetrain.getSpeedMult()),
                                ControllerConstants.JOYSTICK_DEADBAND),
                        () -> MathUtil.applyDeadband(
                                -(driver.getRightX() * drivetrain.getTurnMult()),
                                ControllerConstants.JOYSTICK_DEADBAND))));

        // sets slow mode
        new Trigger(() -> driver.getRightTriggerAxis() > 0.25)
                .onTrue(new InstantCommand(() -> drivetrain.setSlowMode(true)))
                .onFalse(new InstantCommand(() -> drivetrain.setSlowMode(false)));

        // drivetrain brake
        new Trigger(driver::getXButton).whileTrue(drivetrain.applyRequest(DriveRequests.getBrake()));

        // reset forward
        new Trigger(() -> driver.getStartButton() && driver.getBackButton()).onTrue(
                new InstantCommand(drivetrain::seedFieldCentric).alongWith(
                        new InstantCommand(() -> drivetrain
                                .setOperatorPerspectiveForward(new Rotation2d(Degrees
                                        .of(DriverStation.getAlliance().orElse(
                                                Alliance.Blue) == Alliance.Red
                                                        ? 180
                                                        : 0))))));

        // AUTO ALIGN
        new Trigger(driver::getLeftBumperButton)
                .whileTrue(PoseBasedAutoAlign.getPoseAutoAlign(drivetrain, ReefPose.RIGHT, leds).withRodState(rod, () -> queuedRodState)
                        .deadlineFor(leds.strip.enableState(LEDStates.ALIGNING)));
        new Trigger(driver::getRightBumperButton)
                .whileTrue(PoseBasedAutoAlign.getPoseAutoAlign(drivetrain, ReefPose.LEFT, leds).withRodState(rod, () -> queuedRodState)
                        .deadlineFor(leds.strip.enableState(LEDStates.ALIGNING)));

        // new Trigger(driver::getBButton)
        // .whileTrue(new AutonAutoAlign(drivetrain, Camera.LEFT, leds, rod,
        // LightningTagID.Two,
        // () -> queuedRodState));

        // L1 AUTOALIGN
        new Trigger(() -> (driver.getLeftBumperButton() && driver.getAButton()))
                .whileTrue(PoseBasedAutoAlign.getL1PoseAutoAlign(drivetrain, ReefPose.RIGHT, leds)
                        .deadlineFor(leds.strip.enableState(LEDStates.ALIGNING)));
        new Trigger(() -> (driver.getRightBumperButton() && driver.getAButton()))
                .whileTrue(PoseBasedAutoAlign.getL1PoseAutoAlign(drivetrain, ReefPose.LEFT, leds)
                        .deadlineFor(leds.strip.enableState(LEDStates.ALIGNING)));

        // BARGE Autoalign
        new Trigger(driver::getBButton)
                .whileTrue(PoseBasedAutoAlign.getLightningIDAutoAlign(drivetrain, ReefPose.RIGHT, leds, LightningTagID.Barge)
                        .withYControl(() -> -driver.getLeftX())
                        .deadlineFor(leds.strip.enableState(LEDStates.ALIGNING)));

        // source autoalign
        // new Trigger(() -> driver.getPOV() == 0)
        // .whileTrue(new PoseBasedAutoAlign(drivetrain, Camera.RIGHT, leds,
        // LightningTagID.RightSource)
        // .deadlineFor(leds.strip.enableState(LEDStates.ALIGNING)));

        /* COPILOT BINDINGS */

        // Stow (automaticaly goes to coral mode and reset the rod queue)
        new Trigger(copilot::getLeftBumperButton)
                .onTrue((new InstantCommand(() -> rod.setCoralMode(true))
                        .alongWith(new InstantCommand(
                                () -> queuedRodState = RodStates.DEFAULT)))
                        .andThen(new SetRodState(rod, RodStates.STOW).withStowZoneCheck(drivetrain::getCurrentStowZone, drivetrain::getLastStowZone)));

        // Swap modes (TODO: add LED state notif)
        new Trigger(copilot::getBackButton)
                .onTrue(new InstantCommand(() -> rod.setCoralMode(false)));
        new Trigger(copilot::getStartButton)
                .onTrue(new InstantCommand(() -> rod.setCoralMode(true)));

        // coral mode
        new Trigger(() -> rod.isCoralMode() && driver.getYButton())
                .onTrue(new SetRodState(rod, RodStates.INVERSE_STOW));
        new Trigger(() -> rod.isCoralMode() && copilot.getAButton()).onTrue(new SetRodState(rod, RodStates.L1));
        new Trigger(() -> rod.isCoralMode() && copilot.getBButton()).onTrue(new SetRodState(rod, RodStates.L2));
        new Trigger(() -> rod.isCoralMode() && copilot.getXButton()).onTrue(new SetRodState(rod, RodStates.L3));
        // new Trigger(() -> rod.isCoralMode() && copilot.getYButton()).onTrue(new
        // SetRodState(rod, RodStates.L4));

        // new Trigger(() -> rod.isCoralMode() && copilot.getAButton())
        // .whileTrue(setRodQueue(RodStates.L1))
        // .onFalse(new SetRodState(rod,
        // RodStates.L1).andThen(resetRodQueue(RodStates.L1)));

        // new Trigger(() -> rod.isCoralMode() && copilot.getBButton())
        // .whileTrue(setRodQueue(RodStates.L2))
        // .onFalse(new SetRodState(rod,
        // RodStates.L2).andThen(resetRodQueue(RodStates.L2)));

        // new Trigger(() -> rod.isCoralMode() && copilot.getXButton())
        // .whileTrue(setRodQueue(RodStates.L3))
        // .onFalse(new SetRodState(rod,
        // RodStates.L3).andThen(resetRodQueue(RodStates.L3)));

        new Trigger(() -> rod.isCoralMode() && copilot.getYButton())
                .whileTrue(setRodQueue(RodStates.L4))
                .onFalse(new SetRodState(rod, RodStates.L4).andThen(resetRodQueue(RodStates.L4)));
        new Trigger(() -> rod.isCoralMode() && copilot.getRightBumperButton())
                .onTrue(new SetRodState(rod, RodStates.SOURCE));

        // algae mode
        new Trigger(() -> !rod.isCoralMode() && copilot.getAButton())
                .onTrue(new SetRodState(rod, RodStates.PROCESSOR));
        new Trigger(() -> !rod.isCoralMode() && copilot.getBButton())
                .onTrue(new SetRodState(rod, RodStates.LOW));
        new Trigger(() -> !rod.isCoralMode() && copilot.getXButton())
                .onTrue(new SetRodState(rod, RodStates.HIGH));
        new Trigger(() -> !rod.isCoralMode() && copilot.getYButton())
                .onTrue(new SetRodState(rod, RodStates.BARGE));
        new Trigger(() -> !rod.isCoralMode() && copilot.getRightBumperButton())
                .onTrue(new SetRodStateReefAlgae(drivetrain, rod));

        // biases
        new Trigger(() -> copilot.getPOV() == 0).onTrue(rod.addElevatorBias(0.5d)); // ELE UP
        new Trigger(() -> copilot.getPOV() == 180).onTrue(rod.addElevatorBias(-0.5d)); // ELE DOWN
        new Trigger(() -> copilot.getPOV() == 90).onTrue(rod.addWristBias(-2.5)); // WRIST DOWN
        new Trigger(() -> copilot.getPOV() == 270).onTrue(rod.addWristBias(2.5)); // WRIST UP

        // algae control
        // new Trigger(() -> copilot.getRightTriggerAxis() > -1)
        // .whileTrue(new CollectAlgae(algaeCollector,
        // copilot::getRightTriggerAxis).deadlineFor(leds.strip.enableState(LEDStates.COLLECTING)));

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
        // new Trigger(driver::getAButton).whileTrue(new SysIdSequence(drivetrain,
        //         DrivetrainConstants.SysIdTestType.DRIVE));
        // new Trigger(driver::getBackButton).whileTrue(new InstantCommand(() -> SignalLogger.stop()));

    }

    @Override
    protected void initializeNamedCommands() {
        /*
         * 1 is the target facing the driver station, 2 is to the right of 1...6 is to
         * left of 1 (CCW+)
         * 7 is blue-barge source, 8 is red-barge source
         */
        for (LightningTagID tagID : LightningTagID.values()) {
            switch (tagID) {
                case LeftSource, RightSource:
                    // NamedCommands.registerCommand("AlignTo" + tagID.name(),
                    // new ParallelDeadlineGroup(
                    // new ParallelRaceGroup(new IntakeCoral(coralCollector, 1), new
                    // WaitCommand(4d)),
                    // new PoseBasedAutoAlign(drivetrain, Camera.RIGHT, leds,
                    // tagID).deadlineFor(leds.strip.enableState(LEDStates.ALIGNING)),
                    // new SetRodState(rod, RodStates.SOURCE)
                    // .deadlineFor(leds.strip.enableState(LEDStates.ROD_MOVING))));

                    NamedCommands.registerCommand("AlignTo" + tagID.name(),
                        PoseBasedAutoAlign.getLightningIDAutoAlign(drivetrain, ReefPose.RIGHT, leds, tagID)
                            .deadlineFor(leds.strip.enableState(LEDStates.ALIGNING)));

                    NamedCommands.registerCommand("AUTONAlignTo" + tagID.name(),
                        PoseBasedAutoAlign.getLightningIDAutoAlign(drivetrain, ReefPose.RIGHT, leds, tagID)
                            .withRodState(rod, () -> RodStates.SOURCE)
                            .deadlineFor(leds.strip.enableState(LEDStates.ALIGNING)));

                    break;

                case Barge, BargeBack:
                    NamedCommands.registerCommand("AlignTo" + tagID.name(),
                        PoseBasedAutoAlign.getLightningIDAutoAlign(drivetrain, ReefPose.RIGHT, leds, tagID)
                            .withRodState(rod, () -> RodStates.BARGE)
                            .deadlineFor(leds.strip.enableState(LEDStates.ALIGNING)));

                    break;

                default:
                    NamedCommands.registerCommand("AlignTo" + tagID.name() + "Left",
                        PoseBasedAutoAlign.getLightningIDAutoAlign(drivetrain, ReefPose.LEFT, leds, tagID)
                            .deadlineFor(leds.strip.enableState(LEDStates.ALIGNING)));
                    NamedCommands.registerCommand("AlignTo" + tagID.name() + "Right",
                        PoseBasedAutoAlign.getLightningIDAutoAlign(drivetrain, ReefPose.RIGHT, leds, tagID)
                            .deadlineFor(leds.strip.enableState(LEDStates.ALIGNING)));

                    NamedCommands.registerCommand("AUTONAlignTo" + tagID.name() + "Left",
                        PoseBasedAutoAlign.getLightningIDAutoAlign(drivetrain, ReefPose.LEFT, leds, tagID)
                            .withRodState(rod, () -> RodStates.L4)
                            .deadlineFor(leds.strip.enableState(LEDStates.ALIGNING)));
                    NamedCommands.registerCommand("AUTONAlignTo" + tagID.name() + "Right",
                        PoseBasedAutoAlign.getLightningIDAutoAlign(drivetrain, ReefPose.RIGHT, leds, tagID)
                            .withRodState(rod, () -> RodStates.L4)
                            .deadlineFor(leds.strip.enableState(LEDStates.ALIGNING)));

                // test for algae
                    NamedCommands.registerCommand("AlignTo" + tagID.name() + "Middle",
                        PoseBasedAutoAlign.getLightningIDAutoAlign(drivetrain, ReefPose.MIDDLE, leds, tagID)
                            .deadlineFor(leds.strip.enableState(LEDStates.ALIGNING)));

                    NamedCommands.registerCommand("OpAlignTo" + tagID.name() + "Middle",
                            PoseBasedAutoAlign.getOtherAllianceLightningIDAutoAlign(drivetrain, ReefPose.MIDDLE, leds, tagID)
                                .deadlineFor(leds.strip.enableState(LEDStates.ALIGNING)));

                    break;
            }
        }

        NamedCommands.registerCommand("AutoAlignRange", new WaitUntilCommand(() -> PoseConstants.getScorePose(drivetrain.getPose()) != 0));

        NamedCommands.registerCommand("IntakeCoral",
                new IntakeCoral(coralCollector, 1, true));

        NamedCommands.registerCommand("IntakeAlgae",
                new IntakeCoral(coralCollector, 1, false));

        NamedCommands.registerCommand("IntuahCoral",
                new RunCommand(() -> coralCollector.setPower(1), coralCollector));

        NamedCommands.registerCommand("StowTuah",
                new InstantCommand(() -> rod.setState(RodStates.STOW, RodTransitionStates.DEFAULT)));

        NamedCommands.registerCommand("ScoreCoral",
                new ScoreCoral(coralCollector));

        NamedCommands.registerCommand("WristThenStow",
                new InstantCommand(() -> rod.setState(RodStates.STOW,
                        RodTransitionStates.WRIST_UP_THEN_ELE)));

        NamedCommands.registerCommand("WaitOnTarget", new WaitUntilCommand(rod::onTarget));

        // psst! we should use a for loop like we did with LightningTagID!
        NamedCommands.registerCommand("RodStow",
                (new InstantCommand(() -> rod.setCoralMode(true))
                        .alongWith(new SetRodState(rod, RodStates.STOW)))
                        .deadlineFor(leds.strip.enableState(LEDStates.ROD_MOVING)));
        NamedCommands.registerCommand("RodInverseStow",
                (new InstantCommand(() -> rod.setCoralMode(true)).alongWith(
                        new InstantCommand(() -> rod.setState(RodStates.INVERSE_STOW))))
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
        NamedCommands.registerCommand("RodHigh",
                (new InstantCommand(() -> rod.setCoralMode(false))
                        .alongWith(new SetRodState(rod, RodStates.HIGH)))
                        .deadlineFor(leds.strip.enableState(LEDStates.ROD_MOVING)));
        NamedCommands.registerCommand("RodLow",
                (new InstantCommand(() -> {
                    rod.setCoralMode(false);
                    rod.setState(RodStates.LOW);
                }).deadlineFor(leds.strip.enableState(LEDStates.ROD_MOVING))));
        NamedCommands.registerCommand("RodProcessor",
                (new InstantCommand(() -> rod.setCoralMode(false))
                        .alongWith(new SetRodState(rod, RodStates.PROCESSOR)))
                        .deadlineFor(leds.strip.enableState(LEDStates.ROD_MOVING)));
        NamedCommands.registerCommand("RodSource",
                new SetRodState(rod, RodStates.SOURCE)
                        .deadlineFor(leds.strip.enableState(LEDStates.ROD_MOVING)));

        NamedCommands.registerCommand("RodLow",
                new SetRodState(rod, RodStates.LOW)
                        .deadlineFor(leds.strip.enableState(LEDStates.ROD_MOVING)));

        NamedCommands.registerCommand("DriveLeft", drivetrain.applyRequest(() -> DriveRequests.getRobotCentric(2, 1, 0)).withTimeout(0.5));
        NamedCommands.registerCommand("DriveRight", drivetrain.applyRequest(() -> DriveRequests.getRobotCentric(-2, 1, 0)).withTimeout(0.5));
        NamedCommands.registerCommand("DriveBackLeft", drivetrain.applyRequest(() -> DriveRequests.getRobotCentric(1, 2, 0)).withTimeout(0.5));
        NamedCommands.registerCommand("DriveBackRight", drivetrain.applyRequest(() -> DriveRequests.getRobotCentric(-1, 2, 0)).withTimeout(0.5));

        NamedCommands.registerCommand("SetCoralMode", new InstantCommand(() -> rod.setCoralMode(true)));
        NamedCommands.registerCommand("SetAlgaeMode", new InstantCommand(() -> rod.setCoralMode(false)));

        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption("LEAVE", drivetrain.leaveAuto());
        LightningShuffleboard.send("Auton", "Auto Chooser", autoChooser);
    }

    @Override
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Sets queuedRodState to the new state
     *
     * @param newState
     * @return command to set the new state
     */
    private Command setRodQueue(RodStates newState) {
        return new InstantCommand(() -> queuedRodState = newState);
    }

    /**
     * Sets queuedRodState to default if it is the same as the checkState
     *
     * Ex. We are going to L4, and when the button becomes false again
     * we can stop queuing L4 since it is in the queue already
     *
     * @param checkState State to compare to
     * @return command to reset the queue
     */
    private Command resetRodQueue(RodStates checkState) {
        return new InstantCommand(() -> {
            if (queuedRodState == checkState) {
                queuedRodState = RodStates.DEFAULT;
            }
        });
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
