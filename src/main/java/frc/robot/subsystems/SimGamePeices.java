// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants.AlgaeCollectorConstants;
import frc.robot.Constants.SimGamePeicesConstants;
import frc.robot.Constants.WristConstants;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class SimGamePeices extends SubsystemBase {

    /* 
     * Instructions:
     * 
     * 0. Open 3d Field in AdvantageScope
     * 1. Drag in the poses in the tab "SimGamePieces" (only need those listed under Algae and Coral)
     * 2. You Start With a coral in the robot, and the Algaes should be where they start in the match
     * 3. Collecting/ejecting peices is based on the collector speed and estimated position of the collectors
     */

    private Swerve drivetrain;
    private Elevator elevator;
    private Wrist wrist;
    private CoralCollector coralCollector;
    private AlgaeCollector algaeCollector;
    private Climber climber;

    private HashMap<Integer, Peice> peices = new HashMap<Integer, Peice>();
    private HashMap<Integer, Coral> corals = new HashMap<Integer, Coral>();
    private HashMap<Integer, Algae> algaes = new HashMap<Integer, Algae>();

    private Pose3d coralCollectorPose;
    private Pose3d algaeCollectorPose;

    private boolean hasPeice = false;
    private Peice heldPeice;

    private boolean heldPeiceInCoralCollector = false;

    private StructPublisher<Pose3d> robotPosePublisher = NetworkTableInstance.getDefault().getTable("Shuffleboard")
        .getSubTable("SimGamePeices").getStructTopic("Robot: Robot Pose", Pose3d.struct).publish();
    private StructPublisher<Pose3d> coralCollectorPosePublisher = NetworkTableInstance.getDefault().getTable("Shuffleboard")
        .getSubTable("SimGamePeices").getStructTopic("Robot: Coral Collector Pose", Pose3d.struct).publish();
    private StructPublisher<Pose3d> algaeCollectorPosePublisher = NetworkTableInstance.getDefault().getTable("Shuffleboard")
        .getSubTable("SimGamePeices").getStructTopic("Robot: Algae Collector Pose", Pose3d.struct).publish();

    private class Peice{
        private Pose3d pose;
        private StructPublisher<Pose3d> publisher;

        private Peice(StructPublisher<Pose3d> publisher){
            this.publisher = publisher;
        }


        protected Pose3d getPose(){
            return pose;
        }

        private void setPose(Pose3d pose){
            this.pose = pose;
        }

        /**
         * publish the pose3d of the peice to advantageScope
         */
        private void publish(){
            publisher.set(pose);
        }

    }

    private final class Coral extends Peice {

        private Coral(Pose3d pose){
            super(NetworkTableInstance.getDefault().getTable("Shuffleboard")
                .getSubTable("SimGamePeices").getStructTopic("Coral: Peice# " + peices.size(), Pose3d.struct).publish());

            super.setPose(pose);
        }

        private Coral(){
            this(SimGamePeicesConstants.DEFAULT_POSE);
        }
    }

    private final class Algae extends Peice {
        private Algae(Pose3d pose){
            super(NetworkTableInstance.getDefault().getTable("Shuffleboard")
                .getSubTable("SimGamePeices").getStructTopic("Algae: Peice# " + peices.size(), Pose3d.struct).publish());

            super.setPose(pose);
        }

        private Algae(){
            this(SimGamePeicesConstants.DEFAULT_POSE);
        }
    }

    public SimGamePeices(Elevator elevator, Wrist wrist, Swerve drivetrain, CoralCollector coralCollector, AlgaeCollector algaeCollector,
            Climber climber) {

        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.wrist = wrist;
        this.coralCollector = coralCollector;
        this.algaeCollector = algaeCollector;
        this.climber = climber;
        
        // add default peices
        addPeice(new Algae(SimGamePeicesConstants.A1B));
        addPeice(new Algae(SimGamePeicesConstants.A2B));
        addPeice(new Algae(SimGamePeicesConstants.A3B));
        addPeice(new Algae(SimGamePeicesConstants.A1R));
        addPeice(new Algae(SimGamePeicesConstants.A2R));
        addPeice(new Algae(SimGamePeicesConstants.A3R));

        Peice startingCoral = new Coral();
        addPeice(startingCoral);

        heldPeiceInCoralCollector = true;
        hasPeice = true;
        heldPeice = startingCoral;
  }

    @Override
    public void periodic() {
        updateRobotPoses();
        collect();
        release();
        updateHeldPeicePose();
    }

    /**
     * @param peice add peice to hashmaps of peices, corals, and algaes
     */
    public void addPeice(Peice peice){

        peices.put(peices.size(), peice);

        peice.publish();

        if(peice instanceof Coral){
            corals.put(corals.size(), (Coral) peice);
        }

        if(peice instanceof Algae){
            algaes.put(algaes.size(), (Algae) peice);
        }
    }

    /**
     * if a collector is moving fast enough, check if the robot is close enough to a peice to collect it
     */
    private void collect(){

        if(hasPeice){
            return;
        }

        // check if the coral collector is moving fast enough to collect a peice
        if (coralCollector.getVelocity() < -SimGamePeicesConstants.COLECTOR_SPEED_THRESHHOLD){

            for (int i = 0; i < peices.size(); i++){

                Peice peice = peices.get(i);

                // check if the peice is close enough to the robot to be collected

                if(peice.getPose().getTranslation().getDistance(coralCollectorPose.getTranslation()) 
                    < SimGamePeicesConstants.COLLECTION_TOLERANCE){

                        hasPeice = true;
                        heldPeice = peice;
                        coralCollector.setSimBeamBreak(true);
                        heldPeiceInCoralCollector = true;
                        return;
                }
            }
        }

        // check if the algae collector is moving fast enough to collect a peice
        if (algaeCollector.getRollerVelocity() < -SimGamePeicesConstants.COLECTOR_SPEED_THRESHHOLD){

            for (int i = 0; i < algaes.size(); i++){

                Algae peice = algaes.get(i);

                // check if the peice is close enough to the robot to be collected

                if(peice.getPose().getTranslation().getDistance(algaeCollectorPose.getTranslation())
                    < SimGamePeicesConstants.COLLECTION_TOLERANCE){

                        hasPeice = true;
                        heldPeice = peice;
                        heldPeiceInCoralCollector = false;
                        return;
                }
            }
        }
    }

    /**
     * release the held peice if the collector is ejecting it
     */
    private void release(){

        // check collector speeds, and distance from applicable collector

        if (hasPeice && heldPeiceInCoralCollector && coralCollector.getVelocity() > SimGamePeicesConstants.COLECTOR_SPEED_THRESHHOLD){
            
            heldPeice = null;
            hasPeice = false;
            coralCollector.setSimBeamBreak(false);
        }

        if (hasPeice && !heldPeiceInCoralCollector && algaeCollector.getRollerVelocity() > SimGamePeicesConstants.COLECTOR_SPEED_THRESHHOLD){
    
            heldPeice = null;
            hasPeice = false;

        }
    }

    /**
     * update collector poses for peice pose estimation
     */
    private void updateRobotPoses(){

        Pose3d robotPose = new Pose3d(drivetrain.getPose()).plus(new Transform3d(0, 0, -Units.inchesToMeters(climber.getPostion()), 
            new Rotation3d()));
        double robotAngle = robotPose.getRotation().getZ();

        double wristAngle = Units.degreesToRadians(wrist.getAngle());
        double algaeCollectorAngle = Units.degreesToRadians(algaeCollector.getPivotAngle());
        double elevatorHeight = Units.inchesToMeters(elevator.getPosition());

        double wristLength = WristConstants.LENGTH.in(Meters) * 0.5;
        double algaeCollectorLength = AlgaeCollectorConstants.PIVOT_LENGTH * 0.5;

        // add offsets to robot pose to get collector poses

        coralCollectorPose = new Pose3d(
            // elevator root offset + wrist length (take angle into account) + elevator height -> make field centric -> add robot pose & rotation
            SimGamePeicesConstants.ELEVATOR_OFFSET.plus(new Translation3d(0, -Math.cos(wristAngle) * wristLength, 
            Math.sin(wristAngle) * wristLength + elevatorHeight)).rotateBy(new Rotation3d(0, 0, robotAngle - Math.PI / 2))
            .plus(robotPose.getTranslation()), new Rotation3d(0, Units.degreesToRadians(wrist.getAngle()), robotAngle - Math.PI));

        algaeCollectorPose = new Pose3d(
            // algae collector root offset + arm length (take angle into account) -> make feild centric -> add robot pose and rotation
            SimGamePeicesConstants.ALGAE_COLLECTOR_OFFSET.plus(new Translation3d(0, Math.cos(algaeCollectorAngle) * algaeCollectorLength, 
            Math.sin(algaeCollectorAngle) * algaeCollectorLength)).rotateBy(new Rotation3d(0, 0, robotAngle - Math.PI / 2))
            .plus(robotPose.getTranslation()), new Rotation3d(drivetrain.getPose().getRotation()));

        // publish poses to networktables (mostly useless, but can be used to see poses in advantagescope)
        robotPosePublisher.set(robotPose);
        coralCollectorPosePublisher.set(coralCollectorPose);
        algaeCollectorPosePublisher.set(algaeCollectorPose);
    }

    /**
     * set the pose of the held peice to the current pose of the robot
     */
    private void updateHeldPeicePose(){

        // set the pose of the peice to that of the applicable collector

        if (hasPeice){
            if (heldPeiceInCoralCollector){
                heldPeice.setPose(coralCollectorPose);
            }
            
            if (!heldPeiceInCoralCollector){
                heldPeice.setPose(algaeCollectorPose);
            }

            heldPeice.publish();
        }
    }
    

}