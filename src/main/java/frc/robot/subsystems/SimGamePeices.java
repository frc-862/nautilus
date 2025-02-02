// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants.AlgaeCollectorConstants;
import frc.robot.Constants.SimGamePeicesConstants;

public class SimGamePeices extends SubsystemBase {

    public Swerve drivetrain;
    public Elevator elevator;
    public Wrist wrist;
    public CoralCollector coralCollector;
    public AlgaeCollector algaeCollector;

    public HashMap<Integer, Peice> peices = new HashMap<Integer, Peice>();
    public HashMap<Integer, Coral> corals = new HashMap<Integer, Coral>();
    public HashMap<Integer, Algae> algaes = new HashMap<Integer, Algae>();

    public boolean hasPeice = false;
    public Peice heldPeice;
    public class Peice{
        public Pose3d pose;
        public boolean inRobot;
        public int peiceNumber;
        public StructPublisher<Pose3d> publisher;

        public Peice(int peiceNumber, StructPublisher<Pose3d> publisher){
            this.publisher = publisher;
            this.peiceNumber = peiceNumber;
        }


        public Pose3d getPose(){
            return pose;
        }

        public void setPose(Pose3d pose){
            this.pose = pose;
        }

        /**
         * @return whether the peice is in the robot
         */
        public boolean inRobot(){
            return inRobot;
        }

        /**
         * @param inRobot whether the peice is in the robot
         */
        public void setInRobot(boolean inRobot){
            this.inRobot = inRobot;
        }

        /**
         * @return the number of the peice (should be the index in peices[])
         */
        public int getNumber(){
            return peiceNumber;
        }

        /**
         * publish the pose3d of the peice to advantageScope
         */
        public void publish(){
            publisher.set(pose);
        }

    }

    public class Coral extends Peice {

        public Coral(Pose3d pose){
            super(peices.size(), NetworkTableInstance.getDefault().getTable("Shuffleboard")
                .getSubTable("SimGamePeices").getStructTopic("Coral: Peice# " + peices.size(), Pose3d.struct).publish());

            setPose(pose);
        }

        public Coral(){
            super(peices.size(), NetworkTableInstance.getDefault().getTable("Shuffleboard")
                .getSubTable("SimGamePeices").getStructTopic("Coral: Peice# " + peices.size(), Pose3d.struct).publish());

            setPose(SimGamePeicesConstants.DEFAULT_POSE);
        }
    }

    public class Algae extends Peice {
        public Algae(Pose3d pose){
            super(peices.size(), NetworkTableInstance.getDefault().getTable("Shuffleboard")
                .getSubTable("SimGamePeices").getStructTopic("Algae: Peice# " + peices.size(), Pose3d.struct).publish());

            setPose(pose);
        }

        public Algae(){
            super(peices.size(), NetworkTableInstance.getDefault().getTable("Shuffleboard")
                .getSubTable("SimGamePeices").getStructTopic("Algae: Peice# " + peices.size(), Pose3d.struct).publish());

            setPose(SimGamePeicesConstants.DEFAULT_POSE);
        }
    }

    public SimGamePeices(Elevator elevator, Wrist wrist, Swerve drivetrain, CoralCollector coralCollector, AlgaeCollector algaeCollector) {

        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.wrist = wrist;
        this.coralCollector = coralCollector;
        this.algaeCollector = algaeCollector;
        
        // add default peices
        addPeice(new Algae(SimGamePeicesConstants.A1B));
        addPeice(new Algae(SimGamePeicesConstants.A2B));
        addPeice(new Algae(SimGamePeicesConstants.A3B));
        addPeice(new Algae(SimGamePeicesConstants.A1R));
        addPeice(new Algae(SimGamePeicesConstants.A2R));
        addPeice(new Algae(SimGamePeicesConstants.A3R));

        Peice startingCoral = new Coral();
        addPeice(startingCoral);
        startingCoral.setInRobot(true);

        hasPeice = true;
        heldPeice = startingCoral;
  }

    @Override
    public void periodic() {
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
    public void collect(){

        if(hasPeice){
            return;
        }

        // check if the coral collector is moving fast enough to collect a peice
        if (coralCollector.getVelocity() > SimGamePeicesConstants.COLECTOR_SPEED_THRESHHOLD){

            for (int i = 0; i < corals.size(); i++){

                Coral peice = corals.get(i);

                // check if the peice is close enough to the robot to be collected

                if(new Translation2d(peice.pose.getX(), peice.pose.getY())
                    .getDistance(drivetrain.getPose().getTranslation()) < SimGamePeicesConstants.COLLECTION_TOLERANCE
                    && Math.abs(elevator.getPosition() + SimGamePeicesConstants.ELEATOR_ROOT_HEIGHT 
                    - peice.pose.getTranslation().getZ()) < SimGamePeicesConstants.COLLECTION_TOLERANCE){

                        peice.inRobot = true;
                        hasPeice = true;
                        heldPeice = peice;

                        coralCollector.setSimBeamBreak(true);
                        return;
                }
            }
        }

        // check if the algae collector is moving fast enough to collect a peice
        if (algaeCollector.getRollerVelocity() > SimGamePeicesConstants.COLECTOR_SPEED_THRESHHOLD){

            for (int i = 0; i < algaes.size(); i++){

                Algae peice = algaes.get(i);

                // check if the peice is close enough to the robot to be collected

                if(new Translation2d(peice.pose.getX(), peice.pose.getY())
                    .getDistance(drivetrain.getPose().getTranslation()) < SimGamePeicesConstants.COLLECTION_TOLERANCE){

                        peice.inRobot = true;
                        hasPeice = true;
                        heldPeice = peice;
                        return;
                }
            }
        }
    }

    /**
     * release the held peice if the collector is ejecting it
     */
    public void release(){
        if (hasPeice && heldPeice instanceof Coral){
            if(coralCollector.getVelocity() < -SimGamePeicesConstants.COLECTOR_SPEED_THRESHHOLD){
                heldPeice.inRobot = false;
                heldPeice = null;
                hasPeice = false;

                coralCollector.setSimBeamBreak(false);
            }
        }

        if (hasPeice && heldPeice instanceof Algae){
            if(algaeCollector.getRollerVelocity() < -SimGamePeicesConstants.COLECTOR_SPEED_THRESHHOLD){
                heldPeice.inRobot = false;
                heldPeice = null;
                hasPeice = false;
            }
        }
    }

    /**
     * set the pose of the held peice to the current pose of the robot
     */
    public void updateHeldPeicePose(){
        if (hasPeice){
            if (heldPeice instanceof Coral){
                heldPeice.pose = new Pose3d(drivetrain.getPose().getX(), drivetrain.getPose().getY(), 
                elevator.getPosition() + SimGamePeicesConstants.ELEATOR_ROOT_HEIGHT, 
                new Rotation3d(0, wrist.getAngle(), drivetrain.getPose().getRotation().getDegrees() + 180));
            }
            
            if (heldPeice instanceof Algae){
                heldPeice.pose = new Pose3d(drivetrain.getPose().getX(), drivetrain.getPose().getY(), 
                SimGamePeicesConstants.ALGAE_COLLECTOR_ROOT_HEIGHT + (Math.sin(algaeCollector.getPivotAngle()) * AlgaeCollectorConstants.PIVOT_LENGTH), 
                new Rotation3d(0, algaeCollector.getPivotAngle(), drivetrain.getPose().getRotation().getDegrees()));
            }

            heldPeice.publish();
        }
    }
    

}