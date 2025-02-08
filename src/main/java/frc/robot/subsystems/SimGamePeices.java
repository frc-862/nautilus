// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants.AlgaeCollectorConstants;
import frc.robot.Constants.SimGamePeicesConstants;

public class SimGamePeices extends SubsystemBase {

    private Swerve drivetrain;
    private Elevator elevator;
    private Wrist wrist;
    private CoralCollector coralCollector;
    private AlgaeCollector algaeCollector;

    private HashMap<Integer, Peice> peices = new HashMap<Integer, Peice>();
    private HashMap<Integer, Coral> corals = new HashMap<Integer, Coral>();
    private HashMap<Integer, Algae> algaes = new HashMap<Integer, Algae>();

    private boolean hasPeice = false;
    private Peice heldPeice;
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

    private class Coral extends Peice {

        private Coral(Pose3d pose){
            super(NetworkTableInstance.getDefault().getTable("Shuffleboard")
                .getSubTable("SimGamePeices").getStructTopic("Coral: Peice# " + peices.size(), Pose3d.struct).publish());

            super.setPose(pose);
        }

        private Coral(){
            this(SimGamePeicesConstants.DEFAULT_POSE);
        }
    }

    private class Algae extends Peice {
        private Algae(Pose3d pose){
            super(NetworkTableInstance.getDefault().getTable("Shuffleboard")
                .getSubTable("SimGamePeices").getStructTopic("Algae: Peice# " + peices.size(), Pose3d.struct).publish());

            super.setPose(pose);
        }

        private Algae(){
            this(SimGamePeicesConstants.DEFAULT_POSE);
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
    private void collect(){

        if(hasPeice){
            return;
        }

        // check if the coral collector is moving fast enough to collect a peice
        if (coralCollector.getVelocity() < -SimGamePeicesConstants.COLECTOR_SPEED_THRESHHOLD){

            for (int i = 0; i < corals.size(); i++){

                Coral peice = corals.get(i);

                // check if the peice is close enough to the robot to be collected

                if(new Translation2d(peice.getPose().getX(), peice.getPose().getY())
                    .getDistance(drivetrain.getPose().getTranslation()) < SimGamePeicesConstants.COLLECTION_TOLERANCE
                    && Math.abs(Units.inchesToMeters(elevator.getPosition()) + SimGamePeicesConstants.ELEATOR_ROOT_HEIGHT 
                    - peice.getPose().getTranslation().getZ()) < SimGamePeicesConstants.COLLECTION_TOLERANCE){

                        hasPeice = true;
                        heldPeice = peice;

                        coralCollector.setSimBeamBreak(true);
                        return;
                }
            }
        }

        // check if the algae collector is moving fast enough to collect a peice
        if (algaeCollector.getRollerVelocity() < -SimGamePeicesConstants.COLECTOR_SPEED_THRESHHOLD){

            for (int i = 0; i < algaes.size(); i++){

                Algae peice = algaes.get(i);

                // check if the peice is close enough to the robot to be collected

                if(new Translation2d(peice.getPose().getX(), peice.getPose().getY())
                    .getDistance(drivetrain.getPose().getTranslation()) < SimGamePeicesConstants.COLLECTION_TOLERANCE){

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
    private void release(){
        if (hasPeice && heldPeice instanceof Coral && coralCollector.getVelocity() > SimGamePeicesConstants.COLECTOR_SPEED_THRESHHOLD){
            
            heldPeice = null;
            hasPeice = false;

            coralCollector.setSimBeamBreak(false);
            
        }

        if (hasPeice && heldPeice instanceof Algae && algaeCollector.getRollerVelocity() > SimGamePeicesConstants.COLECTOR_SPEED_THRESHHOLD){
    
            heldPeice = null;
            hasPeice = false;

        }
    }

    /**
     * set the pose of the held peice to the current pose of the robot
     */
    private void updateHeldPeicePose(){
        if (hasPeice){
            if (heldPeice instanceof Coral){
                heldPeice.setPose(new Pose3d(drivetrain.getPose().getX(), drivetrain.getPose().getY(), 
                Units.inchesToMeters(elevator.getPosition()) + SimGamePeicesConstants.ELEATOR_ROOT_HEIGHT, 
                new Rotation3d(0, wrist.getAngle(), drivetrain.getPose().getRotation().getDegrees() + 180)));
            }
            
            if (heldPeice instanceof Algae){
                heldPeice.setPose(new Pose3d(drivetrain.getPose().getX(), drivetrain.getPose().getY(), 
                SimGamePeicesConstants.ALGAE_COLLECTOR_ROOT_HEIGHT + (Math.sin(algaeCollector.getPivotAngle()) * AlgaeCollectorConstants.PIVOT_LENGTH), 
                new Rotation3d(0, algaeCollector.getPivotAngle(), drivetrain.getPose().getRotation().getDegrees())));
            }

            heldPeice.publish();
        }
    }
    

}