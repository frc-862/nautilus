// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.PermissionCollection;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants.SimGamePeiceConstants;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.robot.Constants.FishingRodConstants.states;

public class SimGamePeices extends SubsystemBase {

    public FishingRod fishingRod;
    public Swerve drivetrain;
    public Elevator elevator;
    public Wrist wrist;
    public Peice[] peices = new Peice[] {};
    public Coral[] corals = new Coral[] {};
    public Algae[] algaes = new Algae[] {};
    public boolean hasPeice = false;
    public Peice lastHeldPeice;
    public class Peice{
        public Pose3d pose;
        public boolean inRobot;
        public int peiceNumber;
        public StructPublisher<Pose3d> publisher;

        public Peice(int peiceNumber){
            publisher = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("SimGamePeices")
                .getStructTopic("Peice# " + peiceNumber, Pose3d.struct).publish();
        }


        public Pose3d getPose(){
            return pose;
        }

        public void setPose(Pose3d pose){
            this.pose = pose;
        }

        public boolean inRobot(){
            return inRobot;
        }

        public void setInRobot(boolean inRobot){
            this.inRobot = inRobot;
        }

        public int getNumber(){
            return peiceNumber;
        }

        public void publish(){
            publisher.set(pose);
        }

    }

    public class Coral extends Peice {

        public Coral(Pose3d pose){
            super(peices.length);
            setPose(pose);
        }

        public Coral(){
            super(peices.length);
            setPose(SimGamePeiceConstants.DefaultPose);
        }
    }

    public class Algae extends Peice {
        public Algae(Pose3d pose){
            super(peices.length);
            setPose(pose);
        }

        public Algae(){
            super(peices.length);
            setPose(SimGamePeiceConstants.DefaultPose);
        }
    }

  /** Creates a new SimGamePeices. */
    public SimGamePeices(Elevator elevator, Wrist wrist, FishingRod fishingRod, Swerve drivetrain, Peice[] peices) {

        this.fishingRod = fishingRod;
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.wrist = wrist;
        this.peices = peices;
  }

    public SimGamePeices(Elevator elevator, Wrist wrist, FishingRod fishingRod, Swerve drivetrain) {

        this.fishingRod = fishingRod;
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.wrist = wrist;
        
        addPeice(new Algae(SimGamePeiceConstants.A1B));
        addPeice(new Algae(SimGamePeiceConstants.A2B));
        addPeice(new Algae(SimGamePeiceConstants.A3B));
        addPeice(new Algae(SimGamePeiceConstants.A1R));
        addPeice(new Algae(SimGamePeiceConstants.A2R));
        addPeice(new Algae(SimGamePeiceConstants.A3R));
        addPeice(new Coral());
        corals[0].setInRobot(true);

        hasPeice = true;
        lastHeldPeice = corals[0];

        System.out.println("Peices added to SimGamePeices");
  }

    @Override
    public void periodic() {
        // collect();
        // drop();
        for (Peice peice : peices){
            peice.publish();
        }

        if(hasPeice){
            updateHeldPeicePose();
        }
    }

    public void addPeice(Peice peice){

        Peice[] newPeices = new Peice[peices.length + 1];
        for(int i = 0; i < peices.length; i++){
            newPeices[i] = peices[i];
        }
        newPeices[peices.length] = peice;
        peice.peiceNumber = peices.length;
        peices = newPeices;

        if (peice instanceof Coral){

            Coral[] newCorals = new Coral[corals.length + 1];
            for(int i = 0; i < corals.length; i++){
                newCorals[i] = corals[i];
            }
            newCorals[corals.length] = (Coral) peice;
            corals = newCorals;
            return;

        } else if (peice instanceof Algae){

            Algae[] newAlgae = new Algae[algaes.length + 1];

            for(int i = 0; i < algaes.length; i++){
                newAlgae[i] = algaes[i];
            }
            newAlgae[algaes.length] = (Algae) peice;
            algaes = newAlgae;
        }

    }

    public void removePeice(Peice peice){
        peice.pose = null;
        peice.inRobot = false;
    }

    // TODO: Add correct conditions for collecting & dropping
    public void collect(){

        for(Peice peice : peices){
            if(new Translation2d(peice.pose.getX(), peice.pose.getY())
                .getDistance(drivetrain.getPose().getTranslation()) < 0.5
                && Math.abs(elevator.getPosition() - peice.pose.getTranslation().getZ()) < 0.5){
                    peice.inRobot = true;
                    hasPeice = true;
                    lastHeldPeice = peice;
            }
        }
    }

    public void drop(){
        if(hasPeice && fishingRod.getState() != states.STOW){
            lastHeldPeice.inRobot = false;
            lastHeldPeice = null;
            hasPeice = false;
        }
    }

    public void forceDrop(){

        if(hasPeice){
            lastHeldPeice.inRobot = false;
            hasPeice = false;
        }
    }

    public void forceCollect(){

        double minDist = 10000;
        Peice closestPeice = null;

        if (!hasPeice){

            for(Peice peice : peices){
                double dist = new Translation2d(peice.pose.getX(), peice.pose.getY())
                    .getDistance(drivetrain.getPose().getTranslation());

                if(dist < minDist){
                    closestPeice = peice;
                    minDist = dist;
                }
            }
            closestPeice.inRobot = true;  
            hasPeice = true;
            lastHeldPeice = closestPeice;
        }
    }

    public void dropOrCollect(){
        if (hasPeice){
            forceDrop();

            System.out.println("Peice Dropped");
        } else {
            System.out.println("Peice Collected");
            forceCollect();
            
        }
    }

    public void updateHeldPeicePose(){
        lastHeldPeice.pose = new Pose3d(drivetrain.getPose().getX(), drivetrain.getPose().getY(), elevator.getPosition(), 
            new Rotation3d(wrist.getAngle(), 0, drivetrain.getPose().getRotation().getDegrees()));
    }
}