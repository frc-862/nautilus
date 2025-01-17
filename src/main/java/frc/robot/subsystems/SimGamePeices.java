// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.PermissionCollection;

import edu.wpi.first.math.geometry.Pose3d;
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
    public Peice heldPeice;

    public class Peice{
        protected Pose3d pose;
        protected boolean inRobot;
        public double peiceNumber;

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

        public void publish(){
            LightningShuffleboard.setDoubleArray(this instanceof Coral ? "Corals" : "Algae", 
                "Peice# " + (peiceNumber), 
                () -> new double[] {pose.getX(), pose.getY(), pose.getZ(),
                Units.radiansToDegrees(pose.getRotation().getX()), 
                Units.radiansToDegrees(pose.getRotation().getY()),
                Units.radiansToDegrees(pose.getRotation().getZ())});

        }
    }

    public class Coral extends Peice {

        public Coral(Pose3d pose){
            setPose(pose);;
        }

        public Coral(){
            setPose(SimGamePeiceConstants.DefaultPose);
        }
    }

    public class Algae extends Peice {
        public Algae(Pose3d pose){
            setPose(pose);
        }

        public Algae(){
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

        System.out.println("Peices added to SimGamePeices");
  }

    @Override
    public void periodic() {
        // collect();
        // drop();
    }

    public void addPeice(Peice peice){

        Peice[] newPeices = new Peice[peices.length + 1];
        for(int i = 0; i < peices.length; i++){
            newPeices[i] = peices[i];
        }
        newPeices[peices.length] = peice;
        peice.peiceNumber = peices.length;
        peices = newPeices;

        peice.publish();

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
                    heldPeice = peice;
            }
        }
    }

    public void drop(){
        if(hasPeice = true && fishingRod.getState() != states.STOW){
            heldPeice.inRobot = false;
            heldPeice = null;
            hasPeice = false;
        }
    }

    public void forceDrop(){

        if(hasPeice = true){
            heldPeice.inRobot = false;
            heldPeice = null;
            hasPeice = false;
        }
    }

    public void forceCollect(){

        double minDist = 10000;
        Peice closestPeice = null;

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
        heldPeice = closestPeice;
    }

    public void dropOrCollect(){
        if (hasPeice = true){
            forceDrop();
        } else {
            forceCollect();
        }
    }
}