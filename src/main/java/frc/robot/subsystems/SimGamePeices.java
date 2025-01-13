// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.SimGamePeiceConstants;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.robot.Constants.FishingRodConstants.states;
import java.lang.Runnable;

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
    }

    public class Coral extends Peice {

        public Coral(Pose3d pose){
            this.pose = super.pose;
        }

        public Coral(){}
    }

    public class Algae extends Peice {
        public Algae(Pose3d pose){
            this.pose = super.pose;
        }

        public Algae(){}
    }

  /** Creates a new SimGamePeices. */
    public SimGamePeices(FishingRod fishingRod, Swerve drivetrain, Peice[] peices) {

        this.fishingRod = fishingRod;
        this.drivetrain = drivetrain;
        this.peices = peices;

        elevator = fishingRod.getElevator();
        wrist = fishingRod.getWrist();
  }

    public SimGamePeices(FishingRod fishingRod, Swerve drivetrain) {

        this.fishingRod = fishingRod;
        this.drivetrain = drivetrain;
        
        addPeice(new Algae(SimGamePeiceConstants.A1B));
        addPeice(new Algae(SimGamePeiceConstants.A2B));
        addPeice(new Algae(SimGamePeiceConstants.A3B));
        addPeice(new Algae(SimGamePeiceConstants.A1R));
        addPeice(new Algae(SimGamePeiceConstants.A2R));
        addPeice(new Algae(SimGamePeiceConstants.A3R));
        addPeice(new Coral());
        corals[0].setInRobot(true);
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
        peices = newPeices;

        if (peice instanceof Coral){

            LightningShuffleboard.setDoubleArray("Corals", "Peice# " + (peices.length - 1), 
                () -> new double[] {peice.pose.getX(), peice.pose.getY(), peice.pose.getZ(),
                Units.radiansToDegrees(peice.pose.getRotation().getX()), 
                Units.radiansToDegrees(peice.pose.getRotation().getY()),
                Units.radiansToDegrees(peice.pose.getRotation().getZ())});

            Coral[] newCorals = new Coral[corals.length + 1];
            for(int i = 0; i < corals.length; i++){
                newCorals[i] = corals[i];
            }
            newCorals[corals.length] = (Coral) peice;
            corals = newCorals;
            return;

        } else /*if (peice instanceof Algae)*/{

            LightningShuffleboard.setDoubleArray("Algae", "Peice# " + (peices.length - 2), 
                () -> new double[] {peice.pose.getX(), peice.pose.getY(), peice.pose.getZ(),
                Units.radiansToDegrees(peice.pose.getRotation().getX()), 
                Units.radiansToDegrees(peice.pose.getRotation().getY()),
                Units.radiansToDegrees(peice.pose.getRotation().getZ())});

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

    public static class DropOrCollect implements Runnable{

        private SimGamePeices simGamePeices;

        public DropOrCollect(SimGamePeices simGamePeices){
            this.simGamePeices = simGamePeices;
        }

        public void run(){

            if (simGamePeices.hasPeice = true){
                simGamePeices.forceDrop();
            } else {
                simGamePeices.forceCollect();
            }
        }
    }

    public static Runnable dropOrCollect(SimGamePeices simGamePeices){
        return new DropOrCollect(simGamePeices);
    }
}
