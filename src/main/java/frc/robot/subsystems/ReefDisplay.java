// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.VisionConstants.Camera;
import frc.thunder.util.Tuple;
import frc.thunder.shuffleboard.LightningShuffleboard;
import java.util.List;

public class ReefDisplay extends SubsystemBase {

    CoralCollector collector;
    FishingRod rod;
    Swerve drivetrain;
    Pose2d robotPose;

    Tuple<Integer, Boolean> targetReefSide;

    // StructPublisher<Boolean[]> reefPublisher = new StructPublisher<Boolean[]>("ReefDisplay", Boolean[]);

    BooleanArrayPublisher L2Pubil;
    BooleanArrayPublisher L3Pubil;
    BooleanArrayPublisher L4Pubil;

    boolean[][] reef = new boolean[][] {new boolean[] {false, false, false, false, false, false, false, false, false, false, false, false},
        new boolean[] {false, false, false, false, false, false, false, false, false, false, false, false},
        new boolean[] {false, false, false, false, false, false, false, false, false, false, false, false}}; // iner = level; outer = tag

    public ReefDisplay(CoralCollector coralCollector, FishingRod rod, Swerve drivetrain) {
        this.collector = coralCollector;
        this.rod = rod;
        this.drivetrain = drivetrain;
        targetReefSide = new Tuple<Integer, Boolean>(drivetrain.reefTagToRobot(drivetrain.getPose()), false);

        L2Pubil = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("ReefDisplay").getBooleanArrayTopic("L2").publish();
        L3Pubil = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("ReefDisplay").getBooleanArrayTopic("L3").publish();
        L4Pubil = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("ReefDisplay").getBooleanArrayTopic("L4").publish();
    }

    @Override
    public void periodic() {
        setArrayIndex();
        if (collector.getVelocity() < 0 && targetReefSide.k != 0){
            switch(rod.getState()){

                case L2:
                    reef[0][getArrayIndex()] = true;
                    break;

                case L3:
                    reef[1][getArrayIndex()] = true;
                    break;

                case L4:
                    reef[2][getArrayIndex()] = true;
                    break;
            }
        }

        publish();
        LightningShuffleboard.setDouble("ReefDisplay", "arrayIndex", getArrayIndex());
        LightningShuffleboard.setDouble("ReefDisplay", "tagNum", targetReefSide.k);
        LightningShuffleboard.setBool("ReefDisplay", "isRight", targetReefSide.v);

    }

    public void updateTargetReef(){
        int tagNum = drivetrain.reefTagToRobot(drivetrain.getPose());

        Pose2d rightPose = PoseConstants.poseHashMap.get(new Tuple<Camera, Integer>(Camera.RIGHT, tagNum));
        Pose2d leftPose = PoseConstants.poseHashMap.get(new Tuple<Camera, Integer>(Camera.LEFT, tagNum));


        if (rightPose != null && leftPose != null && drivetrain.getPose().nearest(List.of(rightPose, leftPose)) == rightPose) {
            targetReefSide = new Tuple<Integer, Boolean>(tagNum, false);
        }
        else{
            targetReefSide = new Tuple<Integer, Boolean>(tagNum, true);
        }
    }



    public void publish(){
        L2Pubil.accept(reef[0]);
        L3Pubil.accept(reef[1]);
        L4Pubil.accept(reef[2]);
    }

    public int getArrayIndex(){
        int sideNum = 0;

        switch(targetReefSide.k){
            case 7:
                sideNum = 0;
                break;

            case 18:
                sideNum = 0;
                break;

            case 8:
                sideNum = 1;
                break;

            case 17:
                sideNum = 1;
                break;

            case 9:
                sideNum = 2;
                break;

            case 22:
                sideNum = 2;
                break;

            case 10:
                sideNum = 3;
                break;

            case 21:
                sideNum = 3;
                break;
            
            case 11:
                sideNum = 4;
                break;

            case 20:
                sideNum = 4;
                break;

            case 6:
                sideNum = 5;
                break;

            case 19:
                sideNum = 5;
                break;
        }


        return sideNum * 2 + (targetReefSide.v ? 1 : 0);
    }

}
