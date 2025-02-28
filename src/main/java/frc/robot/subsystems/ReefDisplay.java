// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PoseConstants;
import frc.robot.Constants.VisionConstants.Camera;
import frc.thunder.util.Tuple;
import frc.thunder.shuffleboard.LightningShuffleboard;
import java.util.List;

public class ReefDisplay extends SubsystemBase {

    private CoralCollector collector;
    private FishingRod rod;
    private Swerve drivetrain;
    private Pose2d robotPose;

    private Tuple<Integer, Boolean> targetReefSide;

    // StructPublisher<Boolean[]> reefPublisher = new StructPublisher<Boolean[]>("ReefDisplay", Boolean[]);

    private BooleanArrayPublisher L2Pubil;
    private BooleanArrayPublisher L3Pubil;
    private BooleanArrayPublisher L4Pubil;

    private boolean[][] reef = new boolean[][] {new boolean[] {false, false, false, false, false, false, false, false, false, false, false, false},
        new boolean[] {false, false, false, false, false, false, false, false, false, false, false, false},
        new boolean[] {false, false, false, false, false, false, false, false, false, false, false, false}}; // iner = level; outer = tag

    public ReefDisplay(CoralCollector coralCollector, FishingRod rod, Swerve drivetrain) {
        this.collector = coralCollector;
        this.rod = rod;
        this.drivetrain = drivetrain;
        targetReefSide = new Tuple<Integer, Boolean>(drivetrain.reefTagToRobot(drivetrain.getPose()), false);

    }

    public class ReefDisplayUpdate extends Command {

        ReefDisplay reefDisplay;

        public ReefDisplayUpdate(ReefDisplay reefDisplay){
            this.reefDisplay = reefDisplay;

            addRequirements(reefDisplay);
        }
        
        @Override
        public void initialize(){
    
        }

        @Override
        public void execute(){
            reefDisplay.updateTargetReef();

            if (reefDisplay.collector.getVelocity() < 0 && reefDisplay.targetReefSide.k != 0){
                switch(reefDisplay.rod.getState()){
    
                    case L2:
                        reefDisplay.reef[0][reefDisplay.getArrayIndex()] = true;
                        break;
    
                    case L3:
                        reefDisplay.reef[1][reefDisplay.getArrayIndex()] = true;
                        break;
    
                    case L4:
                        reefDisplay.reef[2][reefDisplay.getArrayIndex()] = true;
                        break;

                    default:
                        break;

                }
                reefDisplay.publish();
            }
        }   

        @Override
        public boolean isFinished(){
            return false;
        }
    }


    @Override
    public void periodic() {

        // LightningShuffleboard.setDouble("ReefDisplay", "arrayIndex", getArrayIndex());
        // LightningShuffleboard.setDouble("ReefDisplay", "tagNum", targetReefSide.k);
        // LightningShuffleboard.setBool("ReefDisplay", "isRight", targetReefSide.v);
    }

    public void updateTargetReef(){
        int tagNum = drivetrain.reefTagToRobot(drivetrain.getPose());

        Pose2d rightPose = PoseConstants.poseHashMap.get(new Tuple<Camera, Integer>(Camera.RIGHT, tagNum));
        Pose2d leftPose = PoseConstants.poseHashMap.get(new Tuple<Camera, Integer>(Camera.LEFT, tagNum));

        try{
            if ((rightPose != null && leftPose != null) && drivetrain.getPose().nearest(List.of(rightPose, leftPose)) == rightPose) {
                targetReefSide = new Tuple<Integer, Boolean>(tagNum, false);
            }
            else{
                targetReefSide = new Tuple<Integer, Boolean>(tagNum, true);
            }
        } catch (Exception e){
            
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

            default:
                return 0;
        }


        return sideNum * 2 + (targetReefSide.v ? 1 : 0);
    }

}
