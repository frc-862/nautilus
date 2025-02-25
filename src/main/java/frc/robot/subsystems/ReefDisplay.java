// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ReefDisplay extends SubsystemBase {

    CoralCollector collector;
    FishingRod rod;
    Swerve drivetrain;
    Pose2d robotPose;

    Tuple<Camera, Integer> targetReefArm;
    int tagNum;
    int arrayIndex;

    Boolean[][] reef = new Boolean[][] {new Booean[] {false, false, false, false, false, false, false, false, false, false, false, false},
        new Booean[] {false, false, false, false, false, false, false, false, false, false, false, false},
        new Booean[] {false, false, false, false, false, false, false, false, false, false, false, false}}; // iner = level; outer = tag

    public ReefDisplay(CoralCollector coralCollector, FishingRod rod, Swerve drivetrain) {
        this.collector = coralCollector;
        this.rod = rod;
        this.drivetrain = drivetrain;

        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("ReefDisplay").add("L1", reef[0]);
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("ReefDisplay").add("L2", reef[1]);
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("ReefDisplay").add("L3", reef[2]);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

    public void updateReef(){

        if (targetReefArm == null || tagNum == null){
            return;
        }

        if (collector.getVelocity() > 0){
            switch(rod.getState()){

                case L2:
                    reef[0][arrayIndex] = true;
                    break;

                case L3:
                    reef[1][arrayIndex] = true;
                    break;

                case L4:
                    reef[2][arrayIndex] = true;
                    break;
            }

            publish();
        }
    }

    public void updateScorePose(Tuple<Camera, Integer> targetReefArm){
        this.targetReefArm = targetReefArm;
        tagNum = targetReefArm.getSecond();
        setArrayIndex(tagNum, targetReefArm.getFirst());
    }

    public void publish(){
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("ReefDisplay").getEntry("L2").setBooleanArray(reef[0]);
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("ReefDisplay").getEntry("L3").setBooleanArray(reef[1]);
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("ReefDisplay").getEntry("L4").setBooleanArray(reef[2]);
    }

    public void setArrayIndex(int tagNum, Camera camera){
        int sideNum;

        switch(tagNum){
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

        arrayIndex = sideNum * 2 + (camera == Camera.LEFT ? 1 : 0);
    }

}
