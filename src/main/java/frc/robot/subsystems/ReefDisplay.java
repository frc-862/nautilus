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

    Boolean[][] reef = new Boolean[][] {new Booean[] {false, false, false, false, false, false, false, false, false, false, false, false},
        new Booean[] {false, false, false, false, false, false, false, false, false, false, false, false},
        new Booean[] {false, false, false, false, false, false, false, false, false, false, false, false}}; // iner = level; outer = tag

    public ReefDisplay(CoralCollector coralCollector, FishingRod rod, Swerve drivetrain) {
        this.collector = coralCollector;
        this.rod = rod;
        this.drivetrain = drivetrain;
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
                  if (tagNum == 18 || tagNum == 7){
                      if(targetReefArm.getFirst() == Camera.LEFT){
                          reef[0][0] = true;
                      }

                      if(targetReefArm.getFirst() == Camera.RIGHT){
                          reef[0][1] = true;
                      }
                  }
                  break;

                case L3:
                    if (tagNum == 17 || tagNum == 8){
                        if(targetReefArm.getFirst() == Camera.RIGHT){
                            reef[1][0] = true;
                        }

                        if(targetReefArm.getFirst() == Camera.LEFT){
                            reef[1][1] = true;
                        }
                    }
                    break;

                case L4:
                    break;
            }
        }
    }

    public void updateScorePose(Tuple<Camera, Integer> targetReefArm){
        this.targetReefArm = targetReefArm;
        tagNum = targetReefArm.getSecond();
    }

    public void publish(){
        NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("ReefDisplay").getEntry("L1").setBooleanArray(reef[0]);
    }

}
