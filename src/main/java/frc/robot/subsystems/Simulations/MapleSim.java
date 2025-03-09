// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Simulations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve;

import org.ironmaple.simulation.SimulatedArena;

public class MapleSim extends SubsystemBase {

    private SimulatedArena arena;
    private Swerve drivetrain;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable shuffleboard = inst.getTable("Shuffleboard").getSubTable("MapleSim");

    private final StructArrayPublisher<Pose3d> algaePublisher = shuffleboard.getStructArrayTopic("algae", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> coralPoses = shuffleboard.getStructArrayTopic("coral", Pose3d.struct).publish();

    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();
    private final DoubleArraySubscriber robotSubscriber = table.getDoubleArrayTopic("robotPose").subscribe(new double[] {1, 1, 0});
  
    public MapleSim(Swerve drivetrain) {

        this.drivetrain = drivetrain;

        arena = SimulatedArena.getInstance();
        arena.placeGamePiecesOnField();

        fieldTypePub.accept("Field2d");
        fieldPub.accept(new double[] {1, 1, 0});
    }

    @Override
    public void simulationPeriodic() {

        // publish game piece
        Pose3d[] algae = arena.getGamePiecesArrayByType("Algae");
        algaePublisher.accept(algae);

        Pose3d[] coral = arena.getGamePiecesArrayByType("Coral");
        coralPoses.accept(coral);


        if (DriverStation.isDisabled()){
            // allow robot pose to be set through networktables
            double[] poseArray = robotSubscriber.get();
            Pose2d pose = new Pose2d(poseArray[0], poseArray[1], new Rotation2d(poseArray[2]));

            drivetrain.resetPose(pose);

        } else {
            // use actual simulated pose for field positioning instead of odometry pose
            fieldPub.accept(new double[] {drivetrain.getExactPose().getTranslation().getX(), 
                drivetrain.getExactPose().getTranslation().getY(), drivetrain.getExactPose().getRotation().getDegrees()});
        }
    }

}
