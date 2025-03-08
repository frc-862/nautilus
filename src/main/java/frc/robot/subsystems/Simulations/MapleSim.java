// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Simulations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import static edu.wpi.first.units.Units.Inches;

public class MapleSim extends SubsystemBase {

    private static SimulatedArena arena;

    // init publishers

    StructArrayPublisher<Pose3d> algaePublisher = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("MapleSim")
        .getStructArrayTopic("algae", Pose3d.struct).publish();
    
    StructArrayPublisher<Pose3d> coralPoses = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("MapleSim")
        .getStructArrayTopic("coral", Pose3d.struct).publish();

    StructPublisher<Pose2d> robotPublisher = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("MapleSim")
        .getStructTopic("robot", Pose2d.struct).publish();

    SwerveDriveSimulation drivetrainSim;

    IntakeSimulation collectorSim;

    Swerve drivetrain;
  
    public MapleSim(Swerve drivetrain) {

        this.drivetrain = drivetrain;

        arena = SimulatedArena.getInstance();
        arena.placeGamePiecesOnField();
    }

    @Override
    public void simulationPeriodic() {

        // publish game piece
        Pose3d[] algae = arena.getGamePiecesArrayByType("Algae");
        algaePublisher.accept(algae);

        Pose3d[] coral = arena.getGamePiecesArrayByType("Coral");
        coralPoses.accept(coral);
        
        robotPublisher.accept(drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose());
    }


}
