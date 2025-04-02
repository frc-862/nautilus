// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.ElevatorConstants;
// import frc.robot.subsystems.Elevator;

// public class OldElevatorSync extends Command {

//     private Elevator elevator;
//     private double error = ElevatorConstants.CANRANGE_TOLERANCE;
//     private double rangeDistance = 0;

//     public OldElevatorSync(Elevator elevator) {
//         this.elevator = elevator;

//         addRequirements(elevator);
//     }

//     @Override
//     public void initialize() {
//     }

//     @Override
//     public void execute() {
//         double position = elevator.getPosition();
//         rangeDistance = elevator.getCANRangeDist();
//         error = rangeDistance - position;
    
//         if (elevator.shouldSyncCANRange()) {
//             elevator.setPosition(position + error);
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         elevator.setEncoder(rangeDistance);
//         elevator.setPosition(elevator.getPosition());
//     }

//     @Override
//     public boolean isFinished() {
//         return Math.abs(error) < ElevatorConstants.CANRANGE_TOLERANCE;
//     }
// }
