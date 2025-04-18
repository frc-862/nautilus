// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import org.junit.jupiter.api.AfterEach;
// import static org.junit.jupiter.api.Assertions.assertEquals;
// import org.junit.jupiter.api.BeforeEach;
// import org.junit.jupiter.api.Test;

// import com.ctre.phoenix6.sim.TalonFXSimState;

// import edu.wpi.first.hal.HAL;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.simulation.DriverStationSim;
// import frc.robot.Constants.ElevatorConstants;
// import frc.robot.Constants.RobotMotors;
// import frc.robot.Constants.WristConstants;
// import frc.thunder.hardware.ThunderBird;

// public class ElevatorTest implements AutoCloseable {

//     ThunderBird rightMotor;
//     TalonFXSimState simRightMotor;

//     ThunderBird leftMotor;
//     TalonFXSimState simLeftMotor;

//     Elevator elevator;

//     @BeforeEach
//     void constructMotors() {
//         assert HAL.initialize(500, 0);

//         leftMotor = RobotMotors.leftElevatorMotor;
//         simLeftMotor = leftMotor.getSimState();

//         rightMotor = RobotMotors.rightElevatorMotor;
//         simRightMotor = rightMotor.getSimState();

//         if (elevator == null) {
//             elevator = new Elevator(leftMotor, rightMotor);
//         }

//         HAL.simPeriodicBefore();
//         DriverStationSim.setEnabled(true);
//         HAL.simPeriodicAfter();
//         DriverStationSim.notifyNewData();

//         Timer.delay(0.100);
//     }

//     @AfterEach
//     void shutdown() {
//         close();
//     }

//     @Override
//     public void close() {
//         leftMotor.close();
//         rightMotor.close();
//     }

//     @Test
//     public void testInitialPosition() {
//         assertEquals(0, elevator.getPosition(), 0.1);
//     }

//     @Test
//     public void testSetPower() {
//         var dutyCycle = leftMotor.getDutyCycle();

//         elevator.setPower(0d);
//         Timer.delay(0.1);

//         elevator.simulationPeriodic();
//         dutyCycle.waitForUpdate(0.1);

//         System.out.println(dutyCycle.getValue());
//         assertEquals(0, dutyCycle.getValue(), 0);
//     }

//     @Test
//     public void testSetPostion() {
//         // Create a duty cycle and target position
//         var dutyCycle = leftMotor.getDutyCycle();
//         var targetPos = 30;
//         var tolerance = 0.5d;

//         // Create variables for the timer
//         var timer = new Timer();
//         var timeOut = 2;

//         // Set the inital position
//         elevator.setPosition(targetPos);
//         Timer.delay(0.1d);

//         // Initally update the elevator motors and simulation
//         elevator.simulationPeriodic();
//         dutyCycle.waitForUpdate(0.1);
//         System.out.println(elevator.getPosition());

//         // Create timer and start it
//         timer.restart();
//         timer.start();

//         /*
//          * Create a loop that will continue to update the simulation as long as the the
//          * position hasen't
//          * reached the target position and the time hasen't run out
//          */
//         while (!(Math.abs(targetPos - elevator.getPosition()) <= tolerance) && !timer.hasElapsed(timeOut)) {
//             elevator.simulationPeriodic();
//             dutyCycle.waitForUpdate(0.1);

//             if (Math.round(elevator.getPosition()) % 10 == 0) {
//                 System.out.println(elevator.getPosition());
//             }
//         }

//         // Print the final position and check the elevator position is equal to the
//         // target position
//         System.out.println(elevator.getPosition() + "\n");
//         assertEquals(targetPos, elevator.getPosition(), tolerance);
//     }
// }
