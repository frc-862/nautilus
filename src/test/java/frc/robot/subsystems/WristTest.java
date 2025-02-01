// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.derive;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.thunder.hardware.ThunderBird;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.RobotMotors;
import frc.robot.Constants.WristConstants;

public class WristTest implements AutoCloseable {

    ThunderBird motor;
    TalonFXSimState simMotor;

    Wrist wrist;

    @BeforeEach
    void constructMotors() {
        assert HAL.initialize(500, 0); 

        motor = RobotMotors.wristMotor;
        simMotor = motor.getSimState();

        if(wrist == null) {
            wrist = new Wrist(motor);
        }

        HAL.simPeriodicBefore();
        DriverStationSim.setEnabled(true);
        HAL.simPeriodicAfter();
        DriverStationSim.notifyNewData();

        Timer.delay(0.100);
    }

    @AfterEach
    void shutdown() {
        close();
    }

    @Override
    public void close() {
        motor.close();
    }

    @Test
    public void testSetPower() { 
        var dutyCycle = motor.getDutyCycle();
        
        wrist.setPower(0);
        
        dutyCycle.waitForUpdate(0.1);

        assertEquals(0, dutyCycle.getValue());
    }

    @Test
    public void testSetPostion() {
        // Create a duty cycle and target position
        var dutyCycle = motor.getDutyCycle();
        var targetPos = -20;

        // Create variables for the timer
        var timer = new Timer();
        var timeOut = 1;

        // Set the inital position
        wrist.setPosition(targetPos);
        Timer.delay(0.1d);

        // Initally update the robot motors and simulation
        wrist.simulationPeriodic();
        dutyCycle.waitForUpdate(0.1);
        System.out.println(wrist.getAngle());

        // Create timer and start it
        timer.restart();
        timer.start();
        
        /*
         * Create a loop that will continue to update the simulation as long as the the position hasen't
         * reached the target position and the time hasen't run out
         */
        while (!(Math.abs(targetPos - wrist.getAngle()) <= WristConstants.TOLERANCE) && !timer.hasElapsed(timeOut)) {
            wrist.simulationPeriodic();
            dutyCycle.waitForUpdate(0.1);

            if (Math.round(wrist.getAngle()) % 10 == 0) {
                System.out.println(wrist.getAngle());
            }
        }
        
        // Print the final position and check the wrist position is equal to the target position
        System.out.println(wrist.getAngle() + "\n");
        assertEquals(targetPos, wrist.getAngle(), WristConstants.TOLERANCE);
    }
}
