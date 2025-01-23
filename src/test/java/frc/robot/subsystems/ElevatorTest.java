// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.junit.jupiter.api.AfterEach;
import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants.RobotMotors;
import frc.thunder.hardware.ThunderBird;
/** Add your docs here. */
public class ElevatorTest implements AutoCloseable {

    ThunderBird rightMotor;
    TalonFXSimState simRightMotor;

    ThunderBird leftMotor;
    TalonFXSimState simLeftMotor;

    Elevator elevator;
    @BeforeEach
    void constructMotors() {
        assert HAL.initialize(500, 0); 


        leftMotor = RobotMotors.leftElevatorMotor;
        simLeftMotor = leftMotor.getSimState();

        rightMotor = RobotMotors.rightElevatorMotor;
        simRightMotor = rightMotor.getSimState();

        elevator = new Elevator(leftMotor, rightMotor);

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
        leftMotor.close();
        rightMotor.close();
    }

    @Test
    public void testInitialPosition() { 
        assertEquals(0, elevator.getPosition(), 0.1);
    }

    @Test
    public void testSetPower() {
        var dutyCycle = leftMotor.getDutyCycle();

        elevator.setPower(0.2d);

        elevator.simulationPeriodic();
        dutyCycle.waitForUpdate(0.1);
        elevator.simulationPeriodic();
        dutyCycle.waitForUpdate(0.1);
        elevator.simulationPeriodic();
        dutyCycle.waitForUpdate(0.1);
        elevator.simulationPeriodic();
        dutyCycle.waitForUpdate(0.1);

        assertEquals(0.2, dutyCycle.getValue(), 0.05);
    }

    // TODO: Make a test for set position
}
