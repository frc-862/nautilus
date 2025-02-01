// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.DynamicContainer.dynamicContainer;
import static org.junit.jupiter.api.DynamicTest.dynamicTest;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants.RobotMotors;
import frc.thunder.hardware.ThunderBird;

public class CoralCollectorTest implements AutoCloseable {

    ThunderBird motor;
    TalonFXSimState simMotor;

    static CoralCollector coralCollector;

    @BeforeEach
    void constructMotors() {
        assert HAL.initialize(500, 0);

        motor = RobotMotors.coralCollectorMotor;
        simMotor = motor.getSimState();

        if (coralCollector == null) {
            coralCollector = new CoralCollector(motor);
        }

        HAL.simPeriodicBefore();
        DriverStationSim.setEnabled(true);
        HAL.simPeriodicAfter();
        DriverStationSim.notifyNewData();

        Timer.delay(0.1);
    }

    @AfterEach
    void shutDown() {
        close();
    }

    @Override
    public void close() {
        motor.close();
    }

    // @Test
    // public void testSetPower() {
    //     var dutyCycle = motor.getDutyCycle();

    //     // Checking power goes up
    //     coralCollector.setPower(0.2);
    //     Timer.delay(0.1);

    //     System.out.println(dutyCycle.getValueAsDouble());

    //     dutyCycle.waitForUpdate(0.1);
    //     coralCollector.simulationPeriodic();

    //     System.out.println(dutyCycle.getValueAsDouble());
        
    //     assertEquals(0.2, dutyCycle.getValue(), 0.05);

    //     // Checking power goes down
    //     coralCollector.setPower(0);
    //     Timer.delay(0.1);

    //     dutyCycle.waitForUpdate(0.1);
    //     coralCollector.simulationPeriodic();

    //     assertEquals(0, dutyCycle.getValue(), 0.05);
    // }   
    
    @Test
    public void testBeamBreak() {
        coralCollector.setSimBeamBreak(true);
        assertTrue(coralCollector.getBeamBreakOutput());

        coralCollector.setSimBeamBreak(false);
        assertFalse(coralCollector.getBeamBreakOutput());
    }
}