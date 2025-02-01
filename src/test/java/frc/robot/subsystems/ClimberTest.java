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

public class ClimberTest implements AutoCloseable {

    ThunderBird motor;
    TalonFXSimState simMotor;

    Climber climber;

    @BeforeEach
    void constructMotors() {
        assert HAL.initialize(500, 0);

        motor = RobotMotors.climberMotor;
        simMotor = motor.getSimState();

        climber = new Climber(motor);

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
    public void testInitialPosition() {
        assertEquals(0, climber.getPostion());
    }

    @Test
    public void testSetPower() {
        var dutyCycle = motor.getDutyCycle();

        climber.setPower(0d);

        climber.simulationPeriodic();
        dutyCycle.waitForUpdate(0.1);

        System.out.println(dutyCycle.getValue());
        assertEquals(0, dutyCycle.getValue(), 0);
    }
}
