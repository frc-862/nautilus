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
public class AlgaeCollectorTest implements AutoCloseable {

    ThunderBird rollerMotor;
    TalonFXSimState simRollerMotor;

    ThunderBird pivotMotor;
    TalonFXSimState simPivotMotor;

    Tusks algaeCollector;

    @BeforeEach
    void constructMotors() {
        assert HAL.initialize(500, 0);

        rollerMotor = RobotMotors.algaeCollectorRollerMotor;
        simRollerMotor = rollerMotor.getSimState();

        pivotMotor = RobotMotors.algaeCollectorPivotMotor;
        simPivotMotor = pivotMotor.getSimState();

        algaeCollector = new Tusks(rollerMotor, pivotMotor);

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
        rollerMotor.close();
        pivotMotor.close();
    }

    @Test
    public void testPivotInitialPosition() {
        assertEquals(0, algaeCollector.getPivotAngle(), 0.1);
    }

    @Test
    public void testRollerSetPower() {
        var dutyCycle = rollerMotor.getDutyCycle();

        algaeCollector.setRollerPower(0);

        dutyCycle.waitForUpdate(0.1);

        assertEquals(0, dutyCycle.getValue());
    }

    @Test
    public void testPivotSetPower() {
        var dutyCycle = pivotMotor.getDutyCycle();

        algaeCollector.setPivotPower(0);

        dutyCycle.waitForUpdate(0.1);

        assertEquals(0, dutyCycle.getValue());
    }
}
