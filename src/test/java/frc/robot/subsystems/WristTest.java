// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.thunder.hardware.ThunderBird;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.WristConstants;
/** Add your docs here. */
public class WristTest implements AutoCloseable {

    ThunderBird wristMotor;
    TalonFXSimState simWristMotor;
    Wrist wrist;
    @BeforeEach
    void constructMotors() {
        assert HAL.initialize(500, 0); 


        wristMotor = new ThunderBird(RobotMap.WRIST, RobotMap.CANIVORE_CAN_NAME, WristConstants.INVERTED, WristConstants.STATOR_CURRENT_LIMIT, WristConstants.BRAKE_MODE);
        simWristMotor = wristMotor.getSimState();

        wrist = new Wrist(wristMotor);

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
        wristMotor.close();
    }

    @Test
    public void testSetPower() { 
        var dutyCycle = wristMotor.getDutyCycle();
        
        wrist.setPower(0);
        
        dutyCycle.waitForUpdate(0.1);

        assertEquals(0, dutyCycle.getValue());
    }
}
