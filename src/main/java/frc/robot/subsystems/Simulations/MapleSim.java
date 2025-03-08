// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Simulations;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.ironmaple.simulation.SimulatedArena;

public class MapleSim extends SubsystemBase {
  
    public MapleSim() {

    }

    @Override
    public void simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic();
        
    }


}
