// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeCollectorConstants.PIVOT_STATES;
import frc.robot.subsystems.AlgaeCollector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CollectAlgae extends Command {
	private AlgaeCollector collector;
	private PIVOT_STATES state;

	public CollectAlgae(AlgaeCollector collector, PIVOT_STATES state) {
		
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
	return false;
	}
}
