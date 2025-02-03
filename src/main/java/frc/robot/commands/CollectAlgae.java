// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeCollectorConstants.PIVOT_STATES;
import frc.robot.subsystems.AlgaeCollector;
import frc.robot.Constants.AlgaeCollectorConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CollectAlgae extends Command {
	private AlgaeCollector collector;
	private DoubleSupplier triggerPower;

	public CollectAlgae(AlgaeCollector collector, DoubleSupplier triggerPower) {
		this.collector = collector;
		this.triggerPower = triggerPower;

		addRequirements(collector);
	}

	@Override
	public void initialize() {
		collector.setPivotState(PIVOT_STATES.DEPLOYED);
	}

	@Override
	public void execute() {
		collector.setRollerPower(triggerPower.getAsDouble() * AlgaeCollectorConstants.ALGAE_ROLLER_SPEED);
	}

	@Override
	public void end(boolean interrupted) {
		collector.setPivotState(PIVOT_STATES.STOWED);
		collector.stop();
	}

	@Override
	public boolean isFinished() {
	return false;
	}
}
