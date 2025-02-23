// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.RobotIdentifiers;
import frc.robot.Constants.LEDConstants.LEDStates;
import frc.thunder.leds.ThunderStrip;
import frc.thunder.leds.Thunderbolt;
import frc.thunder.shuffleboard.LightningShuffleboard;
import frc.robot.Constants.RobotMap;
import frc.thunder.leds.LightningColors;

public class LEDs extends Thunderbolt {

	public boolean pdhEnabled = true;
	private boolean pdhSim = false;

	public final ThunderStrip strip = new ThunderStrip(LEDConstants.LENGTH, 0, leds) {
		@Override
		public void updateLEDs(LEDStates state) {
			switch (state) {
				case COLLECTED -> pulse(LightningColors.GREEN);

				case SCORED -> pulse(LightningColors.GREEN);

				case ALIGNING -> blink(LightningColors.BLUE);

				case COLLECTING -> blink(LightningColors.YELLOW);

				case SCORING -> pulse(LightningColors.YELLOW);

				case ROD_MOVING -> pulse(LightningColors.PINK);

				case UPDATING_POSE -> pulse(LightningColors.YELLOW);

				case POSE_BAD -> solid(LightningColors.RED);

				default -> System.err.println("Unexpected State Found: " + state);
			}
		}

		@Override
		public void defaultLEDs() {
			swirl(LightningColors.BLUE, LightningColors.ORANGE, LEDConstants.SWRIL_SEGMENT_SIZE);
		}
	};

	public LEDs() {
		super(LEDConstants.PWM_PORT, LEDConstants.LENGTH, RobotMap.UPDATE_FREQ);

		addStrip(strip);
	}

	/**
	 * blinks the PDH leds with a given rate
	 * @param pdh power distribution to switch
	 * @param tps ticks per second
	 */
	public void pdhLedsBlink(PowerDistribution pdh, double tps) {
		if (Constants.ROBOT_IDENTIFIER != RobotIdentifiers.NAUTILUS) {
			return;
		}

		// 2 decimals of precision when checking time in seconds
		if ((int)(Timer.getFPGATimestamp() * 100) % (100 * tps) <= 1 && pdhEnabled) {
			pdh.setSwitchableChannel(!pdh.getSwitchableChannel());
			// pdhSim = !pdhSim;
			// LightningShuffleboard.setBool("PDH", "on", pdhSim);
		}
	}

	/**
	 * sets the PDH leds to on
	 * @param pdh
	 */
	public void pdhLedsSolid(PowerDistribution pdh) {
		if (Constants.ROBOT_IDENTIFIER != RobotIdentifiers.NAUTILUS) {
			return;
		}

		pdh.setSwitchableChannel(true);
		// pdhSim = pdhEnabled;
		// LightningShuffleboard.setBool("PDH", "on", pdhSim);
	}

	/**
	 * Turns on or off the PDH leds (ignores any blink states)
	 * @param pdh
	 * @return command to use
	 */
	public Command togglePdh(PowerDistribution pdh) {
		if (Constants.ROBOT_IDENTIFIER != RobotIdentifiers.NAUTILUS) {
			return new InstantCommand();
		}

		return new InstantCommand(() -> {
			pdhEnabled = !pdhEnabled;
			pdh.setSwitchableChannel(pdhEnabled);
		}).ignoringDisable(true);
	}
}
