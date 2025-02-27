// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.RobotIdentifiers;
import frc.robot.Constants.LEDConstants.LEDStates;
import frc.thunder.leds.ThunderStrip;
import frc.thunder.leds.Thunderbolt;
import frc.robot.Constants.RobotMap;
import frc.thunder.leds.LightningColors;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class LEDs extends Thunderbolt {
	// LED testing
	SendableChooser<LEDStates> ledChooser;

	public boolean pdhEnabled = true;
	// private boolean pdhSim = false;

	public final ThunderStrip strip = new ThunderStrip(LEDConstants.LENGTH, 0, leds) {
		@Override
		public void updateLEDs(LEDStates state) {
			switch (state) {
				case MIXER -> {
					LEDStates testState = getTestState();
					if (testState != null) {
						updateLEDs(testState);
					}
				}

				case ERROR -> blink(LightningColors.RED);
				
				case COLLECTED -> blink(LightningColors.GREEN);

				case ALIGNED -> blink(LightningColors.GREEN);

				case ALGAE_MODE -> solid(LightningColors.LIGHT_BLUE);

				case CLIMBED -> rainbow();

				case ALIGNING -> pulse(LightningColors.BLUE);

				case COLLECTING -> pulse(LightningColors.PURPLE);

				case SCORING -> pulse(LightningColors.PURPLE);

				case READY_TO_ALIGN -> pulse(LightningColors.ORANGE);

				case ROD_MOVING -> solid(LightningColors.PINK);

				case UPDATING_POSE -> blink(LightningColors.YELLOW);

				case POSE_BAD -> solid(LightningColors.RED);

				case SWIRL -> swirl(LightningColors.BLUE, LightningColors.ORANGE, LEDConstants.SWRIL_SEGMENT_SIZE);

				case RAINBOW -> rainbow();

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

		ledChooser = new SendableChooser<LEDStates>() {{
			addOption("None", null);
			for (LEDStates state : LEDStates.values()) {
				if (state != LEDStates.MIXER) {
					addOption(state.name(), state);
				}
			}
			setDefaultOption("None", null);
		}};
		LightningShuffleboard.send("LEDs", "Test State", ledChooser);

		// pdh = new PowerDistribution(1, ModuleType.kRev);

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

	public LEDStates getTestState() {
		return ledChooser.getSelected();
	}
	
	@Override
	public void periodic() {
		// if (pdh.getSwitchableChannel()) {
		// 	pdh.setSwitchableChannel(true);	
		// }
		LEDStates state = strip.getState();
		if (state != null) {
			LightningShuffleboard.setString("LEDs", "Current State", state.toString());
		} else {
			LightningShuffleboard.setString("LEDs", "Current State", "DEFAULT");
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
