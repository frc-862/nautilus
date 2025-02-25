// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDStates;
import frc.thunder.leds.ThunderStrip;
import frc.thunder.leds.Thunderbolt;
import frc.robot.Constants.RobotMap;
import frc.thunder.leds.LightningColors;

public class LEDs extends Thunderbolt {

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
}
