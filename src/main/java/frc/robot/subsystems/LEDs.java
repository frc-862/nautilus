// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDStates;
import frc.thunder.leds.Lightningstrip;
import frc.thunder.leds.Thunderbolt;
import frc.robot.Constants.RobotMap;
import frc.thunder.leds.Colors;

public class LEDs extends Thunderbolt {
	public final Lightningstrip strip = new Lightningstrip(LEDConstants.LENGTH, 0, leds, LEDStates.DISABLED, LEDStates.RAINBOW, LEDStates.ALIGNING, LEDStates.ROD_MOVING, LEDStates.ALGAE_COLLECT, LEDStates.ALGAE_SCORE, LEDStates.CORAL_COLLECT, LEDStates.CORAL_SCORE, LEDStates.MIXER) {
		@Override
		public void updateLEDs(LEDStates state) {
			switch (state) {
				case DISABLED -> color(Colors.BLACK);

				case RAINBOW -> rainbow();

				case ALIGNING -> pulse(Colors.BLUE);

				case ROD_MOVING -> pulse(Colors.PINK);

				case ALGAE_COLLECT -> blink(Colors.LIGHT_BLUE);

				case ALGAE_SCORE -> pulse(Colors.LIGHT_BLUE);

				case CORAL_COLLECT -> blink(Colors.PURPLE);

				case CORAL_SCORE -> pulse(Colors.PURPLE);
				
				default -> throw new IllegalArgumentException("Unexpected value: " + state);

			}
		}

		@Override
		public void defaultLEDs() {
			swirl(Colors.BLUE, Colors.ORANGE, LEDConstants.SWRIL_SEGMENT_SIZE);
		}
	};

	public LEDs() {
		super(LEDConstants.PWM_PORT, LEDConstants.LENGTH, RobotMap.UPDATE_FREQ);

		addStrip(strip);

	}
}