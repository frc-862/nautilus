// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDStates;
import frc.robot.Constants.RobotMap;
import frc.thunder.hardware.Thunderbolt;

public class LEDs extends Thunderbolt {
	public LEDs() {
		super(LEDConstants.PWM_PORT, LEDConstants.LENGTH, RobotMap.UPDATE_FREQ);
	}

	@Override
	public void updateLEDs(LEDStates state) {
		switch (state) {

			case DISABLED -> setSolidHSV(0, 0, 0);

			case RAINBOW -> rainbow();

			case ALIGNING -> pulse(LEDConstants.BLUE_HUE);

			case ROD_MOVING -> pulse(LEDConstants.YELLOW_HUE);

			case ALGAE_COLLECT -> blink(LEDConstants.LIGHT_BLUE_HUE);

			case ALGAE_SCORE -> pulse(LEDConstants.LIGHT_BLUE_HUE);

			case CORAL_COLLECT -> blink(LEDConstants.PURPLE_HUE);

			case CORAL_SCORE -> pulse(LEDConstants.PURPLE_HUE);

			case MIXER -> {}
		}
	}

	@Override
	protected void defaultLEDs() {
		swirl(LEDConstants.SWRIL_SEGMENT_SIZE);
	}
}