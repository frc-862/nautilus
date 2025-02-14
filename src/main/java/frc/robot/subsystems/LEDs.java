// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDStates;
import frc.thunder.leds.Lightningstrip;
import frc.thunder.leds.Thunderbolt;
import frc.robot.Constants.RobotMap;
import frc.thunder.leds.Lightningbolt;

public class LEDs extends Thunderbolt {
	public LEDs() {
		super(LEDConstants.PWM_PORT, LEDConstants.LENGTH, RobotMap.UPDATE_FREQ);

	}

	Lightningstrip strip1 = new Lightningstrip(0, 60) {
		@Override
		protected void updateLEDs(LEDStates state) {
			switch (state) {
				case DISABLED -> setSolidHSV(0, 0, 0);

				case RAINBOW -> rainbow();

				case ALIGNING -> pulse(Colors.BLUE.getHue());

				case ROD_MOVING -> pulse(Colors.PINK.getHue());

				case ALGAE_COLLECT -> blink(Colors.LIGHT_BLUE.getHue());

				case ALGAE_SCORE -> pulse(Colors.LIGHT_BLUE.getHue());

				case CORAL_COLLECT -> blink(Colors.PURPLE.getHue());

				case CORAL_SCORE -> pulse(Colors.PURPLE.getHue());

				case MIXER -> {}
			}
		}

		@Override
		protected void defaultLEDs() {
			swirl(LEDConstants.SWRIL_SEGMENT_SIZE);
		}

            @Override
            protected void updateLEDs(Lightningbolt leds, LEDStates state) {
                throw new UnsupportedOperationException("Not supported yet.");
            }

            @Override
            protected void defaultLEDs(Lightningbolt leds) {
                throw new UnsupportedOperationException("Not supported yet.");
            }
	};
}