// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDStates;
import frc.thunder.leds.ThunderStrip;
import frc.thunder.leds.Thunderbolt;
import frc.robot.Constants.RobotMap;
import frc.thunder.leds.LightningColors;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class LEDs extends Thunderbolt {
	// LED testing
	SendableChooser<LEDStates> ledChooser;

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

				case ALGAE_MODE -> rainbow();

				case ALIGNING -> pulse(LightningColors.BLUE);

				case COLLECTING -> pulse(LightningColors.PURPLE);

				case SCORING -> pulse(LightningColors.PURPLE);

				case READY_TO_ALIGN -> pulse(LightningColors.ORANGE);

				case ROD_MOVING -> solid(LightningColors.PINK);

				case UPDATING_POSE -> blink(LightningColors.YELLOW);

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

	public LEDStates getTestState() {
		return ledChooser.getSelected();
	}
	
	@Override
	public void periodic() {
		LEDStates state = strip.getState();
		if (state != null) {
			LightningShuffleboard.setString("LEDs", "Current State", state.toString());
		} else {
			LightningShuffleboard.setString("LEDs", "Current State", "DEFAULT");
		}
	} 
}
