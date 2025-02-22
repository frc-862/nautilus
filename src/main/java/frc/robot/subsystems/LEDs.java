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
					if (testState != null) {updateLEDs(testState);}
				}

				case ERROR -> blink(LightningColors.RED);
				
				case COLLECTED -> pulse(LightningColors.GREEN);

				case ALIGNING -> blink(LightningColors.BLUE);

				case COLLECTING -> blink(LightningColors.YELLOW);

				case SCORING -> pulse(LightningColors.YELLOW);

				case READY_TO_ALIGN -> solid(LightningColors.BLUE);

				case ROD_MOVING -> pulse(LightningColors.PINK);

				case UPDATING_POSE -> pulse(LightningColors.GREEN);

				case POSE_BAD -> solid(LightningColors.YELLOW);

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
		// if (pdh.getSwitchableChannel()) {
		// 	pdh.setSwitchableChannel(true);	
		// }
	}
}