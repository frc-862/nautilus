// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.PriorityQueue;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LED_STATES;

public class LEDs extends SubsystemBase {
	AddressableLED leds;
	AddressableLEDBuffer ledBuffer;
	LED_STATES state;
	PriorityQueue<LED_STATES> ledStates;

	public LEDs() {
		leds = new AddressableLED(LEDConstants.LED_PWM_PORT);
		ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_LENGTH);
		leds.setLength(ledBuffer.getLength());
		leds.start();
		ledStates = new PriorityQueue<>();
		ledStates.add(LED_STATES.OFF);
	}

	@Override
	public void periodic() {
		Thread ledThread = new Thread(() -> updateLEDs());
		ledThread.start();

	}

	public void updateLEDs() {
		state = ledStates.peek();

		switch (state) {
			case OFF -> swirl(LEDConstants.SWRIL_SEGMENT_SIZE);

			case DISABLED -> setSolidHSV(0, 0, 0);

			case RAINBOW -> rainbow();

			case ROD_MOVING -> pulse(LEDConstants.YELLOW_HUE);

			case ROD_ON_TARGET -> setSolidHSV(LEDConstants.GREEN_HUE, 255, 255);

			case ALGAE_COLLECT -> blink(LEDConstants.PINK_HUE);

			case ALGAE_SCORE -> pulse(LEDConstants.LIGHT_BLUE_HUE);

			case CORAL_COLLECT -> blink(LEDConstants.PURPLE_HUE);

			case CORAL_SCORE -> pulse(LEDConstants.PURPLE_HUE);

			case MIXER -> {
			}
		}

		leds.setData(ledBuffer);
	}

	public Command enableState(LED_STATES state) {
		return new StartEndCommand(() -> {
			ledStates.add(state);
		},
		() -> {
			ledStates.remove(state);
		}).ignoringDisable(true);
	}

	public void rainbow() {
		for (int i = 0; i < LEDConstants.LED_LENGTH; i++) {
			ledBuffer.setHSV(i, (i + (int) (Timer.getFPGATimestamp() * 20)) % ledBuffer.getLength() * 180 / 14, 255,
					100);
		}
	}

	/**
	 * @param segmentSize size of each color segment
	 */
	public void swirl(int segmentSize) {
		for (int i = 0; i < LEDConstants.LED_LENGTH; i++) {
			if (((i + (int) (Timer.getFPGATimestamp() * 10)) / segmentSize) % 2 == 0) {
				ledBuffer.setHSV(i, LEDConstants.BLUE_HUE, 255, 255);
			} else {
				ledBuffer.setHSV(i, LEDConstants.ORANGE_HUE, 255, 255);
			}
		}
	}

	/**
	 * @param hue the hue to blink
	 */
	public void blink(int hue) {
		if ((int) (Timer.getFPGATimestamp() * 10) % 2 == 0) {
			setSolidHSV(hue, 255, 255);
		} else {
			setSolidHSV(0, 0, 0);
		}
	}

	/**
	 * @param hue the hue to blink
	 */
	public void pulse(int hue) {
		setSolidHSV(hue, 255, (int) Math.abs((Math.sin(Timer.getFPGATimestamp() * 2) * 255)));
	}

	public void setSolidHSV(int h, int s, int v) {
		for (var i = 0; i < LEDConstants.LED_LENGTH; i++) {
			ledBuffer.setHSV(i, h, s, v);
		}
	}
}