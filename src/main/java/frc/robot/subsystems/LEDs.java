// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.PriorityQueue;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.LEDConstants.LED_STATES;

public class LEDs extends SubsystemBase {
	AddressableLED leds;
	AddressableLEDBuffer ledBuffer;
	LED_STATES state;
	PriorityQueue<LED_STATES> ledStates;
	ScheduledExecutorService scheduler;
	

	public LEDs() {
		leds = new AddressableLED(LEDConstants.LED_PWM_PORT);
		ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_LENGTH);
		
		leds.setLength(ledBuffer.getLength());
		leds.start();

		ledStates = new PriorityQueue<>();
		ledStates.add(LED_STATES.OFF);

		scheduler = Executors.newSingleThreadScheduledExecutor();
		scheduler.scheduleAtFixedRate(this::updateLEDs, 0, (long) (RobotMap.UPDATE_FREQ * 100), java.util.concurrent.TimeUnit.MILLISECONDS);

	}

	@Override
	public void periodic() {}

	/**
	 * Updates the LEDs based on the current state
	 */
	private void updateLEDs() {
		state = ledStates.peek();

		switch (state) {
			case OFF -> swirl(LEDConstants.SWRIL_SEGMENT_SIZE);

			case DISABLED -> setSolidHSV(0, 0, 0);

			case RAINBOW -> rainbow();

			case ALIGNING -> pulse(LEDConstants.BLUE_HUE);

			case ROD_MOVING -> pulse(LEDConstants.YELLOW_HUE);

			case ALGAE_COLLECT -> blink(LEDConstants.LIGHT_BLUE_HUE);

			case ALGAE_SCORE -> pulse(LEDConstants.LIGHT_BLUE_HUE);

			case CORAL_COLLECT -> blink(LEDConstants.PURPLE_HUE);

			case CORAL_SCORE -> pulse(LEDConstants.PURPLE_HUE);

			case MIXER -> {
			}
		}

		leds.setData(ledBuffer);
	}

	/**
	 * @param state the state to enable
	 * @return a command that enables the state
	 */
	public Command enableState(LED_STATES state) {
		return new StartEndCommand(() -> {
			ledStates.add(state);
		},
		() -> {
			ledStates.remove(state);
		}).ignoringDisable(true);
	}

	/**
	 * Sets the LED buffer to a rainbow pattern
	 */
	private void rainbow() {
		for (int i = 0; i < LEDConstants.LED_LENGTH; i++) {
			ledBuffer.setHSV(i, (i + (int) (Timer.getFPGATimestamp() * 20)) % ledBuffer.getLength() * 180 / 14, 255,
					100);
		}
	}

	/**
	 * @param segmentSize size of each color segment
	 */
	private void swirl(int segmentSize) {
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
	private void blink(int hue) {
		if ((int) (Timer.getFPGATimestamp() * 10) % 2 == 0) {
			setSolidHSV(hue, 255, 255);
		} else {
			setSolidHSV(0, 0, 0);
		}
	}

	/**
	 * @param hue the hue to blink
	 */
	private void pulse(int hue) {
		setSolidHSV(hue, 255, (int) Math.abs((Math.sin(Timer.getFPGATimestamp() * 2) * 255)));
	}

	/**
	 * @param h hue
	 * @param s saturation
	 * @param v value
	 */
	private void setSolidHSV(int h, int s, int v) {
		for (var i = 0; i < LEDConstants.LED_LENGTH; i++) {
			ledBuffer.setHSV(i, h, s, v);
		}
	}
}