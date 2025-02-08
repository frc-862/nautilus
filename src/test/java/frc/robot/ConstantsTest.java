// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

/** Add your docs here. */
public class ConstantsTest {
    @Test
    public void testConstantsTests() {
        assertEquals("cam1", Constants.VisionConstants.leftCamName, "camera1Name should be 'cam1'");
    }
}
