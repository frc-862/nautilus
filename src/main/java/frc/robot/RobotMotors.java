// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.thunder.hardware.ThunderBird;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.WristConstants;

/** Add your docs here. */
public class RobotMotors {
    static public ThunderBird wristMotor = new ThunderBird(RobotMap.WRIST, 
        RobotMap.CANIVORE_CAN_NAME, WristConstants.INVERTED, WristConstants.STATOR_CURRENT_LIMIT, WristConstants.BRAKE_MODE);
}
