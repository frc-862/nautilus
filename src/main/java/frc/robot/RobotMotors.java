// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.WristConstants;
import frc.thunder.hardware.ThunderBird;

/** Add your docs here. */
public class RobotMotors {
    public static ThunderBird wristMotor = new ThunderBird(RobotMap.WRIST, RobotMap.CANIVORE_CAN_NAME,
        WristConstants.INVERTED, WristConstants.STATOR_CURRENT_LIMIT, WristConstants.BRAKE_MODE);
    public static ThunderBird elevatorLeftMotor = new ThunderBird(RobotMap.L_ELEVATOR, RobotMap.CANIVORE_CAN_NAME, ElevatorConstants.L_INVERTED,
        ElevatorConstants.STATOR_CURRENT_LIMIT, ElevatorConstants.BRAKE_MODE);
    public static ThunderBird elevatorRightMotor = new ThunderBird(RobotMap.R_ELEVATOR, RobotMap.CANIVORE_CAN_NAME, ElevatorConstants.R_INVERTED,
        ElevatorConstants.STATOR_CURRENT_LIMIT, ElevatorConstants.BRAKE_MODE);
}
