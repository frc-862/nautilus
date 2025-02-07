// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FishingRodConstants.states;
import frc.robot.subsystems.FishingRod;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetRodState extends Command {
  private final FishingRod rod;
  private final states state;

  
  /** Creates a new SetRodState. 
   * @param rod rod subsystem
   * @param state state to set
  */
  public SetRodState(FishingRod rod, states state) {
    this.rod = rod;
    this.state = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rod.setState(state);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rod.onTarget();
  }
}
