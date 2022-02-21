// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import frc.robot.subsystems.IntakeSubsystem;

public class ExtakeCommand extends AbstractIntakeCommand {
  /** Creates a new IntakeCommand. */
  public ExtakeCommand(IntakeSubsystem intakeSubsystem) {
    super(intakeSubsystem, Direction.OUT);
  }
}
