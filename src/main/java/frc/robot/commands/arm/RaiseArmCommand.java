// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;

public class RaiseArmCommand extends AbstractArmCommand {
  /** Creates a new IntakeCommand. */
  public RaiseArmCommand(ArmSubsystem armSubsystem) {
    super(armSubsystem, Direction.UP);
  }
}
