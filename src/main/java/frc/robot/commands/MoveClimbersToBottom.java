// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Climber;
import frc.robot.subsystems.ClimberSubsystem;

public class MoveClimbersToBottom extends CommandBase {
  private final ClimberSubsystem climberSubsystem;

  /** Creates a new ResetClimbers. */
  public MoveClimbersToBottom(ClimberSubsystem climberSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.climberSubsystem = climberSubsystem;

    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climberSubsystem.setSpeed(Climber.speed * Climber.resetDirection * 0.8);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climberSubsystem.atLimit();
  }
}
