// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Climber;
import frc.robot.subsystems.ClimberSubsystem;

public class ResetClimbers extends CommandBase {
  private final ClimberSubsystem leftClimber;
  private final ClimberSubsystem rightClimber;

  private boolean leftDone = false;
  private boolean rightDone = false;

  /** Creates a new ResetClimbers. */
  public ResetClimbers(ClimberSubsystem left, ClimberSubsystem right) {
    // Use addRequirements() here to declare subsystem dependencies.

    leftClimber = left;
    rightClimber = right;

    addRequirements(left, right);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftClimber.setSpeed(Climber.speed * Climber.resetDirection);
    rightClimber.setSpeed(Climber.speed * Climber.resetDirection);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!leftDone && leftClimber.atLimit()) {
      leftDone = true;
      leftClimber.resetEncoder();
    }
    if (!rightDone && rightClimber.atLimit()) {
      rightDone = true;
      rightClimber.resetEncoder();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rightDone && leftDone;
  }
}
