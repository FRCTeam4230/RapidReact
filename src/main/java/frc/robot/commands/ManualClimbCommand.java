// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Climber;
import frc.robot.subsystems.ClimberSubsystem;

public class ManualClimbCommand extends CommandBase {
  private final ClimberSubsystem climber;
  private final DoubleSupplier input;

  /** Creates a new ManualClimbCommand. */
  public ManualClimbCommand(ClimberSubsystem climberSubsystem, DoubleSupplier input) {
    climber = climberSubsystem;
    this.input = input;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.useBrake(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setSpeed(input.getAsDouble() * Climber.speed * (climber.getPosition() - Climber.highLimit > 0 ? 1 : -1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stop();
    climber.useBrake(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
