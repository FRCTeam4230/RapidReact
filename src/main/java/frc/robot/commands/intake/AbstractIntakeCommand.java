// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.IntakeSubsystem;

/** Add your docs here. */
public abstract class AbstractIntakeCommand extends CommandBase {
  protected enum Direction {
    IN(Intake.speed), OUT(-Intake.speed);

    private double value;

    private Direction(double value) {
      this.value = value;
    }

    public double getValue() {
      return this.value;
    }
  }

  private final Direction mDirection;
  private final IntakeSubsystem intakeSubsystem;

  protected AbstractIntakeCommand(IntakeSubsystem intakeSubsystem, Direction direction) {
    this.intakeSubsystem = intakeSubsystem;
    mDirection = direction;

    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.setSpeed(mDirection.getValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
  }

  // this will have stop stop when we there are no balls left
  @Override
  public boolean isFinished() {
    return false;
  }
}
