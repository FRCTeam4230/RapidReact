// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.ArmSubsystem;

/** Add your docs here. */
public abstract class AbstractArmCommand extends CommandBase {
  protected enum Direction {
    DOWN(Intake.downArmSpeed), UP(Intake.upArmSpeed);

    private double value;

    private Direction(double value) {
      this.value = value;
    }

    public double getValue() {
      return this.value;
    }
  }

  private final Direction mDirection;
  private final ArmSubsystem armSubsystem;

  protected AbstractArmCommand(ArmSubsystem armSubsystem, Direction direction) {
    this.armSubsystem = armSubsystem;
    mDirection = direction;

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.setSpeed(mDirection.getValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return mDirection == Direction.DOWN ? armSubsystem.isDown() : armSubsystem.isUp();
  }
}
