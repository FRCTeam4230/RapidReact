// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Climber;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetClimbersFull extends ParallelCommandGroup {
  /** Creates a new ResetClimbersFull. */
  public ResetClimbersFull(Set<ClimberSubsystem> climbers) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    climbers.forEach(climber -> {
      addCommands(new SequentialCommandGroup(
          new WaitUntilCommand(() -> climber.getPosition() > Climber.testRotations * Climber.resetDirection)
              .deadlineWith(new MoveClimbersToBottom(climber)),

          new ConditionalCommand(
              new InstantCommand(),
              new InstantCommand(climber::invert, climber).andThen(new MoveClimbersToBottom(climber)),
              climber::atLimit),

          new InstantCommand(climber::resetEncoder, climber)));
    });
  }
}
