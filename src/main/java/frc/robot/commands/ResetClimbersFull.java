// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
    Set<Command> moveClimbers = new HashSet<>();
    ParallelCommandGroup moveClimbers;
    climbers.forEach(climber -> {
     moveClimbers.addCommands(new WaitUntilCommand((n)))
    } );

    for (int i = 0; i < climbers.length; i++) {
      final int index = i;
      moveClimbers[i] = new WaitUntilCommand(() -> climbers[index].getPosition() > Climber.testRotations)
          .deadlineWith(new MoveClimbersToBottom(climbers[i]));
    }
    addCommands(new ParallelCommandGroup(moveClimbers));

    final Command[] finishResetCommands = new Command[climbers.length];

    for (int i = 0; i < climbers.length; i++) {
      final int index = i;
      finishResetCommands[i] = new ConditionalCommand(
          new InstantCommand(),
          new InstantCommand(climbers[i]::invert, climbers[i]).andThen(new MoveClimbersToBottom(climbers[i])),
          () -> climbers[index].atLimit()).andThen(
              new InstantCommand(climbers[i]::resetEncoder, climbers[i]));
    }

    addCommands(new ParallelCommandGroup(finishResetCommands));
  }
}
